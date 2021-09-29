//无人机追踪目标（此处先直接用目标位置进行追踪）
//无人机吊舱根据视觉算法的反馈调节角度，保证目标在相机中心位置
//ROS 头文件
#include <ros/ros.h>
#include <iostream>
#include "mission_utils.h"
#include "gimbal_control.h"
#include "message_utils.h"
#include <prometheus_msgs/PositionReference.h>
#include <prometheus_msgs/DroneState.h>

using namespace std;
using namespace Eigen;

#define NODE_NAME "gimbal_control_demo"
#define PI 3.1415926
#define VISION_THRES_TRACKING 100

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>全 局 变 量<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
struct _pid{
float SetSpeed;//定义设定值
float ActualSpeed;//定义实际值
float err;//定义偏差值
float err_last;//定义上一个偏差值
float Kp,Ki,Kd;//定义比例、积分、微分系数
float voltage;//定义电压值(控制执行器的变量)
float integral;//定义积分值
float umax;//执行机构可执行的最大值
float umin;//执行机构可执行的最小值
float index_state; //抗积分饱和
};

_pid pid1;
_pid pid2;



float start_point[3];    // 起始降落位置
Eigen::Vector3d gimbal_att_deg;
Eigen::Vector3d gimbal_att;
//---------------------------------------Drone---------------------------------------------
prometheus_msgs::DroneState _DroneState;    // 无人机状态
Eigen::Vector3d mav_pos_;
float attitude;

prometheus_msgs::ControlCommand Command_to_pub;                               //发送给控制模块 [px4_pos_controller.cpp]的命令

float state_desired[4];
int Move_mode = 0;  // 0 for XYZ_POS, 1 for XY_POS_Z_VEL, 2 for XY_VEL_Z_POS, 3 for XYZ_VEL, 5 for TRAJECTORY
int Move_frame = 0; //0 for ENU_FRAME, 1 for BODY_FRAME
bool is_detected = true;
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>　函数声明　<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
void printf_result();
void PID_init(_pid *pid,float max, float min);
float PID_realize(_pid *pid, float speed, float actual);
void generate_com(int Move_mode, float state_desired[4]);
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>回 调 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
void drone_state_cb(const prometheus_msgs::DroneState::ConstPtr& msg)
{
    _DroneState = *msg;
    mav_pos_ << _DroneState.position[0],_DroneState.position[1],_DroneState.position[2];
    attitude = _DroneState.attitude[2];
    if(attitude>=0)
      attitude = 180/3.14*attitude;
    else
      attitude = 180/3.14*(attitude+3.1415926)+180;
    cout << "attitude[2]" << attitude << endl;

}

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>主函数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
int main(int argc, char **argv)
{
    ros::init(argc, argv, "my_plane_control");
    ros::NodeHandle nh("~");
    PID_init(&pid1,0.5,0);
    PID_init(&pid2,500,-500);

    gimbal_control gimbal_control_;
    // 节点运行频率： 10hz 
    ros::Rate rate(0.5);

    bool moving_target;

    ros::Publisher message_pub = nh.advertise<prometheus_msgs::Message>("/prometheus/message/main", 10);

    //【订阅】无人机状态
    ros::Subscriber drone_state_sub = nh.subscribe<prometheus_msgs::DroneState>("/prometheus/drone_state", 3, drone_state_cb);

    //【发布】发送给控制模块 [px4_pos_controller.cpp]的命令
    ros::Publisher command_pub = nh.advertise<prometheus_msgs::ControlCommand>("/prometheus/control_command", 10);

    // 初始化命令 - Idle模式 电机怠速旋转 等待来自上层的控制指令
    Command_to_pub.Mode                                = prometheus_msgs::ControlCommand::Idle;
    Command_to_pub.Command_ID                          = 0;
    Command_to_pub.source = NODE_NAME;
    Command_to_pub.Reference_State.Move_mode           = prometheus_msgs::PositionReference::XYZ_POS;
    Command_to_pub.Reference_State.Move_frame          = prometheus_msgs::PositionReference::ENU_FRAME;
    Command_to_pub.Reference_State.position_ref[0]     = 0;
    Command_to_pub.Reference_State.position_ref[1]     = 0;
    Command_to_pub.Reference_State.position_ref[2]     = 0;
    Command_to_pub.Reference_State.velocity_ref[0]     = 0;
    Command_to_pub.Reference_State.velocity_ref[1]     = 0;
    Command_to_pub.Reference_State.velocity_ref[2]     = 0;
    Command_to_pub.Reference_State.acceleration_ref[0] = 0;
    Command_to_pub.Reference_State.acceleration_ref[1] = 0;
    Command_to_pub.Reference_State.acceleration_ref[2] = 0;
    Command_to_pub.Reference_State.yaw_ref             = 0;

    //固定的浮点显示
    cout.setf(ios::fixed);
    //setprecision(n) 设显示小数精度为n位
    cout<<setprecision(2);
    //左对齐
    cout.setf(ios::left);
    // 强制显示小数点
    cout.setf(ios::showpoint);
    
    int flag = 0;
    while(flag == 0)
    {
        cout << "Please enter the Z"<<endl;
        Move_frame = 0;
        Move_mode = 0;
        cin >> state_desired[2];
        state_desired[0] = 0;
        state_desired[1] = 0;
        state_desired[3] = 0;
        Command_to_pub.header.stamp = ros::Time::now();
        Command_to_pub.Mode = prometheus_msgs::ControlCommand::Move;
        Command_to_pub.Command_ID = Command_to_pub.Command_ID + 1;
        Command_to_pub.source = NODE_NAME;
        Command_to_pub.Reference_State.Move_mode  = Move_mode;
        Command_to_pub.Reference_State.Move_frame = Move_frame;
        Command_to_pub.Reference_State.time_from_start = -1;
        generate_com(Move_mode, state_desired);

        command_pub.publish(Command_to_pub);

        cout << "Please enter 1 to checking"<<endl;
        cout << "Or enter 0 to reset the Z"<<endl;
        cin >> flag;
    }


    Move_frame = 1;
    Move_mode = 2;

    while(ros::ok())
    {
    cout << "Please enter the state_desired[3]"<<endl;
    cin >> state_desired[3];
        
    Command_to_pub.header.stamp = ros::Time::now();
    Command_to_pub.Mode = prometheus_msgs::ControlCommand::Move;
    Command_to_pub.Command_ID = Command_to_pub.Command_ID + 1;
    Command_to_pub.source = NODE_NAME;
    Command_to_pub.Reference_State.Move_mode  = Move_mode;
    Command_to_pub.Reference_State.Move_frame = Move_frame;
    Command_to_pub.Reference_State.time_from_start = -1;
    generate_com(Move_mode, state_desired);
    command_pub.publish(Command_to_pub);

    ros::spinOnce();
    rate.sleep();
    }

    return 0;

}

void printf_result()
{
    //固定的浮点显示
    cout.setf(ios::fixed);
    //setprecision(n) 设显示小数精度为n位
    cout<<setprecision(2);
    //左对齐
    cout.setf(ios::left);
    // 强制显示小数点
    cout.setf(ios::showpoint);
    // 强制显示符号
    cout.setf(ios::showpos);

    cout << ">>>>>>>>>>>>>>>>>>>>>>Autonomous Landing Mission<<<<<<<<<<<<<<<<<<<"<< endl;

    cout << ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>Vision State<<<<<<<<<<<<<<<<<<<<<<<<<<" <<endl;
    if( is_detected)
    {
        cout << "is_detected: ture" <<endl;
    }else
    {
        cout << "is_detected: false" <<endl;
    }    
}

void PID_init(_pid *pid,float max, float min)
{
    cout<<"PID_init begin \n";
    pid->SetSpeed=0.0;
    pid->ActualSpeed=0.0;
    pid->err=0.0;
    pid->err_last=0.0;
    pid->voltage=0.0;
    pid->integral=0.0;
    pid->Kp=0.07;
    pid->Ki=0.0;
    pid->Kd=0.0;
    pid->umax= max;
    pid->umin= min;
    pid->index_state = 0;
    cout<<"PID_init end \n";
}

float PID_realize(_pid *pid, float target, float actual){
    pid->SetSpeed=target;
    //cout << "target" << target <<endl;
    pid->ActualSpeed = actual;
    //cout << "actual" << actual <<endl;
    pid->err=pid->SetSpeed-pid->ActualSpeed;
    if (pid->err > 400) pid->err = 400;
    else if(pid->err < -400) pid->err = -400;
    pid->integral+=pid->err;

    
    pid->voltage=pid->Kp*pid->err+pid->Ki*pid->integral+pid->Kd*(pid->err-pid->err_last);
    //cout << "voltage" << pid->voltage <<endl;
    pid->err_last=pid->err;

    if (pid->voltage > pid->umax) pid->voltage = pid->umax;
    else if(pid->voltage < pid->umin) pid->voltage = pid->umin;
    //cout << "ActualSpeed" << pid->ActualSpeed <<endl;

    if(pid->voltage > 360)
    {
        pid->voltage = pid->voltage - 360;
    }
    else if(pid->voltage < 0)
    {
        pid->voltage = pid->voltage + 360;
    }

    return pid->voltage;

}

void generate_com(int Move_mode, float state_desired[4])
{
    //# Move_mode 2-bit value:
    //# 0 for position, 1 for vel, 1st for xy, 2nd for z.
    //#                   xy position     xy velocity
    //# z position       	0b00(0)       0b10(2)
    //# z velocity		0b01(1)       0b11(3)

    if(Move_mode == prometheus_msgs::PositionReference::XYZ_ACC)
    {
        cout << "ACC control not support yet." <<endl;
    }

    if((Move_mode & 0b10) == 0) //xy channel
    {
        Command_to_pub.Reference_State.position_ref[0] = state_desired[0];
        Command_to_pub.Reference_State.position_ref[1] = state_desired[1];
        Command_to_pub.Reference_State.velocity_ref[0] = 0;
        Command_to_pub.Reference_State.velocity_ref[1] = 0;
    }
    else
    {
        Command_to_pub.Reference_State.position_ref[0] = 0;
        Command_to_pub.Reference_State.position_ref[1] = 0;
        Command_to_pub.Reference_State.velocity_ref[0] = state_desired[0];
        Command_to_pub.Reference_State.velocity_ref[1] = state_desired[1];
    }

    if((Move_mode & 0b01) == 0) //z channel
    {
        Command_to_pub.Reference_State.position_ref[2] = state_desired[2];
        Command_to_pub.Reference_State.velocity_ref[2] = 0;
    }
    else
    {
        Command_to_pub.Reference_State.position_ref[2] = 0;
        Command_to_pub.Reference_State.velocity_ref[2] = state_desired[2];
    }

    Command_to_pub.Reference_State.acceleration_ref[0] = 0;
    Command_to_pub.Reference_State.acceleration_ref[1] = 0;
    Command_to_pub.Reference_State.acceleration_ref[2] = 0;


    Command_to_pub.Reference_State.yaw_ref = state_desired[3]/180.0*M_PI;
}
