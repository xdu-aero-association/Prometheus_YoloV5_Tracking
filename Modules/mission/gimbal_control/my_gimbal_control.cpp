//无人机追踪目标（此处先直接用目标位置进行追踪）
//无人机吊舱根据视觉算法的反馈调节角度，保证目标在相机中心位置
//ROS 头文件
#include <ros/ros.h>
#include <iostream>
#include "mission_utils.h"
#include "gimbal_control.h"
#include "message_utils.h"
#include <mavros_msgs/Thrust.h>

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

int cal=0;
//---------------------------------------Drone---------------------------------------------
bool hold_mode;
bool ignore_vision;
prometheus_msgs::DroneState _DroneState;    // 无人机状态
mavros_msgs::Thrust Switch;

Eigen::Vector3d gimbal_att_sp;
Eigen::Vector3d gimbal_att;
Eigen::Vector3d gimbal_att_deg;
Eigen::Vector3d gimbal_att_rate;
Eigen::Vector3d gimbal_att_rate_deg;

float gimbal_rate;
Eigen::Vector3d mav_pos_;

int setout = 1;
int num_regain = 0;
int num_lost = 0;
bool is_detected = false;
Eigen::Vector3d det_pos_enu;
prometheus_msgs::MultiDetectionInfo aruco_info;

#define ON "1"           //开启
#define OFF "0"          //关闭
#define DIRECTION "out"  //方向为输出
#define LED "448"     //控制LED引脚

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>　函数声明　<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
void printf_result();
void PID_init(_pid *pid,float max, float min);
float PID_realize(_pid *pid, float speed, float actual);
int setGpioDirection(const char *port,const char *direction);
int setGpioValue(const char *port,const char *level);

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>回 调 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
void det_cb(const prometheus_msgs::MultiDetectionInfo::ConstPtr& msg)
{

    int fd;
    cal = 1;
    if(setout)
    {      
        cout << "the setout is 1" <<endl;
        if(setGpioDirection(LED,DIRECTION) == -1)
        {
            cout << "open the gpio false"<<endl;
        }
        else
        {   
            cout << "open the gpio"<<endl;
            setout = 0;
        }       
    }
    aruco_info = *msg;
    if(aruco_info.num_objs == 1)
    {
        // 暂不考虑无人机姿态的影响
        det_pos_enu[0] = aruco_info.detection_infos[0].pixel_position[0] - 320;
        det_pos_enu[1] = aruco_info.detection_infos[0].pixel_position[1] - 240;
        
        //cout << "det_pos_enu[0]:"<< det_pos_enu[0] <<endl;
        //cout << "det_pos_enu[1]:"<< det_pos_enu[1] <<endl;

        if(setout == 0 )
        {
            //cout << "the distance:"<< det_pos_enu[0]*det_pos_enu[0]+det_pos_enu[1]*det_pos_enu[1] <<endl;
            //if((det_pos_enu[0]*det_pos_enu[0]+det_pos_enu[1]*det_pos_enu[1]) < 250)
            if(true)
            {
                setGpioValue(LED,ON);
            }
            else
            {
                setGpioValue(LED,OFF);
            }
            
        }
        if(aruco_info.detection_infos[0].detected)
        {
            num_regain++;
            num_lost = 0;
        }else
        {
            num_regain = 0;
            num_lost++;
        }

        // 当连续一段时间无法检测到目标时，认定目标丢失
        if(num_lost > VISION_THRES_TRACKING)
        {
            is_detected = false;

            //　丢失后　对sight_angle清零，否则云台会移动
            det_pos_enu[0] = 0.0;
            det_pos_enu[1] = 0.0;
        }

        // 当连续一段时间检测到目标时，认定目标得到
        if(num_regain > 2)
        {
            is_detected = true;
        }
    }
    else
    {
        // 对sight_angle清零，否则云台会移动
        det_pos_enu[0] = 0.0;
        det_pos_enu[1] = 0.0;
        // 云台俯仰角
        gimbal_att_sp[1] =0;
        // 云台偏航角 取决于左右夹角
        gimbal_att_sp[2] =0;
    }
}
void drone_state_cb(const prometheus_msgs::DroneState::ConstPtr& msg)
{
    _DroneState = *msg;

    mav_pos_ << _DroneState.position[0],_DroneState.position[1],_DroneState.position[2];

}
void gimbal_control_cb(const ros::TimerEvent& e)
{
    // 使用目标位置的视场角控制云台
    // 品灵吊舱原做法：根据像素来控制云台
    // 传统做法(Jin Ren)：使用视场角误差来反馈，控制吊舱角速度(PD control)
    // 但我们这里只能直接控制云台角度（不能直接用于品灵吊舱的控制），注意sight_angle是误差值
    //　遇到的问题１：由于传进来的sight_angle有噪音，导致云台有一点点抽搐（可以考虑做一个平滑滤波，或者降低控制频率）
    // 云台滚转角不控制
    gimbal_att_sp[0] = 0.0;
    // 云台俯仰角
    gimbal_att_sp[1] = -PID_realize(&pid1, 0, det_pos_enu[1]);
    // 云台偏航角 取决于左右夹角
    gimbal_att_sp[2] = -PID_realize(&pid2, 0, det_pos_enu[0]);
    //cout << "gimbal_att_sp    : " << gimbal_att_sp <<endl;

}
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>主函数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
int main(int argc, char **argv)
{
    ros::init(argc, argv, "gimbal_control");
    ros::NodeHandle nh("~");
    PID_init(&pid1,80,-60);
    PID_init(&pid2,60,-60);

    // 节点运行频率： 20hz 
    ros::Rate rate(20.0);

    bool moving_target;
    nh.param<bool>("hold_mode", hold_mode, false);
    nh.param<bool>("moving_target", moving_target, false);
    nh.param<float>("gimbal_rate", gimbal_rate, 0.1);

    ros::Publisher Switch_pub = nh.advertise<mavros_msgs::Thrust>("/prometheus/switch", 1);
    //【订阅】
    ros::Subscriber aruco_sub = nh.subscribe<prometheus_msgs::MultiDetectionInfo>("/prometheus/object_detection/yolov5_tensorrt_det", 10, det_cb);
    //【订阅】无人机状态
    ros::Subscriber drone_state_sub = nh.subscribe<prometheus_msgs::DroneState>("/prometheus/drone_state", 20, drone_state_cb);
    //　吊舱控制timer
    ros::Timer timer = nh.createTimer(ros::Duration(gimbal_rate), gimbal_control_cb);

    gimbal_control gimbal_control_;
    /*
    while(ros::ok() && _DroneState.mode != "OFFBOARD")
    { 
        cout << "Switch to OFFBOARD and arm ..."<<endl;
        ros::Duration(2.0).sleep();
        ros::spinOnce();
    }
    */

    while(ros::ok())
    {
        cal += 1;

        //　云台数据打印
        gimbal_att = gimbal_control_.get_gimbal_att();
        gimbal_att_deg = gimbal_att/PI*180;
        cout << "gimbal_att_deg    : " << gimbal_att_deg[0] << " [deg] "<< gimbal_att_deg[1] << " [deg] "<< gimbal_att_deg[2] << " [deg] "<<endl;
        gimbal_att_rate = gimbal_control_.get_gimbal_att_rate();
        gimbal_att_rate_deg = gimbal_att_rate/PI*180;
        Switch.thrust = 0;
         //cout << "send switch" << endl;
        if (cal >= 15)
        {
            //cout << "send switch" << endl;
            Switch.thrust = 1;
            pid1.integral=0.0;
            pid2.integral=0.0;
            gimbal_att_sp[0] = 0;
            gimbal_att_sp[1] = 10;
            gimbal_att_sp[2] = 0;
        }

        Switch_pub.publish(Switch);
        //cout << "gimbal_att_rate    : " << gimbal_att_rate_deg[0] << " [deg/s] "<< gimbal_att_rate_deg[1] << " [deg/s] "<< gimbal_att_rate_deg[2] << " [deg/s] "<<endl;
        gimbal_control_.send_mount_control_command(gimbal_att_sp);
        //printf_result();
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
    
    // cout << "Target_pos (camera): " <<  pos_camera_frame[0] << " [m] "<<  pos_camera_frame[1] << " [m] "<<  pos_camera_frame[2] << " [m] "<<endl;
    // cout << "Target_pos (body): " <<  pos_body_frame[0] << " [m] "<<  pos_body_frame[1] << " [m] "<<  pos_body_frame[2] << " [m] "<<endl;
    // cout << "Target_pos (body_enu): " <<  pos_body_enu_frame[0] << " [m] "<<  pos_body_enu_frame[1] << " [m] "<<  pos_body_enu_frame[2] << " [m] "<<endl;
    // cout << "Detection_ENU(pos): " <<  pos_enu_frame[0] << " [m] "<<  pos_enu_frame[1] << " [m] "<<  pos_enu_frame[2] << " [m] "<<endl;
    
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
    pid->Kp=0.017;
    pid->Ki=0.015;
    pid->Kd=0.002;
    pid->umax= max;
    pid->umin= min;
    pid->index_state = 0;
    cout<<"PID_init end \n";
}

float PID_realize(_pid *pid, float target, float actual){
    /*
    cout << "Kp" <<  pid->Kp <<endl;
    cout << "Ki" <<  pid->Ki <<endl;
    cout << "Kd" <<  pid->Kd <<endl;
    */

    pid->SetSpeed=target;
    //cout << "target" << target <<endl;
    pid->ActualSpeed = actual;
    //cout << "actual" << actual <<endl;
    pid->err=pid->SetSpeed-pid->ActualSpeed;
    if (pid->err > 300) pid->err = 300;
    else if(pid->err < -300) pid->err = -300;
    pid->integral+=pid->err;

    
    pid->voltage=pid->Kp*pid->err+pid->Ki*pid->integral+pid->Kd*(pid->err-pid->err_last);
    //cout << "voltage" << pid->voltage <<endl;
    pid->err_last=pid->err;
    pid->ActualSpeed=pid->voltage*1.0;

    if (pid->ActualSpeed > pid->umax) pid->ActualSpeed = pid->umax;
    else if(pid->ActualSpeed < pid->umin) pid->ActualSpeed = pid->umin;
    //cout << "ActualSpeed" << pid->ActualSpeed <<endl;

    return pid->ActualSpeed;

}

int setGpioValue(const char *port,const char *level)
{
	int fd;
	char path[40];
	sprintf(path, "/sys/class/gpio/gpio%s/value", port);
	fd = open(path, O_RDWR);
	if(fd == -1)
	{
	   perror("Failed to set GPIO value! ");
	   return -1;
	}       
	write(fd, level, sizeof(level));
	close(fd);
	return 0;
}

int setGpioDirection(const char *port,const char *direction)
{
	int fd;
	char path[40];
	sprintf(path, "/sys/class/gpio/gpio%s/direction", port);
	fd = open(path, O_WRONLY);

	if(fd == -1)
	{
	   perror("Failed to set GPIO direction. ");

	   return -1;
	}

	write(fd, direction, sizeof(direction)); 
	close(fd); 
	return 0;
}
