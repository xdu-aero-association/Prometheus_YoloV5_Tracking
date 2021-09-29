//无人机追踪目标（此处先直接用目标位置进行追踪）
//无人机吊舱根据视觉算法的反馈调节角度，保证目标在相机中心位置
//ROS 头文件
#include <ros/ros.h>
#include <iostream>
#include "mission_utils.h"
#include "gimbal_control.h"
#include "message_utils.h"

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


//---------------------------------------Drone---------------------------------------------
bool hold_mode;
bool ignore_vision;
prometheus_msgs::DroneState _DroneState;    // 无人机状态
Eigen::Vector3d gimbal_att_sp;
Eigen::Vector3d gimbal_att;
Eigen::Vector3d gimbal_att_deg;
Eigen::Vector3d gimbal_att_rate;
Eigen::Vector3d gimbal_att_rate_deg;

float gimbal_rate;
Eigen::Vector3d mav_pos_;

int num_regain = 0;
int num_lost = 0;
bool is_detected = false;
Eigen::Vector3d det_pos_enu;
prometheus_msgs::MultiDetectionInfo aruco_info;

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>　函数声明　<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
void printf_result();
void PID_init(_pid pid,float max, float min);
float PID_realize(_pid pid, float speed, float actual);

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>回 调 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
void aruco_det_cb(const prometheus_msgs::MultiDetectionInfo::ConstPtr& msg)
{
    aruco_info = *msg;

    // 暂不考虑无人机姿态的影响
    det_pos_enu[0] = aruco_info.detection_infos[0].pixel_position[0] - 320;
    det_pos_enu[1] = aruco_info.detection_infos[0].pixel_position[1] - 280;
    cout << "det_pos_enu[0]:"<< det_pos_enu[0] <<endl;
    cout << "det_pos_enu[1]:"<< det_pos_enu[1] <<endl; 
    cout << "aruco_info.num_objs:"<< aruco_info.num_objs <<endl; 
     
}

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>主函数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
int main(int argc, char **argv)
{
    ros::init(argc, argv, "gimbal_control");
    ros::NodeHandle nh("~");
    PID_init(pid1,80/180*3.14,-80/180*3.14);
    PID_init(pid2,80/180*3.14,-80/180*3.14);

    // 节点运行频率： 20hz 
    ros::Rate rate(20.0);



    //【订阅】
    ros::Subscriber aruco_sub = nh.subscribe<prometheus_msgs::MultiDetectionInfo>("/prometheus/object_detection/yolov5_tensorrt_det", 10, aruco_det_cb);
;

    gimbal_control gimbal_control_;


    while(ros::ok())
    {
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

    cout << "det_pos_enu[0]:"<< det_pos_enu[0] <<endl;
    cout << "det_pos_enu[1]:"<< det_pos_enu[1] <<endl;
    
    // cout << "Target_pos (camera): " <<  pos_camera_frame[0] << " [m] "<<  pos_camera_frame[1] << " [m] "<<  pos_camera_frame[2] << " [m] "<<endl;
    // cout << "Target_pos (body): " <<  pos_body_frame[0] << " [m] "<<  pos_body_frame[1] << " [m] "<<  pos_body_frame[2] << " [m] "<<endl;
    // cout << "Target_pos (body_enu): " <<  pos_body_enu_frame[0] << " [m] "<<  pos_body_enu_frame[1] << " [m] "<<  pos_body_enu_frame[2] << " [m] "<<endl;
    // cout << "Detection_ENU(pos): " <<  pos_enu_frame[0] << " [m] "<<  pos_enu_frame[1] << " [m] "<<  pos_enu_frame[2] << " [m] "<<endl;
    
}

void PID_init(_pid pid,float max, float min)
{
    cout<<"PID_init begin \n";
    pid.SetSpeed=0.0;
    pid.ActualSpeed=0.0;
    pid.err=0.0;
    pid.err_last=0.0;
    pid.voltage=0.0;
    pid.integral=0.0;
    pid.Kp=0.1;
    pid.Ki=0.05;
    pid.Kd=0.1;
    pid.umax= max;
    pid.umin= min;
    pid.index_state = 0;
    cout<<"PID_init end \n";
}

float PID_realize(_pid pid, float target, float actual){
    pid.SetSpeed=target;
    pid.ActualSpeed = actual;
    pid.err=pid.SetSpeed-pid.ActualSpeed;
    if(pid.ActualSpeed>pid.umax)
    {
        //灰色底色表示抗积分饱和的实现
        //蓝色标注为积分分离过程
        if(abs(pid.err)>target)
        {
            pid.index_state=0;
        }
        else
        {
            pid.index_state=1;
            if(pid.err<0)
            {
                pid.integral+=pid.err;
            }
        }
    }
    else if(pid.ActualSpeed<pid.umin)
    {
        //积分分离过程
        if(abs(pid.err)>target)
        {
            pid.index_state=0;
        }
        else
        {
            pid.index_state=1;
            if(pid.err>0)
            {
                pid.integral+=pid.err;
            }
        }
    }
    else
    {
        //积分分离过程
        if(abs(pid.err)>target)
        {
            pid.index_state=0;
        }
        else
        {
            pid.index_state=1;
            pid.integral+=pid.err;
        }
    }
    pid.voltage=pid.Kp*pid.err+pid.index_state*pid.Ki*pid.integral+pid.Kd*(pid.err-pid.err_last);
    pid.err_last=pid.err;
    pid.ActualSpeed=pid.voltage*1.0;

    if (pid.ActualSpeed > pid.umax) pid.ActualSpeed = pid.umax;
    else if(pid.ActualSpeed < pid.umin) pid.ActualSpeed = pid.umin;
    return pid.ActualSpeed;
}
