/***************************************************************************************************************************
*
* Author: bingo
* Email: bingobin.lw@gmail.com
* Time: 2020.01.31
* Description: 实现ros navigation中move_base速度控制输出的cmd_vel控制px4 quadrotor 
***************************************************************************************************************************/
#include "ros_nav_quadrotor.h"
using namespace std;
using namespace Eigen;
PX4RosNav::PX4RosNav(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private):
  nh_(nh),
  nh_private_(nh_private) 
{
  initialize();
  cmdloop_timer_ = nh_.createTimer(ros::Duration(0.1), &PX4RosNav::CmdLoopCallback, this); //定义运行周期为0.1s
cmd_vel_sub_ = nh_private_.subscribe("/px4_vel", 1, &PX4RosNav::CmdVelCallback, this,ros::TransportHints().tcpNoDelay());

}

PX4RosNav::~PX4RosNav() 
{
  //Destructor
}
void PX4RosNav::CmdLoopCallback(const ros::TimerEvent& event)
{
	PublishVelControl();
}

void PX4RosNav::PublishVelControl()
{

  OffboardControl_.send_velxy_posz_setpoint(px4_vel_,desire_posz_);
//  cout << "px4_vel[0]"<<px4_vel_[0] <<endl;
//  cout << "px4_vel[1]"<<px4_vel_[1] <<endl;
}

void PX4RosNav::CmdVelCallback(const geometry_msgs::Twist &msg)
{
 px4_vel_[0] = -msg.linear.y;
 px4_vel_[1] = msg.linear.x;
 
}
void PX4RosNav::initialize()
{
  px4_vel_[0] = 0;
  px4_vel_[1] = 0;
  //读取offboard模式下飞机的期望高度
  nh_.param<float>("desire_posz_", desire_posz_, 1.0);
}
int main(int argc, char** argv)
{
  ros::init(argc,argv,"ros_nav_quadrotor");
  ros::NodeHandle nh("");
  ros::NodeHandle nh_private("~");

  PX4RosNav PX4RosNav(nh, nh_private);

  ros::spin();
  return 0;
}
