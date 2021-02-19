/***************************************************************************************************************************
*
* Author: bingo
* Email: bingobin.lw@gmail.com
* Time: 2020.05.26
* Description: 实现无人机在未知环境下自主导航，探索目标 
***************************************************************************************************************************/
#include "auto_2d_nav.h"
using namespace std;
using namespace Eigen;
Auto2dNav::Auto2dNav(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private):
  nh_(nh),
  nh_private_(nh_private) 
{
  initialize();
  cmdloop_timer_ = nh_.createTimer(ros::Duration(0.1), &Auto2dNav::CmdLoopCallback, this); //定义运行周期为0.1s


}

Auto2dNav::~Auto2dNav() 
{
  //Destructor
}
void Auto2dNav::CmdLoopCallback(const ros::TimerEvent& event)
{
}


void Auto2dNav::initialize()
{
    MoveBaseClient ac("move_base");

  //give some time for connections to register
  sleep(2.0);
  ROS_INFO("sleep");
  move_base_msgs::MoveBaseGoal goal;

  //we'll send a goal to the robot to move 2 meters forward
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();

  goal.target_pose.pose.position.x = 2.0;
  goal.target_pose.pose.position.y = 0.2;
  goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(M_PI);

  ROS_INFO("Sending goal");
  ac.sendGoal(goal);
  ROS_INFO("wait goal");
  ac.waitForResult();
  ROS_INFO("getState goal");
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("Hooray, the base moved 2 meters forward");
  else
    ROS_INFO("The base failed to move forward 2 meters for some reason");


}
int main(int argc, char** argv)
{
  ros::init(argc,argv,"auto_2d_nav");
  ros::NodeHandle nh("");
  ros::NodeHandle nh_private("~");

  Auto2dNav Auto2dNav(nh, nh_private);

  ros::spin();
  return 0;
}
