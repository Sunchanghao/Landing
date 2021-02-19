/***************************************************************************************************************************
*
* Author: bingo
* Email: bingobin.lw@gmail.com
* Time: 2020.02.19
* Description: 实现小车在offboard模式下的走预定航点,其中的位置控制采用的是飞控中的NED坐标系，可参考这个博客：https://blog.csdn.net/qq_33641919/article/details/101003978
***************************************************************************************************************************/
#include "mission_car_offboard.h"
using namespace std;
using namespace Eigen;
MissionCar::MissionCar(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private):
  nh_(nh),
  nh_private_(nh_private) {
  Initialize();
  cmdloop_timer_ = nh_.createTimer(ros::Duration(0.1), &MissionCar::CmdLoopCallback, this); // Define timer for constant loop rate 0.1s

  position_sub_ = nh_private_.subscribe("/mavros/local_position/pose", 1, &MissionCar::Px4PosCallback,this,ros::TransportHints().tcpNoDelay());

}

MissionCar::~MissionCar() {
  //Destructor
}
void MissionCar::CmdLoopCallback(const ros::TimerEvent& event)
{
	MissionStateUpdate();
}

/**
* @name      bool MissionCar::CarPosControl(Eigen::Vector3d &currPose,float currYaw,Eigen::Vector3d &expectPose)

* @brief      实现px4小车offboard模式下的位置控制
*             
* @param[in]  &currPose currYaw 当前小车的位置以及偏航角（NED坐标系）
*             
* @param[in]  &expectPose 期望位置（NED坐标系）
* @param[out] 是否到达目标点
*
* @param[out] 
**/
bool MissionCar::CarPosControl(Eigen::Vector3d &currPose,float currYaw,Eigen::Vector3d &expectPose)
{
    float expectYaw;
    float tanX,tanY;
    Eigen::Vector3d expectAtt;
    tanX = expectPose[0] - currPose[0];
    tanY = expectPose[1] - currPose[1];
    //Compute arc tangent with two parameters
    //return Principal arc tangent of y/x, in the interval [-pi,+pi] radians.
    //One radian is equivalent to 180/PI degrees.
    expectYaw = atan2(tanY,tanX)*(180/pi);
    expectAtt[0] = 0;
    expectAtt[1] = 0;
    expectAtt[2] = expectYaw;
    if(expectAtt[2] < 0)
    {
    	expectAtt[2] = 360+expectAtt[2];
    }
   // cout << "expectYaw = " << expectAtt[2] << endl;
    currYaw = currYaw * (180/pi);

		OffboardControl_.send_attitude_setpoint(expectAtt,desire_vel_);
   //据航点小于0.5m即作为到达目的地
	if(sqrt((currPose[0]-expectPose[0])*(currPose[0]-expectPose[0]) - (currPose[1]-expectPose[1])*(currPose[1]-expectPose[1])) <= 0.5)
	{
		return true;
	}
	else
	{
		return false;
	}
}


void MissionCar::MissionStateUpdate()
{
    Eigen::Vector3d desirePose;
	if(mission_finish_ == false)
	{

		if(CarPosControl(car_pose_,curr_yaw_,desire_pose_))
		{
			mission_step_ ++;
			if(mission_step_ == 2)
			{
				desire_pose_[0] = step2_Pose_x;
				desire_pose_[1] = step2_Pose_y;	
			}
			else if(mission_step_ == 3)
			{
				desire_pose_[0] = step3_Pose_x;
				desire_pose_[1] = step3_Pose_y;
			}
			else if(mission_step_ == 4)
			{
				desire_pose_[0] = step4_Pose_x;
				desire_pose_[1] = step4_Pose_y;
			}
			 if(mission_step_ == 5)
			{
				mission_finish_ = true;
				cout << "mission finish" << endl;
			}
			else
			{
	      cout << "current step: " << mission_step_ << endl;
				cout << "current desire poseX" << desire_pose_[0] << endl;
				cout << "current desire poseY" << desire_pose_[1] << endl;
			}
		}
	}
	else
	{
		Eigen::Vector3d stopAtt;
		stopAtt[0] = 0;
		stopAtt[1] = 0;
		stopAtt[2] = curr_yaw_*(180/pi);
		OffboardControl_.send_attitude_setpoint(stopAtt,0);
	}
	
//	cout << "missionStep = " << mission_step_ << endl;
}
//接收来自飞控的当前car位置和偏航角，并转换成飞控中的NED坐标系                
void MissionCar::Px4PosCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    double temp_roll,temp_pitch,temp_yaw;
    tf::Quaternion quat;
    tf::quaternionMsgToTF(msg->pose.orientation,quat);
    tf::Matrix3x3(quat).getRPY(temp_roll,temp_pitch,temp_yaw);
    curr_yaw_ = -temp_yaw + pi/2;
    //保证curr_yaw_范围为[-pi,+pi]
    if(curr_yaw_ > pi)
	{
		curr_yaw_ = curr_yaw_ - 2*pi;
	}
//    cout << "curr_yaw = " << curr_yaw_ << endl;
    Eigen::Vector3d pos_drone_fcu_enu(msg->pose.position.x,msg->pose.position.y,msg->pose.position.z);
    car_pose_[0] = pos_drone_fcu_enu[1];
    car_pose_[1] = pos_drone_fcu_enu[0];
    car_pose_[2] = -pos_drone_fcu_enu[2];
    //cout << "car_pose_x = " << car_pose_[0] << endl;
    //cout << "car_pose_y = " << car_pose_[1] << endl;
   
}
void MissionCar::Initialize()
{
	mission_step_ = 1;
	mission_finish_ = false;
  nh_private_.param<float>("step1_Pose_x", step1_Pose_x, 0);
  nh_private_.param<float>("step1_Pose_y", step1_Pose_y, 0);
  nh_private_.param<float>("step2_Pose_x", step2_Pose_x, 0);
  nh_private_.param<float>("step2_Pose_y", step2_Pose_y, 0);
  nh_private_.param<float>("step3_Pose_x", step3_Pose_x, 0);
  nh_private_.param<float>("step3_Pose_y", step3_Pose_y, 0);
  nh_private_.param<float>("step4_Pose_x", step4_Pose_x, 0);
  nh_private_.param<float>("step4_Pose_y", step4_Pose_y, 0);
  nh_private_.param<float>("desire_vel_",  desire_vel_, 0);
	desire_pose_[0] = step1_Pose_x;
	desire_pose_[1] = step1_Pose_y;

	cout << "current desire vel: " << desire_vel_ << endl;
	cout << "current step: " << mission_step_ << endl;
	cout << "current desire poseX: " << step1_Pose_x << endl;
	cout << "current desire poseY: " << step1_Pose_y << endl;
}
int main(int argc, char** argv) {
  ros::init(argc,argv,"mission_car_offboard");
  ros::NodeHandle nh("");
  ros::NodeHandle nh_private("~");

  MissionCar MissionCar(nh, nh_private);

  ros::spin();
  return 0;
}
