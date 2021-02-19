/***************************************************************************************************************************
*
* Author: bingo
* Email: bingobin.lw@gmail.com
* Time: 2020.02.08
* Description: 实现px4 quadrotor 二维码精准着陆，
***************************************************************************************************************************/
#include "landing_quadrotor.h"
using namespace std;
using namespace Eigen;
PX4Landing::PX4Landing(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private):
  nh_(nh),
  nh_private_(nh_private) {
  Initialize();
  cmdloop_timer_ = nh_.createTimer(ros::Duration(0.1), &PX4Landing::CmdLoopCallback, this); //周期为0.1s
  //订阅降落板相对飞机位置
  ar_pose_sub_ = nh_private_.subscribe("/ar_pose_marker", 1, &PX4Landing::ArPoseCallback, this,ros::TransportHints().tcpNoDelay());

  position_sub_ = nh_private_.subscribe("/mavros/local_position/pose", 1, &PX4Landing::Px4PosCallback,this,ros::TransportHints().tcpNoDelay());

  state_sub_ = nh_private_.subscribe("/mavros/state", 1, &PX4Landing::Px4StateCallback,this,ros::TransportHints().tcpNoDelay());
  // 【服务】修改系统模式
  set_mode_client_ = nh_private_.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");

}

PX4Landing::~PX4Landing() {
  //Destructor
}

/**
* @name       S_SETPOINT_VEL PX4Landing::LandingPidProcess(Eigen::Vector3d &currentPos,Eigen::Vector3d &expectPos)

* @brief      pid控制程序
*             
* @param[in]  &currentPos 当前飞机相对降落板的位置,currentYaw 当前飞机相对降落板的方向
*             
* @param[in]  &expectPos 期望位置，expectYaw 飞机相对降落板的期望方向:默认0
* @param[out] 机体系下x,y,z的期望速度,以及yaw方向的期望速度。
*
* @param[out] 
**/
Eigen::Vector4d PX4Landing::LandingPidProcess(Eigen::Vector3d &currentPos,float currentYaw,Eigen::Vector3d &expectPos,float expectYaw)
{
  Eigen::Vector4d s_PidOut;

	/*X方向的pid控制*/
	s_PidItemX.difference = expectPos[0] - currentPos[0];
	s_PidItemX.intergral += s_PidItemX.difference;
	if(s_PidItemX.intergral >= 100)		
		s_PidItemX.intergral = 100;
	else if(s_PidItemX.intergral <= -100) 
		s_PidItemX.intergral = -100;
	s_PidItemX.differential =  s_PidItemX.difference  - s_PidItemX.tempDiffer;
  s_PidItemX.tempDiffer = s_PidItemX.difference;
//	cout << "s_PidItemX.tempDiffer: " << s_PidItemX.tempDiffer << endl;
//	cout << "s_PidItemX.differential: " << s_PidItemX.differential << endl;

	s_PidOut[0] = s_PidXY.p*s_PidItemX.difference + s_PidXY.d*s_PidItemX.differential + s_PidXY.i*s_PidItemX.intergral;
	/*Y方向的pid控制*/
	s_PidItemY.difference = expectPos[1] - currentPos[1];
	s_PidItemY.intergral += s_PidItemY.difference;
	if(s_PidItemY.intergral >= 100)		
		s_PidItemY.intergral = 100;
	else if(s_PidItemY.intergral <= -100) 
		s_PidItemY.intergral = -100;
	s_PidItemY.differential =  s_PidItemY.difference  - s_PidItemY.tempDiffer;
  s_PidItemY.tempDiffer = s_PidItemY.difference;
	s_PidOut[1] = s_PidXY.p*s_PidItemY.difference + s_PidXY.d*s_PidItemY.differential + s_PidXY.i*s_PidItemY.intergral;

	/*Z方向的pid控制*/
	s_PidItemZ.difference = expectPos[2] - currentPos[2];
	s_PidItemZ.intergral += s_PidItemZ.difference;
	if(s_PidItemZ.intergral >= 100)		
		s_PidItemZ.intergral = 100;
	else if(s_PidItemZ.intergral <= -100) 
		s_PidItemZ.intergral = -100;
	s_PidItemZ.differential =  s_PidItemZ.difference  - s_PidItemZ.tempDiffer;
  s_PidItemZ.tempDiffer = s_PidItemZ.difference;
	s_PidOut[2] = s_PidZ.p*s_PidItemZ.difference + s_PidZ.d*s_PidItemZ.differential + s_PidZ.i*s_PidItemZ.intergral;

	/*Yaw方向的pid控制*/
	s_PidItemYaw.difference =  expectYaw - currentYaw;
	s_PidItemYaw.intergral += s_PidItemYaw.difference;
	if(s_PidItemYaw.intergral >= 100)		
		s_PidItemYaw.intergral = 100;
	else if(s_PidItemYaw.intergral <= -100) 
		s_PidItemYaw.intergral = -100;
	s_PidItemYaw.differential =  s_PidItemYaw.difference  - s_PidItemYaw.tempDiffer;
  s_PidItemYaw.tempDiffer = s_PidItemYaw.difference;
	s_PidOut[3] = s_PidYaw.p*s_PidItemYaw.difference + s_PidYaw.d*s_PidItemYaw.differential + s_PidYaw.i*s_PidItemYaw.intergral;

	return s_PidOut;
}
void PX4Landing::CmdLoopCallback(const ros::TimerEvent& event)
{
  LandingStateUpdate();
}


/**
* @name       void PX4Landing::LandingStateUpdate()
* @brief      状态机更新函数
*             
* @param[in]  无
*             
* @param[in]  无
* @param[out] 
*
* @param[out] 
**/
void PX4Landing::LandingStateUpdate()
{

//	desire_vel_ = LandingPidProcess(ar_pose_,markers_yaw_,desire_pose_,0);
//	cout << "desire_vel_[0]:  "<< desire_vel_[0] <<endl;
//	cout << "desire_vel_[1]:  "<< desire_vel_[1] <<endl;
//	cout << "desire_vel_[2]:  "<< desire_vel_[2] <<endl;
//	cout << "desire_vel_[3]:  "<< desire_vel_[3] <<endl;
//	cout << "markers_yaw_: "  << markers_yaw_ << endl;
//	cout << "ar_pose_[0]:  "<<  ar_pose_[0] << endl;
//	cout << "ar_pose_[1]:  "<<  ar_pose_[1] << endl;
//	cout << "ar_pose_[2]:  "<<  ar_pose_[2] << endl;
//	cout << "desire_pose_[0]:  "<<  desire_pose_[0] << endl;
//	cout << "desire_pose_[1]:  "<<  desire_pose_[1] << endl;
//	cout << "desire_pose_[2]:  "<<  desire_pose_[2] << endl;
//	cout << "detect_state : " << detect_state << endl;
	switch(LandingState)
	{
		case WAITING:
			if(px4_state_.mode == "AUTO.RTL" && detect_state == true)
			{
				mode_cmd_.request.custom_mode = "OFFBOARD";
				set_mode_client_.call(mode_cmd_);
			}
			if(px4_state_.mode != "OFFBOARD")//等待offboard模式
			{
				temp_pos_drone[0] = px4_pose_[0];
				temp_pos_drone[1] = px4_pose_[1];
				temp_pos_drone[2] = px4_pose_[2];
				OffboardControl_.send_pos_setpoint(temp_pos_drone, 0);
			}
			if(px4_state_.mode == "OFFBOARD")
			{
				temp_pos_drone[0] = px4_pose_[0];
				temp_pos_drone[1] = px4_pose_[1];
				temp_pos_drone[2] = px4_pose_[2];
				LandingState = CHECKING;
				cout << "CHECKING" <<endl;
			}
			break;
		case CHECKING:
			if(px4_pose_[0] == 0 && px4_pose_[1] == 0) 			//没有位置信息则执行降落模式
			{
				cout << "Check error, make sure have local location" <<endl;
				mode_cmd_.request.custom_mode = "AUTO.LAND";
				set_mode_client_.call(mode_cmd_);
				LandingState = WAITING;	
			}
			else
			{
				LandingState = PREPARE;
				cout << "PREPARE" <<endl;
			}
			
			break;
		case PREPARE:											//起飞到指定高度
			posxyz_target[0] = temp_pos_drone[0];
			posxyz_target[1] = temp_pos_drone[1];
			posxyz_target[2] = search_alt_;
			if((px4_pose_[2]<=search_alt_+0.1) && (px4_pose_[2]>=search_alt_-0.1))
			{
				LandingState = SEARCH;
			}
			else if(detect_state == true)
			{
			//	LandingState = SEARCH;
			}
			OffboardControl_.send_pos_setpoint(posxyz_target, 0);					
			if(px4_state_.mode != "OFFBOARD")				//如果在准备中途中切换到onboard，则跳到WAITING
			{
				LandingState = WAITING;
			}

			break;
		case SEARCH:
			if(detect_state == true)
			{
				LandingState = LANDING;
			  cout << "LANDING" <<endl;
			}	
			else//这里无人机没有主动搜寻目标
			{
				OffboardControl_.send_pos_setpoint(posxyz_target, 0);
			}
			if(px4_state_.mode != "OFFBOARD")				//如果在SEARCH途中切换到onboard，则跳到WAITING
			{
				LandingState = WAITING;
			}
     // cout << "SEARCH" <<endl;
			break;
		case LANDING:
			{
				if(detect_state == true)
				{
					desire_vel_ = LandingPidProcess(ar_pose_,markers_yaw_,desire_pose_,desire_yaw_);

					//cout << "search_" <<endl;
				}
			  else
				{
					desire_vel_[0] = 0;
					desire_vel_[1] = 0;
					desire_vel_[2] = 0;
					desire_vel_[3] = 0;
				}
				if(ar_pose_[2] <= 0.3)
				{
					LandingState = LANDOVER;
					cout << "LANDOVER" <<endl;
				}
				if(px4_state_.mode != "OFFBOARD")			//如果在LANDING中途中切换到onboard，则跳到WAITING
				{
					LandingState = WAITING;
				}
				desire_xyVel_[0] = desire_vel_[0];
				desire_xyVel_[1] = desire_vel_[1];
				desire_xyVel_[2] = desire_vel_[2];
				desire_yawVel_ = desire_vel_[3];
				OffboardControl_.send_body_velxyz_setpoint(desire_xyVel_,desire_yawVel_);
			}

			break;
		case LANDOVER:
			{
				mode_cmd_.request.custom_mode = "AUTO.LAND";
        		set_mode_client_.call(mode_cmd_);
				LandingState = WAITING;
			}

			break;

		default:
			cout << "error" <<endl;
	}	

}

/*接收降落板相对飞机的位置以及偏航角*/
void PX4Landing::ArPoseCallback(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr &msg)
{
	detect_state = false;
  double temp_roll,temp_pitch,temp_yaw;
  tf::Quaternion quat;
	for(auto &item : msg->markers)
	{
		if(item.id == markers_id_)
		{
			detect_state = true;
      ar_pose_[0] = -item.pose.pose.position.x;
      ar_pose_[1] = item.pose.pose.position.y;
      ar_pose_[2] = item.pose.pose.position.z;
      tf::quaternionMsgToTF(item.pose.pose.orientation,quat);
      tf::Matrix3x3(quat).getRPY(temp_roll,temp_pitch,temp_yaw);
			markers_yaw_ = temp_yaw;
//			cout << "ar_pose_[0]:"  << ar_pose_[0] << endl;
//			cout << "ar_pose_[1]:"  << ar_pose_[1] << endl;
//			cout << "ar_pose_[2]:"  << ar_pose_[2] << endl;
//			cout << "markers_yaw_: "  << markers_yaw_ << endl;
		}
	}
//	cout << "detect_state :" << detect_state << endl;
}

/*接收来自飞控的当前飞机位置*/                  
void PX4Landing::Px4PosCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    // Read the Drone Position from the Mavros Package [Frame: ENU]
    Eigen::Vector3d pos_drone_fcu_enu(msg->pose.position.x,msg->pose.position.y,msg->pose.position.z);

    px4_pose_ = pos_drone_fcu_enu;
}
/*接收来自飞控的当前飞机状态*/
void PX4Landing::Px4StateCallback(const mavros_msgs::State::ConstPtr& msg)
{
	px4_state_ = *msg;
}

/*初始化*/
void PX4Landing::Initialize()
{
  //读取offboard模式下飞机的搜索高度
  nh_private_.param<float>("search_alt_", search_alt_, 3);


  nh_private_.param<float>("markers_id_", markers_id_, 4.0);

  nh_private_.param<float>("PidXY_p", s_PidXY.p, 0.4);
  nh_private_.param<float>("PidXY_d", s_PidXY.d, 0.05);
  nh_private_.param<float>("PidXY_i", s_PidXY.i, 0.01);
  nh_private_.param<float>("PidZ_p", s_PidZ.p, 0.1);
  nh_private_.param<float>("PidZ_d", s_PidZ.d, 0);
  nh_private_.param<float>("PidZ_i", s_PidZ.i, 0);
  nh_private_.param<float>("PidYaw_p", s_PidYaw.p, 0);
  nh_private_.param<float>("PidYaw_d", s_PidYaw.d, 0);
  nh_private_.param<float>("PidYaw_i", s_PidYaw.i, 0);

  //期望的飞机相对降落板的位置
	float desire_pose_x,desire_pose_y,desire_pose_z;
  nh_private_.param<float>("desire_pose_x", desire_pose_x, 0);
  nh_private_.param<float>("desire_pose_y", desire_pose_y, 0);
  nh_private_.param<float>("desire_pose_z", desire_pose_z, 0);
  nh_private_.param<float>("desire_yaw_", desire_yaw_, 0);
  desire_pose_[0] = desire_pose_x;
  desire_pose_[1] = desire_pose_y;
  desire_pose_[2] = desire_pose_z;

  detect_state = false;
  desire_vel_[0] = 0;
  desire_vel_[1] = 0;
  desire_vel_[2] = 0;
  desire_vel_[3] = 0;
	desire_xyVel_[0]  = 0;
	desire_xyVel_[1]  = 0;
	desire_xyVel_[2]  = 0;
	desire_yawVel_ = 0;
  s_PidItemX.tempDiffer = 0;
  s_PidItemY.tempDiffer = 0;
  s_PidItemZ.tempDiffer = 0;
  s_PidItemYaw.tempDiffer = 0;
  s_PidItemX.intergral = 0;
  s_PidItemY.intergral = 0;
  s_PidItemZ.intergral = 0;
  s_PidItemYaw.intergral = 0;

	cout << "search_alt_ = " << search_alt_ << endl;
	cout << "markers_id_ = " << markers_id_ << endl;
	cout << "PidXY_p = " << s_PidXY.p << endl;
	cout << "PidXY_d = " << s_PidXY.d << endl;
	cout << "PidXY_i = " << s_PidXY.i << endl;
	cout << "PidZ_p = " << s_PidZ.p << endl;
	cout << "PidZ_d = " << s_PidZ.d << endl;
	cout << "PidZ_i = " << s_PidZ.i << endl;
	cout << "PidYaw_p = " << s_PidYaw.p << endl;
	cout << "PidYaw_d = " << s_PidYaw.d << endl;
	cout << "PidYaw_i = " << s_PidYaw.i << endl;
	cout << "desire_pose_x = " << desire_pose_[0] << endl;
	cout << "desire_pose_y = " << desire_pose_[1] << endl;
	cout << "desire_pose_z = " << desire_pose_[2] << endl;
	cout << "desire_yaw_ = " << desire_yaw_ << endl;

}
int main(int argc, char** argv) {
  ros::init(argc,argv,"landing_quadrotor");
  ros::NodeHandle nh("");
  ros::NodeHandle nh_private("~");

  PX4Landing PX4Landing(nh, nh_private);

  ros::spin();
  return 0;
}
