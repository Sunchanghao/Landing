/***************************************************************************************************************************
*
* Author: bingo
* Email: bingobin.lw@gmail.com
* Time: 2020.03.10
* Description: 实现px4 quadrotor 二维码跟踪
***************************************************************************************************************************/
#include "tracking_quadrotor.h"

#include <iostream>
#include <math.h>
using namespace std;
using namespace Eigen;



PX4Tracking::PX4Tracking(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private):
  nh_(nh),
  nh_private_(nh_private) {
  Initialize();
  cmdloop_timer_ = nh_.createTimer(ros::Duration(0.1), &PX4Tracking::CmdLoopCallback, this); //周期为0.1s
  //订阅发布检测框位置的话题
  box_pose_sub_=nh_private_.subscribe("/base_point",1,&PX4Tracking::BoxPoseCallback, this,ros::TransportHints().tcpNoDelay());
  //订阅二维码相对飞机正前方位置
 //  ar_pose_sub_= nh_private_.subscribe("/ar_pose_marker", 1, &PX4Tracking::ArPoseCallback, this,ros::TransportHints().tcpNoDelay());
  //飞控得到的飞机的位置
  position_sub_ = nh_private_.subscribe("/mavros/local_position/pose", 1, &PX4Tracking::Px4PosCallback,this,ros::TransportHints().tcpNoDelay());
  //接收来自飞控的当前飞机的状态
  state_sub_ = nh_private_.subscribe("/mavros/state", 1, &PX4Tracking::Px4StateCallback,this,ros::TransportHints().tcpNoDelay());
  // 【服务】修改系统模式
  set_mode_client_ = nh_private_.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");

}

PX4Tracking::~PX4Tracking() {
  //Destructor
}

/**
* @name       S_SETPOINT_VEL PX4Tracking::TRACKINGPidProcess(Eigen::Vector3d &currentPos,Eigen::Vector3d &expectPos)

* @brief      pid控制程序,机体坐标系下控制无人机
*             
* @param[in]  &currentPos 当前飞机相对二维码的位置
*             
* @param[in]  &expectPos 期望位置 expectPos[0]:相对二维码前后方向距离；expectPos[1]:相对二维码左右方向距离；expectPos[2]:相对二维码上下方向距离
* @param[out] y,z的期望速度,以及yaw方向的期望速度，横向的roll
*
* @param[out] 
**/
Eigen::Vector4d PX4Tracking::TrackingPidProcess(Eigen::Vector3d &currentPos,Eigen::Vector3d &expectPos,Eigen::Vector4d &orientation)
{
 // Eigen::Vector3d s_PidOut;
	Eigen::Vector4d s_PidOut;
	Eigen::Vector3d xpid;
	Eigen::Vector3d ypid;
	Eigen::Vector3d zpid;


	/*前后方向的pid控制，输出机体坐标系下y方向的速度控制*/
	/* currentPos为飞机相对于二维码的坐标，expectPos为在飞机坐标系下飞机期望的坐标*/
	/*currentPos[2]为二维码的Z轴方向 expectPos[0]为飞机的X轴方向 */
	s_PidItemY.difference = currentPos[2] - expectPos[0] ;  //difference为比例控制项
	
	s_PidItemY.intergral += s_PidItemY.difference;    //intergral为积分控制项
	if(s_PidItemY.intergral >= 100)		
		s_PidItemY.intergral = 100;
	else if(s_PidItemY.intergral <= -100) 
		s_PidItemY.intergral = -100;
	s_PidItemY.differential =  s_PidItemY.difference  - s_PidItemY.tempDiffer;  //differential为微分控制项
    s_PidItemY.tempDiffer = s_PidItemY.difference;
//	cout << "s_PidItemY.tempDiffer: " << s_PidItemY.tempDiffer << endl;
//	cout << "s_PidItemY.differential: " << s_PidItemY.differential << endl;
/*分别用s_PidY的p i d乘上面得到的三项，然后做和 但s_PidY和s_PidItemY里面的结构不清楚  得到s_PidOut[0]，为在飞机坐标系下X方向也就是前后方向的速度*/
	s_PidOut[0] = 1*(s_PidY.p*s_PidItemY.difference  + s_PidY.d*s_PidItemY.differential + s_PidY.i*s_PidItemY.intergral);
	    if (s_PidOut[0]>1.5) 
	  s_PidOut[0] = 1.5;
	 if (s_PidOut[0]<-1.5)
	  s_PidOut[0]=-1.5;
  

 
	/*左右方向的pid控制，输出yaw方向速度控制*/
	/*currentPos[0]为二维码的X轴方向，expectPos[1]为飞机的Y轴方向*/
	/*此处用新的方法计算yaw偏航  加入了角速度*/
	desire_angleVel_=-0.3*currentPos[0]-expectPos[1];
	s_PidOut[1]=desire_angleVel_;


    //左右方向的pid控制，计算速度时暂时与s_PidY或s_PidZ系数相同
	s_PidItemRoll.difference = currentPos[0]-expectPos[1];	
	s_PidItemRoll.intergral += s_PidItemRoll.difference;	
	if(s_PidItemRoll.intergral >= 100)		
	    s_PidItemRoll.intergral = 100;
	else if(s_PidItemRoll.intergral <= -100) 
		s_PidItemRoll.intergral = -100;
		s_PidItemRoll.differential =  s_PidItemRoll.difference  - s_PidItemRoll.tempDiffer;
     s_PidItemRoll.tempDiffer = s_PidItemRoll.difference;
	
	//s_PidOut[3] = s_PidY.p*s_PidItemRoll.difference + s_PidY.d*s_PidItemRoll.differential + s_PidY.i*s_PidItemRoll.intergral;
     s_PidOut[3] = 0.3*(s_PidY.p*s_PidItemRoll.difference + s_PidY.d*s_PidItemRoll.differential + s_PidY.i*s_PidItemRoll.intergral);
     if (s_PidOut[3]>1.5) 
	  s_PidOut[3] = 1.5;
	 if (s_PidOut[3]<-1.5)
	  s_PidOut[3]=-1.5;
 

	/*上下方向的pid控制，输出z方向的速度控制*/
	/*currentPos[1]为二维码的Y轴方向 expectPos[2]为飞机的Z轴方向*/
	s_PidItemZ.difference = expectPos[2] - currentPos[1];
	cout<<"前后误差:"<<s_PidItemY.difference<<"  左右误差:"<<s_PidItemRoll.difference<<"  上下误差:"<<s_PidItemZ.difference<<endl;
	s_PidItemZ.intergral += s_PidItemZ.difference;
	if(s_PidItemZ.intergral >= 100)		
		s_PidItemZ.intergral = 100;
	else if(s_PidItemZ.intergral <= -100) 
		s_PidItemZ.intergral = -100;
	s_PidItemZ.differential =  s_PidItemZ.difference  - s_PidItemZ.tempDiffer;
    s_PidItemZ.tempDiffer = s_PidItemZ.difference;	
	//s_PidOut[2] = s_PidZ.p*s_PidItemZ.difference + s_PidZ.d*s_PidItemZ.differential + s_PidZ.i*s_PidItemZ.intergral;
	s_PidOut[2] =s_PidZ.p*s_PidItemZ.difference + s_PidZ.d*s_PidItemZ.differential + s_PidZ.i*s_PidItemZ.intergral;
	  cout<<"前进速度为:"<<s_PidOut[0]<<" 左右速度:"<<s_PidOut[3]<<"上下速度:"<<s_PidOut[2]<<endl;
	/*返回的是Eigen::Vector3d数据类型是一个三维的向量s_PidOut=(s_PidOut[0],s_PidOut[1],s_PidOut[2]) */
	return s_PidOut;   // 在TrackingStateUpdate()被实例化为desire_vel_ 
}

/**
 * @brief 由句柄nh_创建的cmdloop_timer_ 调用PX4Tracking::CmdLoopCallback（）函数 周期为0.1S
 * @brief 每隔0.1s调用一次TrackingStateUpdate()函数
 **/
void PX4Tracking::CmdLoopCallback(const ros::TimerEvent& event)
{
  TrackingStateUpdate();
}


/**
* @name       void PX4Tracking::TrackingStateUpdate()
* @brief      状态机更新函数
*             
* @param[in]  无
*             
* @param[in]  无
* @param[out] 
*
* @param[out] 
**/
void PX4Tracking::TrackingStateUpdate()
{
	switch(TrackingState)   //tracking_quadrotor.h中枚举的TrackingState
	{
		case WAITING:
			if(px4_state_.mode != "OFFBOARD")//等待offboard模式
			{
				temp_pos_drone[0] = px4_pose_[0];
				temp_pos_drone[1] = px4_pose_[1];
				temp_pos_drone[2] = px4_pose_[2];
				OffboardControl_.send_pos_setpoint(temp_pos_drone, 0);
			}
			else
			{
				/*如果已经是板外模式，设置追踪状态为CHECKING（检查飞机状态）*/
				temp_pos_drone[0] = px4_pose_[0];
				temp_pos_drone[1] = px4_pose_[1];
				temp_pos_drone[2] = px4_pose_[2];
				TrackingState = CHECKING;
				cout << "CHECKING" <<endl;
			}
				//cout << "WAITING" <<endl;
			break;
		case CHECKING:
		    /*如果有问题自动降落 并设置为等待模式重新传入位置 没有问题则设置为PREPARE模式，起飞到指定高度*/
			if(px4_pose_[0] == 0 && px4_pose_[1] == 0) 			//没有位置信息则执行降落模式
			{
				cout << "Check error, make sure have local location" <<endl;
				mode_cmd_.request.custom_mode = "AUTO.LAND";  //由mavros_msgs::SetMode包  请求设置模式为自动着陆
				set_mode_client_.call(mode_cmd_);
				TrackingState = WAITING;	
			}
			else
			{
				TrackingState = PREPARE;
				cout << "PREPARE" <<endl;
			}
			
			break;
		case PREPARE:											//起飞到指定高度
			posxyz_target[0] = temp_pos_drone[0];
			posxyz_target[1] = temp_pos_drone[1];
			posxyz_target[2] = search_alt_;  //X Y上的位置信息不变，Z轴也就是高度上飞到search_alt_，在launch文件中search_alt_设为1.5m
			/*在高度小于一定的误差范围之内，切换为SEARCH搜索状态*/
			if((px4_pose_[2]<=search_alt_+0.1) && (px4_pose_[2]>=search_alt_-0.1))
			{
				TrackingState = SEARCH;
			}
			else if(detect_state == true)
			{
			//	TrackingState = SEARCH;
			}
			OffboardControl_.send_pos_setpoint(posxyz_target, 0);					
			if(px4_state_.mode != "OFFBOARD")				//如果在准备中途中切换到onboard，则跳到WAITING
			{
				TrackingState = WAITING;
			}

			break;
		case SEARCH:
		    /*无人机不会主动搜索目标，在尝试一次搜索且没有成功的情况下会重新发送位置且回到WATTING模式*/
			if(detect_state == true)
			{
				TrackingState = TRACKING;
			  cout << "TRACKING" <<endl;
			}	
			else//这里无人机没有主动搜寻目标
			{
				OffboardControl_.send_pos_setpoint(posxyz_target, 0);
			}
			if(px4_state_.mode != "OFFBOARD")				//如果在SEARCH途中切换到onboard，则跳到WAITING
			{
				TrackingState = WAITING;
			}
     // cout << "SEARCH" <<endl;
			break;
		case TRACKING:
		    /*search检测到目标，跳到TRACKING*/
			{
				/*如果检测到目标，进入pid追踪函数TrackingPidProcess（）*/
				if(detect_state == true)
				{
					/**ar_pose为飞机相对于二维码的位置 desire_pose为飞机期望的位置
					 * ar_pose_在ArPoseCallback（）中被赋值，而ArPoseCallback（）函数为订阅器ar_pose_sub_的回调函数
					 * 过程为此cpp文件通过订阅"/ar_pose_marker"话题，调用回调函数ArPoseCallback（）
					 * 然后在ArPoseCallback（）为ar_pose_赋值，得到飞机相对于二维码的位置
					 * desire_pose_在launch文件中做了初始化，目的是飞机始终与二维码在同一轴且与二维码保持6.5m的距离*
					 * */
					desire_vel_ = TrackingPidProcess(ar_pose_,desire_pose_,orientation); 

					//cout << "search_" <<endl;
				}
			  else
				{
					desire_vel_[0] = 0;
					desire_vel_[1] = 0;
					desire_vel_[2] = 0;
					desire_vel_[3] = 0;
				}
			
				if(px4_state_.mode != "OFFBOARD")			//如果在TRACKING中途中切换到onboard，则跳到WAITING
				{
					TrackingState = WAITING;
				}

				desire_xyzVel_[0] = desire_vel_[0];  //机体x轴前后
				desire_xyzVel_[1] = desire_vel_[2];   //机体z轴上下
				desire_yawVel_    = desire_vel_[1];     //机体y轴yaw
				desire_xyzVel_[2] = desire_vel_[3];  //机体y轴，此处为roll的速度 对应send_body_velyz_setpoint（）函数中的x
                /*通过将desire_yzVel_和desire_yawVel_传入到OffboardControl模块的send_body_velyz_setpoint进行控制*/
				OffboardControl_.send_body_velyz_setpoint(desire_xyzVel_,desire_yawVel_); 
   
			}

			break;
		case TRACKOVER:
			{
				mode_cmd_.request.custom_mode = "AUTO.LAND";
        set_mode_client_.call(mode_cmd_);
				TrackingState = WAITING;
			}

			break;

		default:
			cout << "error" <<endl;
	}	

}

/*接收降落板相对飞机的位置以及偏航角*/
void PX4Tracking::ArPoseCallback(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr &msg)
{
	detect_state = false;
	for(auto &item : msg->markers)
	{
		if(item.id == markers_id_)
		{
			detect_state = true;
      ar_pose_[0] = item.pose.pose.position.x;
      ar_pose_[1] = item.pose.pose.position.y;
      ar_pose_[2] = item.pose.pose.position.z;

	  orientation[0]=item.pose.pose.orientation.x;
	  orientation[1]=item.pose.pose.orientation.y;
	  orientation[2]=item.pose.pose.orientation.z;
	  orientation[3]=item.pose.pose.orientation.w;

//			cout << "ar_pose_[0]:"  << ar_pose_[0] << endl;
//			cout << "ar_pose_[1]:"  << ar_pose_[1] << endl;
//			cout << "ar_pose_[2]:"  << ar_pose_[2] << endl;
//			cout << "markers_yaw_: "  << markers_yaw_ << endl;
		}
	}
//	cout << "detect_state :" << detect_state << endl;
}

void PX4Tracking::BoxPoseCallback(const geometry_msgs::PointStamped::ConstPtr &msg)
{
			detect_state = true;
      		ar_pose_[0] = msg->point.x;
     		ar_pose_[1] = msg->point.y;
      		ar_pose_[2] = msg->point.z;

	  		orientation[0]=0;
	  		orientation[1]=0;
	  		orientation[2]=0;
	  		orientation[3]=0;
}

/*接收来自飞控的当前飞机位置*/                  
void PX4Tracking::Px4PosCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    // Read the Drone Position from the Mavros Package [Frame: ENU]
    Eigen::Vector3d pos_drone_fcu_enu(msg->pose.position.x,msg->pose.position.y,msg->pose.position.z);

    px4_pose_ = pos_drone_fcu_enu;
}
/*接收来自飞控的当前飞机状态*/
void PX4Tracking::Px4StateCallback(const mavros_msgs::State::ConstPtr& msg)
{
	px4_state_ = *msg;
}

/*初始化*/
/*此处做很多参数在launch文件中也做了初始化*/
void PX4Tracking::Initialize()
{
  //读取offboard模式下飞机的搜索高度
  nh_private_.param<float>("search_alt_", search_alt_, 2);

  nh_private_.param<float>("markers_id_", markers_id_, 4.0);

  nh_private_.param<float>("PidY_p", s_PidY.p, 0.6);
  nh_private_.param<float>("PidY_d", s_PidY.d, 0.01);
  nh_private_.param<float>("PidY_i", s_PidY.i, 0);
  nh_private_.param<float>("PidZ_p", s_PidZ.p, 0.6);
  nh_private_.param<float>("PidZ_d", s_PidZ.d, 0.01);
  nh_private_.param<float>("PidZ_i", s_PidZ.i, 0);
  nh_private_.param<float>("PidYaw_p", s_PidYaw.p, 0.4);
  nh_private_.param<float>("PidYaw_d", s_PidYaw.d, 0.01);
  nh_private_.param<float>("PidYaw_i", s_PidYaw.i, 0);

  //期望的飞机相对二维码的位置
	float desire_pose_x,desire_pose_y,desire_pose_z;
  nh_private_.param<float>("desire_pose_x", desire_pose_x, 6.5);
  nh_private_.param<float>("desire_pose_y", desire_pose_y, 0);
  nh_private_.param<float>("desire_pose_z", desire_pose_z, 0);
  desire_pose_[0] = desire_pose_x;
  desire_pose_[1] = desire_pose_y;
  desire_pose_[2] = desire_pose_z;

  detect_state = false;
  desire_vel_[0] = 0;
  desire_vel_[1] = 0;
  desire_vel_[2] = 0;
	desire_xyzVel_[0]  = 0;
	desire_xyzVel_[1]  = 0;
	desire_xyzVel_[2]  = 0;

  s_PidItemY.tempDiffer = 0;
  s_PidItemYaw.tempDiffer = 0;
  s_PidItemZ.tempDiffer = 0;
  s_PidItemY.intergral = 0;
  s_PidItemYaw.intergral = 0;
  s_PidItemZ.intergral = 0;
  s_PidItemRoll.intergral = 0;
}
int main(int argc, char** argv) {
  ros::init(argc,argv,"tracking_quadrotor");
  ros::NodeHandle nh("");
  ros::NodeHandle nh_private("~");

  PX4Tracking PX4Tracking(nh, nh_private);

  ros::spin();
  return 0;
}
