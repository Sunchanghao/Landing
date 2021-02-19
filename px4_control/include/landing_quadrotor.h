#include <ros/ros.h>
#include <iostream>
#include <ar_track_alvar_msgs/AlvarMarkers.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/PositionTarget.h>
#include <std_msgs/Bool.h>
#include <std_msgs/UInt32.h>
#include "offboard_control.h"
#include "px4_control_cfg.h"
#include <tf/transform_datatypes.h>
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>
using namespace std;
using namespace Eigen;
class PX4Landing {
 public:
    /**
     *默认构造函数
     */
    PX4Landing(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);
    /**
     * 析构函数
     */
    ~PX4Landing();
    void Initialize();
   OffboardControl OffboardControl_;
 private:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;
  ros::Timer cmdloop_timer_;
  void CmdLoopCallback(const ros::TimerEvent& event);
  void LandingStateUpdate();
  void ArPoseCallback(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr &msg);
  void Px4PosCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);
  void Px4StateCallback(const mavros_msgs::State::ConstPtr& msg);
  Eigen::Vector4d LandingPidProcess(Eigen::Vector3d &currentPos,float currentYaw,Eigen::Vector3d &expectPos,float expectYaw);
  Eigen::Vector3d temp_pos_drone;
  Eigen::Vector3d posxyz_target;//期望飞机的空间位置
  Eigen::Vector3d velxy_posz_target;//offboard模式下，发送给飞控的期望值
  Eigen::Vector3d  ar_pose_;  //降落板相对飞机位置
  Eigen::Vector3d  px4_pose_; //接收来自飞控的飞机位置 
  Eigen::Vector3d desire_pose_;//期望的飞机相对降落板的位置
	float desire_yaw_;//期望的飞机相对降落板的偏航角
  mavros_msgs::State px4_state_;//飞机的状态
  mavros_msgs::SetMode mode_cmd_;
  float search_alt_;
  float markers_id_;//需要检测到的二维码，默认是4
	float markers_yaw_;//二维码相对飞机的偏航角
  bool detect_state;//是否检测到降落板标志位
  Eigen::Vector4d desire_vel_;
	Eigen::Vector3d desire_xyVel_;
	float desire_yawVel_;
  S_PID s_PidXY,s_PidZ,s_PidYaw;
  S_PID_ITEM s_PidItemX;
  S_PID_ITEM s_PidItemY;
  S_PID_ITEM s_PidItemZ;
  S_PID_ITEM s_PidItemYaw;
  enum
 {
  WAITING,		//等待offboard模式
  CHECKING,		//检查飞机状态
  PREPARE,		//起飞到指定高度
  SEARCH,		//搜索
  LANDING,	        //检测到降落板，开始降落
  LANDOVER,		//结束		
}LandingState = WAITING;//初始状态WAITING

  ros::Subscriber ar_pose_sub_;
  ros::Subscriber position_sub_;
  ros::Subscriber state_sub_;
  ros::ServiceClient set_mode_client_;
};
