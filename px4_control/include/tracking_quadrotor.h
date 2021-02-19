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
#include <Eigen/Eigen>  //Eigen库支持线性代数，矩阵运算等
#include <Eigen/Dense>  
#include <Eigen/Geometry>  //包括2D图像的平移旋转缩放 3d图像的旋转（四元数、轴角）
#include <Eigen/Eigenvalues>  //特征值特征向量
#include "nav_msgs/Odometry.h"
#include <gazebo_msgs/ModelState.h>
#include <gazebo_msgs/ModelStates.h>
using namespace std;
using namespace Eigen;
class PX4Tracking {
 public:
    /**
     *默认构造函数
     */
    PX4Tracking(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);  //定义了两个ros的句柄
    /**
     * 析构函数
     */
    ~PX4Tracking();
    void Initialize();    //定义初始化函数
   OffboardControl OffboardControl_;  //offboard_control.h 中的类 
 private:
  ros::NodeHandle nh_;    //构造函数中的参数
  ros::NodeHandle nh_private_;   //构造函数中的参数
  ros::Timer cmdloop_timer_;   //定义时间
  void CmdLoopCallback(const ros::TimerEvent& event);  //时间的回调函数
  void TrackingStateUpdate();    //状态机更新
  void BoxPoseCallback(const geometry_msgs::PointStamped::ConstPtr &msg);  //新加的
  void ArPoseCallback(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr &msg);  //<ar_track_alvar_msgs/AlvarMarkers.h>这个包
  void Px4PosCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);
  void Px4StateCallback(const mavros_msgs::State::ConstPtr& msg);
  void Px4PoseCallback(const nav_msgs::Odometry::ConstPtr& msg);
  void gazebo_cb(const gazebo_msgs::ModelStates::ConstPtr &msg);
  Eigen::Vector4d TrackingPidProcess(Eigen::Vector3d &currentPos,Eigen::Vector3d &expectPos,Eigen::Vector4d &orientation);//参数为当前位置和期望位置，由这两个参数作为控制PID进程的输入
  Eigen::Vector4d TrackingPidProcess2(Eigen::Vector3d &relative_pos,Eigen::Vector3d &expectPos,Eigen::Quaterniond &q_gazebo);
  Eigen::Vector3d temp_pos_drone;
  Eigen::Vector3d posxyz_target;//期望飞机的空间位置
  Eigen::Vector3d search_target;//期望飞机的search位置
  Eigen::Vector3d  ar_pose_;  //二维码相对飞机位置
  Eigen::Vector3d  yolo_pose_;  //yolo相对飞机位置
  Eigen::Vector4d  orientation; //定义四元数变量 
  Eigen::Vector4d  yolo_orientation; //定义yolo四元数变量 

  Eigen::Vector3d  px4_pose_; //接收来自飞控的飞机位置 
  Eigen::Vector3d  relative_pos; //gazebo真值下的相对位置差
  Eigen::Vector3d desire_pose_;//期望的飞机相对二维码的位置  这个位置坐标有说法 涉及到一个飞机和二维码的位置坐标转换
  Eigen::Vector3d yolo_desire_pose_;//期望的飞机相对二维码的位置  这个位置坐标有说法 涉及到一个飞机和二维码的位置坐标转换

  Eigen::Vector3d pos_drone_gazebo;//gazebo真值
  Eigen::Quaterniond q_gazebo;//gazebo真值
  gazebo_msgs::ModelStates model_states;
  double px4_q_z;//获取飞机的四元数的z
  double relative_q;//相对四元数

  mavros_msgs::State px4_state_;//飞机的状态
  nav_msgs::Odometry px4_odom_;
  mavros_msgs::SetMode mode_cmd_; 
  mavros_msgs::CommandBool disarm_cmd_; 
  float search_alt_;
  float markers_id_;//需要检测到的二维码，默认是4
  bool detect_state;//是否检测到二维码标志位
  bool yolo_state;//yolo是否检测到标志位
  //在此处作出了更改  将desire_vel_由原来的3维改为了4维
  Eigen::Vector4d desire_vel_;
  //Eigen::Vector3d desire_yzVel_;
  Eigen::Vector3d desire_xyzVel_;   //此处也作出了更改 改为了XYZ
  Eigen::Vector3d attitude_rate_sp;   
  float thrust_sp;
  double desire_angleVel_;   //**********在此处加了角速度变量
  // float thrust_sp;        //**************在此处加了一个推力

	float desire_yawVel_;
  S_PID s_PidY,s_PidZ,s_PidYaw;
  S_PID_ITEM s_PidItemY;
  S_PID_ITEM s_PidItemZ;
  S_PID_ITEM s_PidItemYaw;
  S_PID_ITEM s_PidItemRoll; //此处为后加的
  enum
 {
  WAITING,		//等待offboard模式
  CHECKING,		//检查飞机状态
  PREPARE,		//起飞到指定高度
  SEARCH,		//搜索
  TRACKING,	        //检测到二维码，开始跟踪
  TRACKOVER,	        //结束		
}TrackingState = WAITING;//初始状态WAITING

  ros::Subscriber ar_pose_sub_;
  ros::Subscriber gazebo_sub;
  ros::Subscriber position_sub_;
  ros::Subscriber state_sub_;
  ros::Subscriber pose_sub_;
  ros::ServiceClient set_mode_client_;
  ros::Subscriber box_pose_sub_;
  ros::ServiceClient set_arm_client_;
};
