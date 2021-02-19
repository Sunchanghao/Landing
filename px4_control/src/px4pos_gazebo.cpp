//头文件
#include <ros/ros.h>

#include <iostream>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <tf/transform_listener.h>

//msg 头文件
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Float64.h>
#include <tf2_msgs/TFMessage.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/Range.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <gazebo_msgs/ModelState.h>
#include <gazebo_msgs/ModelStates.h>

using namespace std;

//---------------------------------------相关参数-----------------------------------------------
gazebo_msgs::ModelStates model_states;
//---------------------------------------gazebo真值相关------------------------------------------
Eigen::Vector3d pos_drone_gazebo;
Eigen::Quaterniond q_gazebo;
Eigen::Vector3d Euler_gazebo;
//---------------------------------------发布相关变量--------------------------------------------
ros::Publisher vision_pub;
ros::Publisher drone_state_pub;
ros::Publisher message_pub;
ros::Publisher odom_pub;
ros::Publisher trajectory_pub;
nav_msgs::Odometry Drone_odom;
std::vector<geometry_msgs::PoseStamped> posehistory_vector_;

ros::Publisher	base_pub_;
geometry_msgs::PointStamped	output_point;
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>函数声明<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
void send_to_fcu();

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>回调函数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
void gazebo_cb(const gazebo_msgs::ModelStates::ConstPtr &msg)
{
        model_states = *msg;
        pos_drone_gazebo = Eigen::Vector3d(model_states.pose[1].position.x, model_states.pose[1].position.y, model_states.pose[1].position.z);
        q_gazebo = Eigen::Quaterniond(model_states.pose[1].orientation.w, model_states.pose[1].orientation.x, model_states.pose[1].orientation.y, model_states.pose[1].orientation.z);
        output_point.header.stamp	 = ros::Time::now();
		output_point.point.x		 = pos_drone_gazebo[0];
		output_point.point.y		 = pos_drone_gazebo[1];
		output_point.point.z		 = pos_drone_gazebo[2];
		output_point.header.frame_id = "gazebo";
         base_pub_.publish( output_point );
}

// void send_to_fcu()
// {
//     geometry_msgs::PoseStamped vision;
//     vision.pose.position.x = pos_drone_gazebo[0];
//     vision.pose.position.y = pos_drone_gazebo[1];
//     vision.pose.position.z = pos_drone_gazebo[2];
//     vision.pose.orientation.x = q_gazebo.x();
//     vision.pose.orientation.y = q_gazebo.y();
//     vision.pose.orientation.z = q_gazebo.z();
//     vision.pose.orientation.w = q_gazebo.w();
//     vision.header.stamp = ros::Time::now();
//     vision_pub.publish(vision);
// }

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>主 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
int main(int argc, char **argv)
{
    ros::init(argc, argv, "px4pos_gazebo");
    ros::NodeHandle nh("~");
    // 【订阅】gazebo仿真真值
    ros::Subscriber gazebo_sub = nh.subscribe<gazebo_msgs::ModelStates>("/gazebo/model_states", 100, gazebo_cb);
    //vision_pub = nh.advertise<geometry_msgs::PoseStamped>("/mavros/vision_pose/pose", 100);
    base_pub_ = nh.advertise<geometry_msgs::PointStamped>( "/target_gazebo_pos", 10 );
    // 频率
    ros::Rate rate(100.0);
    while (ros::ok())
    {
        //回调一次 更新传感器状态
        ros::spinOnce();
        // 将采集的机载设备的定位信息及偏航角信息发送至飞控，根据参数input_source选择定位信息来源
        //send_to_fcu();
        rate.sleep();
    }
    return 0;
}


