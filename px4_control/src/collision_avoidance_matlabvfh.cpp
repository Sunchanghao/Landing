/***************************************************************************************************************************
*
* Author: bingo
* Email: 1554459957@qq.com
* Time: 2019.10.14
* Description: lidar collision v1.0
*  
***************************************************************************************************************************/

//ROS 头文件
#include <ros/ros.h>
#include <Eigen/Eigen>


//topic 头文件
#include <iostream>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/PositionTarget.h>
#include <std_msgs/Float64MultiArray.h>
using namespace std;
Eigen::Vector3d vel_sp;     
float desire_z = 1.5; //期望高度                                 
void targetVel_cb(const std_msgs::Float64MultiArray::ConstPtr &msg)
{
  vel_sp[0] =  msg->data.at(0);
  vel_sp[1] =  msg->data.at(1);
  ROS_INFO("I heard: [%f],[%f]", msg->data.at(0),msg->data.at(1));
}

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>主 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
int main(int argc, char **argv)
{
    ros::init(argc, argv, "collision_avoidance_matlabvfh");
    ros::NodeHandle nh("~");
    ros::Rate rate(20.0);

    //【订阅】Lidar数据
    ros::Subscriber lidar_sub = nh.subscribe<std_msgs::Float64MultiArray>("/cmd_vel", 100, targetVel_cb);

    ros::Publisher setpoint_raw_local_pub = nh.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 100);

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>Main Loop<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
    while(ros::ok())
    {

		ros::spinOnce();

		mavros_msgs::PositionTarget pos_setpoint;
		pos_setpoint.type_mask = 1 + 2 + /*4 + 8 + 16 + 32 +*/ 64 + 128 + 256 + 512 + 1024 + 2048;
		pos_setpoint.coordinate_frame = 1;
		pos_setpoint.velocity.x = vel_sp[0];
		pos_setpoint.velocity.y = vel_sp[1];
		pos_setpoint.position.z = desire_z;
		//  pos_setpoint.yaw = 1.5;
		setpoint_raw_local_pub.publish(pos_setpoint);
		rate.sleep();
    }
    return 0;

}



