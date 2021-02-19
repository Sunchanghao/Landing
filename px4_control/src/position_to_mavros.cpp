/***************************************************************************************************************************
*
* Author: bingo
* Email: bingobin.lw@gmail.com
* Time: 2019.12.17
* Description: Sending Position Information to mavros,The location information may come from lidar or t265

***************************************************************************************************************************/


#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
ros::Publisher position_pub;
using namespace std;
int main(int argc, char** argv){
  ros::init(argc, argv, "position_to_mavros");

  ros::NodeHandle node("~");

  geometry_msgs::PoseStamped cur_position;

  position_pub = node.advertise<geometry_msgs::PoseStamped>("/mavros/vision_pose/pose", 10);
  ros::Publisher body_path_pubisher = node.advertise<nav_msgs::Path>("body_frame/path", 1);

  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);

  //view path in rviz
  nav_msgs::Path body_path;

  std::string target_frame_id = "carto_odom";
  std::string source_frame_id = "base_link";
  double output_rate = 30, roll_obj = 0, pitch_obj = 0, yaw_obj = 0;

	node.getParam("target_frame_id", target_frame_id);
	node.getParam("source_frame_id", source_frame_id);
	node.getParam("output_rate", output_rate);
	node.getParam("roll_obj", roll_obj);
	node.getParam("pitch_obj", pitch_obj);
	node.getParam("yaw_obj", yaw_obj);

	cout<<"target_frame_id:"<<target_frame_id<<endl;
	cout<<"source_frame_id:"<<source_frame_id<<endl;
	cout<<"output_rate:"<<output_rate<<endl;
	cout<<"roll_obj:"<<roll_obj<<endl;
	cout<<"pitch_obj:"<<pitch_obj<<endl;
	cout<<"yaw_obj:"<<yaw_obj<<endl;

  ros::Rate rate(output_rate);
  while (node.ok()){
    geometry_msgs::TransformStamped transformStamped;
    try{
      transformStamped = tfBuffer.lookupTransform(target_frame_id, source_frame_id,
      ros::Time(0),ros::Duration(3.0));

        static tf2::Quaternion quat_obj, quat_body;
	quat_obj = tf2::Quaternion(transformStamped.transform.rotation.x,transformStamped.transform.rotation.y,transformStamped.transform.rotation.z,transformStamped.transform.rotation.w);

        quat_body.setRPY( roll_obj, pitch_obj, yaw_obj);
        //ROS_INFO_STREAM(quat_body);
        quat_body = quat_obj * quat_body;
        quat_body.normalize();

        cur_position.pose.position.x = transformStamped.transform.translation.x ;
        cur_position.pose.position.y = transformStamped.transform.translation.y ;
        cur_position.pose.position.z = transformStamped.transform.translation.z ;

        cur_position.pose.orientation.x = quat_body.x();
        cur_position.pose.orientation.y = quat_body.y();
        cur_position.pose.orientation.z = quat_body.z();
        cur_position.pose.orientation.w = quat_body.w();
        cur_position.header.stamp = ros::Time::now();
        cur_position.header.frame_id = transformStamped.header.frame_id;
        position_pub.publish(cur_position);

        body_path.header.stamp = cur_position.header.stamp;
        body_path.header.frame_id = cur_position.header.frame_id;
        body_path.poses.push_back(cur_position);
        body_path_pubisher.publish(body_path);
    }
    catch (tf2::TransformException &ex) {
      ROS_WARN("%s",ex.what());
      ros::Duration(1.0).sleep();
      continue;
    }
    rate.sleep();
  }
  return 0;
};


