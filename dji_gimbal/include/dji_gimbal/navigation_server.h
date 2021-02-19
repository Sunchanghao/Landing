#ifndef DJI_GIMBAL_NAVIGATION_SERVER_H
#define DJI_GIMBAL_NAVIGATION_SERVER_H

#include <ros/ros.h>
#include <ros/console.h>
#include <tf/tf.h>
#include <math.h>
#include <actionlib/server/simple_action_server.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <dji_gimbal/NavigateAction.h>

/*
input:

action request - goal point

tf - frame "world" must exist

output:

cmd_vel - topic for quadrotor movement control

*/

class NavigationServer {

public:

  NavigationServer(std::string name);
  void executeNavigate(const dji_gimbal::NavigateGoalConstPtr& goal);

private:

  visualization_msgs::Marker setupMarker();
  void broadcastGoalTF();
  void publishTwist();

  // current goal
  geometry_msgs::Point goal_point;  

  ros::NodeHandle node_handle;
 
  // goal marker publisher - for visualization in rviz
  ros::Publisher goal_marker;

  // geometry twist publisher - for quadcopter movement control
  ros::Publisher cmd_vel_pub_;

  // The action server
  actionlib::SimpleActionServer<dji_gimbal::NavigateAction> as_;

  // static tf broadcaster: goal -> world
  tf::TransformBroadcaster br;

  // tf listener to get transformation: robot_base -> goal
  tf::TransformListener listener;

  // distance from goal
  float dist;

  static constexpr float hTresh = 0.3;
  static constexpr float dTresh = 0.1;

};

#endif
