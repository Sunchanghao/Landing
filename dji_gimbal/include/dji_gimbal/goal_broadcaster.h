#ifndef DJI_GIMBAL_GOAL_BROADCASTER_H
#define DJI_GIMBAL_GOAL_BROADCASTER_H

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include <tf/transform_broadcaster.h>

/*
input:

goal_pos - point in 3D space

output:

goal_marker - topic used for goal point visualization in rviz
static transformation goal -> world - used for navigation in turtle style (computing twist message based on position of robot and its goal)

*/

class GoalBroadcaster {

public:
  
  GoalBroadcaster(ros::NodeHandle &nh);

private:

  void updateGoalPos(const geometry_msgs::Point &point);
  visualization_msgs::Marker setupMarker();

  ros::NodeHandle node_handle;
  ros::Publisher goal_marker;
  ros::Subscriber goal_pos;

  // point where we want to go
  geometry_msgs::Point goal_point;

};

#endif
