#include "dji_gimbal/goal_broadcaster.h"

GoalBroadcaster::GoalBroadcaster(ros::NodeHandle &nh): node_handle(nh)
{
  goal_marker = node_handle.advertise<visualization_msgs::Marker>( "goal_marker", 0 );

  goal_pos = node_handle.subscribe("/goal_pos",1000, &GoalBroadcaster::updateGoalPos,this);

  goal_point.x = 0.0;
  goal_point.y = 0.0;
  goal_point.z = 0.0;

  tf::TransformBroadcaster br;
  tf::Transform transform;

  ros::Rate loop_rate(1000);

  while(ros::ok())
  {
    // publish marker
    goal_marker.publish( setupMarker() );

    // setup transformation - relative position of goal with respect to world
    transform.setOrigin( tf::Vector3(goal_point.x, goal_point.y, goal_point.z) );
    transform.setRotation( tf::Quaternion(0, 0, 0, 1) );

    // publish transform
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "goal"));

    loop_rate.sleep();
    ros::spinOnce();
  }
}

visualization_msgs::Marker GoalBroadcaster::setupMarker()
{
  visualization_msgs::Marker marker;
  marker.header.frame_id = "world";
  marker.header.stamp = ros::Time();
  marker.ns = "mq";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::SPHERE;
  marker.action = visualization_msgs::Marker::ADD;
  // update marker position according to goal_point
  marker.pose.position.x = goal_point.x;
  marker.pose.position.y = goal_point.y;
  marker.pose.position.z = goal_point.z;

  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 0.1;
  marker.scale.y = 0.1;
  marker.scale.z = 0.1;
  marker.color.a = 1.0; // Don't forget to set the alpha!
  marker.color.r = 0.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;

  return marker;
}

void GoalBroadcaster::updateGoalPos(const geometry_msgs::Point &pos)
{
  ROS_INFO("goal_broadcaster: updating goal position");

  goal_point.x = pos.x;
  goal_point.y = pos.y;
  goal_point.z = pos.z;

  ROS_INFO("[X,Y,Z] = [%f, %f, %f]",goal_point.x,goal_point.y,goal_point.z);
}

int main(int argc, char** argv)
{

    ros::init(argc, argv, "goal_broadcaster");

    ROS_INFO("Starting goal_broadcaster node.");

    ros::NodeHandle nh;
    GoalBroadcaster gb(nh);

    ros::spin();

    return 0;
}
