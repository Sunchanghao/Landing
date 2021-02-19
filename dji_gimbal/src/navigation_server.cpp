#include <dji_gimbal/navigation_server.h>

NavigationServer::NavigationServer(std::string name):
as_(node_handle, name, boost::bind(&NavigationServer::executeNavigate, this, _1), false)
{
  as_.start();
  ROS_INFO("dji_gimbal::navigation_server started");

  // initialize marker topic
  goal_marker = node_handle.advertise<visualization_msgs::Marker>( "/goal_marker", 0 );

  // initialize twist topic
  // second argument is number of cached messages on the topic
  cmd_vel_pub_ = node_handle.advertise<geometry_msgs::Twist>( "/cmd_vel", 1 );

  // initialize goal point
  goal_point.x = 0.0;
  goal_point.y = 0.0;
  goal_point.z = 0.0;
}

visualization_msgs::Marker NavigationServer::setupMarker()
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

void NavigationServer::broadcastGoalTF()
{
  tf::Transform transform;
  
  // setup transformation - relative position of goal with respect to world
  transform.setOrigin( tf::Vector3(goal_point.x, goal_point.y, goal_point.z) );
  transform.setRotation( tf::Quaternion(0, 0, 0, 1) );

  // publish transform
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "goal"));
}

void NavigationServer::publishTwist()
{
  geometry_msgs::Twist cmd;

  tf::StampedTransform transform;

  try
  {
    listener.lookupTransform("/base_link", "/goal",
                             ros::Time(0), transform);
  }
  catch (tf::TransformException &ex)
  {
    ROS_ERROR("%s",ex.what());
    ros::Duration(1.0).sleep();
    return;
  }

  // get to desired altitude
  float heightDist = transform.getOrigin().z();
  if ( heightDist > hTresh ) {
    // we need to raise up
    cmd.linear.z = 0.3;
  } else if ( heightDist < -hTresh ) {
    // we need to dive down
    cmd.linear.z = -0.3;
  } else if ( abs(heightDist) < hTresh ) {
    // we are roughly in the same altitude as the goal so we stop any vertical movement
    cmd.linear.z = 0;

    // ---- set angular speed ----
    cmd.angular.z = 3.0 * atan2(    transform.getOrigin().y(),
                                    transform.getOrigin().x());

    // set maximum angular speed
    if ( cmd.angular.z > 1)
    {
      cmd.angular.z = 1;
    }
   
    // if we are almost "facing" the goal
    if ( abs(cmd.angular.z) < 0.3) {
    cmd.linear.x = 0.4 * sqrt(  pow(transform.getOrigin().x(), 2) +
                                pow(transform.getOrigin().y(), 2)
                              );
      // set maximum linear speed
      if ( cmd.linear.x > 4)
      {
        cmd.linear.x = 4;
      }
    } else {
      cmd.linear.x = 0;
    }
  }

  cmd_vel_pub_.publish(cmd);  

  // update distance to the goal
  dist = sqrt(  pow(transform.getOrigin().x(), 2) +
                pow(transform.getOrigin().y(), 2) +
                pow(transform.getOrigin().z(), 2)
              );

  float angle = atan2(  transform.getOrigin().x(),
                        transform.getOrigin().z());
  
  ROS_INFO("origin [X,Y,Z] = [ %f, %f, %f ]",transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z());
}

void NavigationServer::executeNavigate(const dji_gimbal::NavigateGoalConstPtr& goal)
{
  ROS_INFO("dji_gimbal::navigation_server: executeNavigate");
  
  geometry_msgs::Twist empty;

  dist = 20.0;

  // set goal_point
  goal_point.x = goal->target_pos.x;
  goal_point.y = goal->target_pos.y;
  goal_point.z = goal->target_pos.z;

  ros::Rate loop_rate(100);

  while(ros::ok() && (dist > dTresh))
  {
    // broadcast static goal->world transformation
    broadcastGoalTF();

    // publish marker
    goal_marker.publish( setupMarker() );

    // publish twist message based on robot_base -> goal transform
    publishTwist();      

    ros::spinOnce();
    loop_rate.sleep();
  }

  as_.setSucceeded();
}

int main(int argc, char** argv)
{

    ros::init(argc, argv, "nav_server");

    ROS_INFO("Starting navigation_server node.");

    ros::NodeHandle nh;
    NavigationServer nav_server("nav_server");

    ros::spin();

    return 0;
}
