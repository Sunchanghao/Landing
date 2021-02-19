#include <ros/ros.h>
#include <tf/tf.h>
#include <math.h>
#include <actionlib/client/simple_action_client.h>
#include <dji_gimbal/NavigateAction.h>
#include <geometry_msgs/Point.h>

void goalCallback(const geometry_msgs::Point::ConstPtr &msg)
{
    // create the action client
    // true causes the client to spin its own thread
    actionlib::SimpleActionClient<dji_gimbal::NavigateAction> ac("nav_server", true);

    ROS_INFO("goalCallback: Waiting for action server to start.");
    // wait for the action server to start
    ac.waitForServer(); //will wait for infinite time

    ROS_INFO("goalCallback: Navigation service found.");

    // setup goal
   dji_gimbal::NavigateGoal goal;
    
    goal.target_pos.x = msg->x;
    goal.target_pos.y = msg->y;
    goal.target_pos.z = msg->z;

    ac.sendGoalAndWait(goal);

    if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        ROS_INFO("Navigate action is completed!");
    }
    else
    {
        ROS_ERROR("Navigate action failed!");
    }
}


int main (int argc, char **argv)
{
    ros::init(argc, argv, "test_navigate_client");

    ros::NodeHandle nh;

    ros::Subscriber get_goal_point = nh.subscribe("/get_goal", 10, goalCallback);
    
    ros::spin();
    
    return 0;
};
