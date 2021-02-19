#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PointStamped.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Bool.h>
#include <std_msgs/UInt32.h>
#include <ar_track_alvar_msgs/AlvarMarkers.h>

using namespace std;
geometry_msgs::PointStamped	target_pos;

ros::Publisher	ar_pub_;
ros::Subscriber ar_sub ;


void arposCallback(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr &msg)
{
   target_pos.header.frame_id="camera_optical_frame"; 
   target_pos.header.stamp	 = ros::Time::now();
   target_pos.point.x=  msg->markers[0].pose.pose.position.x;
   target_pos.point.y= msg->markers[0].pose.pose.position.y;
   target_pos.point.z= msg->markers[0].pose.pose.position.z;
   ar_pub_.publish(target_pos);
   cout<<"发布"<<endl;
}

int main(int argc,char** argv){
    ros::init(argc, argv, "ar_pose");
	ros::NodeHandle n;
    ros::Rate rate(10.0);
    ar_sub = n.subscribe("/ar_pose_marker", 1, arposCallback);
 
    ar_pub_ = n.advertise<geometry_msgs::PointStamped>( "/ArUco_pose", 1);
    while(ros::ok())
    {
        rate.sleep();
    	ros::spinOnce();	
    }
    return 0;
}
