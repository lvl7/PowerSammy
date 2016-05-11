#include <ros/ros.h>
#include <string>
//#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>

void qrMessReceived(std::string);

ros::NodeHandle node;
ros::Publisher pub = node.advertise<geometry_msgs::Twist>("/rosaria/cmd_vel", 100);
ros::Subscriber sub = node.subscribe("/visp_auto_tracker/code_message", 100, &qrMessReceived);


void qrMessReceived(std::string msg)
{
	std::string git = "https://github.com/lvl7/PowerSammy/wiki";
	ROS_INFO_STREAM(msg);
	geometry_msgs::Twist vel;
    vel.linear.x = vel.linear.y = vel.linear.z = 0;
    vel.angular.x = vel.angular.y = vel.angular.z = 0;
	
	if(msg == git) vel.linear.x = 0.3;
	pub.publish(msg);
	
}

int main (int argc, char** argv)
{
	ros::init(argc, argv, "qr");
	ros::Rate rate(10);

//	while(ros::ok())
//	{
//		ros::spinOnce();
//		rate.sleep();
//	}
	
	ros::spin();
	return 0;
}
