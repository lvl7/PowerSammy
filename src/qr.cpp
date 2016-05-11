#include <ros/ros.h>
//#include <string>
//#include <sensor_msgs/LaserScan.h>
//#include <cstdlib>
#include <stdlib.h>


#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>

void qrMessReceived(std_msgs::String);


ros::Publisher pub;
ros::Subscriber sub;


void qrMessReceived(std_msgs::String msg)
{
	std::string git = "https://github.com/lvl7/PowerSammy/wiki";
	//std_msgs::String gitS;
	//gitS.data = git;
	ROS_INFO_STREAM(msg);
	geometry_msgs::Twist vel;
	//   vel.linear.x = vel.linear.y = vel.linear.z = 0;
    vel.angular.x = vel.angular.y = vel.angular.z = 0;
	
	if(!strcmp(git.c_str(), msg.data)) vel.linear.x = 0.03;
	pub.publish(msg);
	
}

int main (int argc, char** argv)
{
	ros::init(argc, argv, "qr");
	ros::NodeHandle node;
	ros::Rate rate(10);

    pub = node.advertise<geometry_msgs::Twist>("/rosaria/cmd_vel", 100);
    sub = node.subscribe("/visp_auto_tracker/code_message", 100, &qrMessReceived);

	while(ros::ok())
	{
		ros::spinOnce();
		rate.sleep();
	}
	
	ros::spin();
	return 0;
}
