#include <ros/ros.h>
#include "simplecanny.h"

int main(int argc, char** argv)
{
	ROS_INFO("Starting Edge detector based VP");
	ros::init(argc, argv, "simplecanny");
	ROS_INFO("Starting");
	simplecanny cVP;
	ROS_INFO("running");
	cVP.run();

	return 0;
}