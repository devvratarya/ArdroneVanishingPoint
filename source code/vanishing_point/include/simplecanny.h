/* ---------------------------------------------------------------------------
** This software is in the public domain, furnished "as is", without technical
** support, and with no warranty, express or implied, as to its usefulness for
** any purpose.
**
** File : simplecanny.h
** header file for simplecanny.h. Contains function and variable ddeclarations
** Author: Devvrat Arya
** Date: 25 August 2014
** -------------------------------------------------------------------------*/


#include <ros/ros.h>
#include <stdio.h>
#include <iostream>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <std_srvs/Empty.h>
#include <string>


#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Twist.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <cvaux.h>
#include <math.h>
#include <cxcore.h>
#include <highgui.h>
#include <sstream>
#include <stdlib.h>
#include <fstream> 
#include <ardrone_autonomy/Navdata.h>


using namespace std;
using namespace cv;

//ardrone_autonomy::Navdata msg_in;

class simplecanny {
	
	
  	
public:
	enum droneState{NOT_STARTED, SEARCHING, APPROACHING, RECRUITING, LANDING};
    
    int imageHeight;
    int imageWidth;
	simplecanny(void);
	virtual ~simplecanny(void);

	void run(void);
	droneState state;
	double secs_start;
	
	ofstream myfile; 
  	ofstream myfile2;
  	ofstream myfile3;
	

protected:
	ros::NodeHandle node;

	
	image_transport::Subscriber vision_sub; //image subscriber
	ros::Publisher vanishP_pub;
	ros::Subscriber land_sub;
	ros::Subscriber takeoff_sub;
	

	std::string rosnamespace;

	//variables
	int counter;
  	int sizeX, sizeY;
  	int pre_maxi, pre_maxj;
  	int edgeThresh;
  	int lowThreshold;
  	//int const max_lowThreshold = 100;
  	int ratio;
  	int kernel_size;
  	
  	Point pre_vp;
  	std_msgs::String msg ;

	//ROS subscriber callbacks
	void landSubscriptionCallback(const std_msgs::Empty::ConstPtr& msg);
	void takeoffSubscriptionCallback(const std_msgs::Empty::ConstPtr& msg);
	void imageCb(const sensor_msgs::ImageConstPtr& msg);
	
	Point LineProcessing(vector<Vec4i> lines, cv::Mat result);
	Point searchVP(vector<Vec4i> lines, cv::Mat result);
	Point * intersectionPoint(cv::Vec4i l1, cv::Vec4i l2);
	
};
