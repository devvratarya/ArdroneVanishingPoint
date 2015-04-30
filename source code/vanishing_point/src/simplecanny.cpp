/* ---------------------------------------------------------------------------
** This software is in the public domain, furnished "as is", without technical
** support, and with no warranty, express or implied, as to its usefulness for
** any purpose.
**
** File : simplecanny.cpp
** Calculated the vanishing point based on edge detection
**
** Author: Devvrat Arya
** Date: 25 August 2014
** -------------------------------------------------------------------------*/


#include <ros/ros.h>
#include <stdio.h>
#include <iostream>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>    
#include <opencv2/highgui/highgui.hpp>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <geometry_msgs/Point.h>
#include <ardrone_autonomy/Navdata.h>
#include <cvaux.h>
#include <math.h>
#include <cxcore.h>
#include <highgui.h>
#include <sstream>
#include <string>
#include <stdlib.h>
#include <fstream> 
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>

#include "simplecanny.h"

#define PI  3.14159265  
 

std_msgs::String out;

using namespace std;
using namespace cv;

simplecanny::simplecanny(void)
{
  
  state = NOT_STARTED;
  rosnamespace = node.getNamespace();

  land_sub = node.subscribe(rosnamespace+"/ardrone/land", 1, &simplecanny::landSubscriptionCallback, this);
  takeoff_sub = node.subscribe(rosnamespace+"/ardrone/takeoff", 1, &simplecanny::takeoffSubscriptionCallback, this);
  vanishP_pub = node.advertise<geometry_msgs::Point>("/ardrone_vp/vanishing_point", 5);
  
  cv::namedWindow("vanishp");
  cvResizeWindow("vanishp", 320, 240);

  ROS_INFO("initialised LocateTarget");

  counter = 1;
  sizeX=640/33;
  sizeY=360/22;
  imageHeight = 360;
  imageWidth = 640;
  secs_start = ros::Time::now().toSec();
  pre_maxi=0; 
  pre_maxj=0;
  edgeThresh = 1;
  lowThreshold;
  //max_lowThreshold = 100;
  ratio = 3;
  kernel_size = 3;
  return;
}
simplecanny::~simplecanny(void)
{

}

// This method is called from the main file
void simplecanny::run(void)
{
  ros::Rate loop(50); //update at 50Hz

  ROS_INFO("Find Vanishing Point");
  
  while(state == NOT_STARTED)
  {
    ros::spinOnce();
    loop.sleep();
  }
  ROS_INFO("drone is now airborne");
  image_transport::ImageTransport it_(node); 
  
  vision_sub = it_.subscribe(rosnamespace+"/ardrone/front/image_raw", 1, &simplecanny::imageCb, this);
  ROS_INFO("started subscription to VP detection on topic %s", vision_sub.getTopic().c_str());
  secs_start = ros::Time::now().toSec();
  while(state != LANDING)
  {
    ros::spinOnce();
    loop.sleep();
  }
  vanishP_pub.shutdown();
  
  return;
}

void simplecanny::landSubscriptionCallback(const std_msgs::Empty::ConstPtr& msg)
{
  state = LANDING;
  return;
}

void simplecanny::takeoffSubscriptionCallback(const std_msgs::Empty::ConstPtr& msg)
{
  state = SEARCHING;
  return;
}

/*callback function when receieve image from the ardrone
* FUnction : imageCb(const sensor_msgs::ImageConstPtr& msg)
* Description: preprocesses the image and sends image to line processing for vp calculation 
* Return : geometry_msgs::Point
*/
void simplecanny::imageCb(const sensor_msgs::ImageConstPtr& msg)
{
  
  cv::Mat src, src_gray;
  cv::Mat dst, detected_edges;

  /////////implementation//////////////////////////
  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
  src = cv_ptr->image.clone();
  cv::cvtColor( src, src_gray, CV_BGR2GRAY );
  cv::blur( src_gray, detected_edges, Size(5,5) ); 
  lowThreshold = 10;  
  cv::Canny( detected_edges, detected_edges, lowThreshold, lowThreshold*ratio, kernel_size );
  vector<Vec4i> lines;
  HoughLinesP(detected_edges, lines, 1, CV_PI/180, 50, 20, 4 );
  for( size_t i = 0; i < lines.size(); i++ )
  {
    Vec4i l = lines[i];
    line( src, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0,0,255), 1, CV_AA);
    for (size_t j= i+1; j<lines.size(); j++)
    {
      Vec4i lnext = lines[j]; 
      Point * tmp =intersectionPoint(l,lnext);
      //circle(src,cvPoint(tmp->x, tmp->y),1,CV_RGB(0,255,0),-1);
    }
  }
  ROS_INFO("get vanishing point");
  //vector<Vec4i> * deletedLines;
  Point  VP = LineProcessing(lines, src);

  geometry_msgs::Point vanishP;
  if(VP.x == -1 && VP.y == -1)
  {
    vanishP.x = -1; vanishP.y = -1; vanishP.z = -1;
  }
  else
  {
    vanishP.x = VP.x*100/imageWidth; vanishP.y = VP.y*100/imageHeight; vanishP.z = 1; 
    //vanishP.x = 0; vanishP.y = 0; vanishP.z = 1; 
  }
  //circle(src,cvPoint(VP.x,VP.y),3,Scalar(0,255,0),-1,8,0);
  cv::imshow("vanishp", src);
  
  ROS_INFO("Vanishing POint %f %f %f",vanishP.x,vanishP.y, vanishP.z);    
  vanishP_pub.publish(vanishP);
  float x = vanishP.x;//.x - 320;
  myfile << x << endl; 
  double secs =ros::Time::now().toSec() - secs_start;
  myfile2 << secs << endl;
   
  cvWaitKey(2); 
  /////////implementation finish//////////////////////////
  //vision_sub.shutdown();
  return;
}


/* FUnction : LineProcessing(vector<Vec4i> lines, cv::Mat result)
* Description: This function processes the image to get the strainght lines 
*              and filter the unwanted lines(vertical and horizontal)
* Input : preprocessed image  
* Return : cv::Point
*/
Point simplecanny::LineProcessing(vector<Vec4i> lines, cv::Mat result)
{
  vector<Vec4i> deletedLines; 
  // Filtering of the lines regarding the angle, to find treacks
  size_t i = 0;
  while (i < lines.size())
    {
      Vec4i l=lines[i];   
      float the=atan2(l[1]-l[3],l[0]-l[2])*180/PI;
      // for simulation taken out from line filtering, maybe necessary in turns
      // || (abs(the)<110&&abs(the)>75) ||(abs(the)<285&&abs(the)>245)
      //ROS_INFO("filtering lines %f", the);      
      if ( abs(the)<10 || (abs(the)<188 && abs(the)>172)|| (abs(the)<110&&abs(the)>75) ||(abs(the)<285&&abs(the)>245))
      {
        lines.erase(lines.begin()+i);
        deletedLines.push_back (l);
        //ROS_INFO("line deleted");
      }
      else
      {      
        line( result, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(255,0,0), 1);      
        i++;  
      }
      if(i==1)
      {deletedLines.push_back (l);}
    }

    //ROS_INFO("no of deleted lines %d", deletedLines->size());
    // Vanishing Point detectio if not found return is (-1,-1)
    ROS_INFO("lines size %d", lines.size());
    Point VP = searchVP(lines, result);
    return VP;
}


/* FUnction : searchVP(vector<Vec4i> lines, cv::Mat result)
* Description:  search for VP in filterd lines
* Input : filtered lines, cv::Mat image to display result  
* Return : cv::Point
*/
Point simplecanny::searchVP(vector<Vec4i> lines, cv::Mat result)
{
  //int sizeX=result.cols/33, sizeY=result.rows/22;
  Point vp = Point(-1,-1);
  Point vpbox1;
  Point vpbox2;
  if (lines.size()>1 && sizeX>0 && sizeY>0)
  {
    int imgGrid[22][33];    // grid of crossing points              
    for (int i=0;i<22;i++) // initialization
    {
      for (int j=0;j<33;j++) imgGrid[i][j]=0;

    }
    Point * cP;
    // Filling up the grid with crossing points
    if (lines.size()==2)
    {
      cP=intersectionPoint(lines[0],lines[1]);
      if (cP->x>0.0 && cP->y>0.0 && cP->x<sizeX*22.0 && cP->y<sizeY*33.0)
      {
        imgGrid[int(cP->x/sizeX)][int(cP->y/sizeY)]++;
      }     
    }
    else
    {
      for (int i=0;i<lines.size()-1;i++)
      { 
        for (int j=i+1; j<lines.size(); j++)
        {
          cP=intersectionPoint(lines[i],lines[j]);
          if (cP->x>0.0 && cP->y>0.0 && cP->x<sizeX*22.0 && cP->y<sizeY*33.0)
          {
            //circle(result,cvPoint(int(cP->x),int(cP->y)),3,Scalar(220,0,0),-1,8,0);
            imgGrid[int(cP->x/sizeX)][int(cP->y/sizeY)]++;
          }
        }
      }
    }
    delete cP;
    // searching for the most dens aria of intersections
    int countP=0,maxi=0, maxj=0;
    //countP=0; maxi=0; maxj=0;
    for (int i=0;i<22;i++)
    {
      for (int j=0;j<33;j++)
      {
        if (countP<imgGrid[i][j])
        {
          countP=imgGrid[i][j];
          maxi=i;
          maxj=j;
        }
      }
    }
    if (countP>0) 
    {
      //vpbox1 = cvPoint(maxi*sizeX,maxj*sizeY);
      //Point vanishP2 = cvPoint((maxi+1)*sizeX,(maxj+1)*sizeY);
      vp = cvPoint((maxi+0.5)*sizeX,(maxj+0.5)*sizeY);
      CvScalar red= CV_RGB(220,0,0);                      
      //rectangle(result,vanishP1,vanishP2,red,2,8,0); 
      if(counter==1)
      { 
        pre_maxi = maxi;
        pre_vp = vp;
        pre_maxj = maxj;
      }

      vp.x = pre_vp.x/2 + vp.x/2;
      vp.y = pre_vp.y/2 + vp.y/2;
      
      vpbox1 = cvPoint(maxi*sizeX,maxj*sizeY);
      vpbox2 = cvPoint((maxi+1)*sizeX,(maxj+1)*sizeY);
      
      pre_vp = vp;
      counter++;
      
      rectangle(result,vpbox1,vpbox2,red,2,8,0); 
      circle(result,cvPoint(vp.x,vp.y),3,Scalar(0,255,0),-1,8,0);
       
      //ROS_INFO("got the points %d %d",vp.x,vp.y);              
    }
    else
    {
      vp = Point(-1,-1);
      printf("NO vanishing point on the img \n");
    }     
  }
  else
  {
    vp = Point(-1,-1);
    putText(result, "No vanishing point", cvPoint(result.rows/2,result.cols/2),FONT_HERSHEY_COMPLEX_SMALL, 0.5, cvScalar(200,200,250), 0.5, CV_AA);
    printf("NOT enough lines \n");
  }
  return vp;
}

/* FUnction : intersectionPoint(cv::Vec4i l1, cv::Vec4i l2)
* Description:  finds intersevction poin between two lines
* Input : cv::Vec4i line1, cv::Vec4i line2 
* Return : cv::Point * to the intersection point
*/
Point * simplecanny::intersectionPoint(cv::Vec4i l1, cv::Vec4i l2)
{
  Point* cP;
  cP= new Point;
  Point o1 (l1[0], l1[1]) ; // start point line1
  Point p1 (l1[2], l1[3]) ; // end point line 1
  Point o2 (l2[0], l2[1]) ; // start point line2
  Point p2 (l2[2], l2[3]) ; // end point line2
  Point x = o2 - o1;
  Point d1 = p1 - o1;
  Point d2 = p2 - o2;
  float cross = d1.x*d2.y - d1.y*d2.x;
  if (abs(cross) > 1e-8)
  {
      double t1 = (x.x * d2.y - x.y * d2.x)/cross;
        * cP= o1 + d1 * t1;
  }
  else
  {
    Point cPt(-1,-1);
    * cP=cPt; 
  }
  return cP;
}



