/* ---------------------------------------------------------------------------
** This software is in the public domain, furnished "as is", without technical
** support, and with no warranty, express or implied, as to its usefulness for
** any purpose.
**
** densityVP.cpp
** Calculated the vanishing point based on density clustering
**
** Author: Devvrat Arya
** Date: 17 November 2014
** -------------------------------------------------------------------------*/

#include <ros/ros.h>
#include <stdio.h>
#include <iostream>
#include "std_msgs/String.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <cv_bridge/cv_bridge.h>
#include <cvaux.h>
#include <math.h>
#include <cxcore.h>
#include <highgui.h>
#include <sstream>
#include <string>
#include <stdlib.h>
#include <fstream> 

#define PI  3.14159265  

 
using namespace std;
using namespace cv;
namespace enc = sensor_msgs::image_encodings;
 
static const char WINDOW[] = "Image window";
static int counter = 1;

int side_size = 0;
int prev_ysize = 0;
int prev_xsize = 0;
int pxx = 1; 
int pyy = 1;
int xx = 0;
int yy = 0;

 
class densityVP
{
  ros::NodeHandle nh_;
  ros::NodeHandle n;
  ros::Publisher pub ;

  image_transport::ImageTransport it_;    
  image_transport::Subscriber image_sub_;//image subscriber 
  image_transport::Publisher image_pub_; 
  std_msgs::String msg;

  cv::Mat src, src_gray;
  cv::Mat dst, detected_edges;
  cv::Point vp;
 
  public:
  densityVP()
    : it_(nh_)
  {
 
    cv::namedWindow("Sobel Operator");
    cvResizeWindow("Sobel Operator", 320, 240);
    cv::namedWindow("Integral image");
    cvResizeWindow("Integral image", 320, 240);
    cv::namedWindow("vanishp");
    cvResizeWindow("vanishp", 320, 240);
 
    cvStartWindowThread();

    image_sub_ = it_.subscribe("/test/image", 1, &densityVP::imageCb, this);
    //image_sub_ = it_.subscribe("/ardrone/front/image_raw", 1, &densityVP::imageCb, this);
    image_pub_= it_.advertise("/drone/Image",1);
  }
 
  ~densityVP()
  {
    cv::destroyWindow(WINDOW);
  }
 
  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    if(counter==1)
    {
      ROS_INFO("<--------------------------------------->");
      ROS_INFO("counter = %d", counter);
      counter++;
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
     
      vector<string> images;

      for (int i=1;i<285;i++)
      {
        clock_t time = clock();
        ostringstream convert;
        
        char buff[256] = "";
        sprintf(buff,"/home/devvrat/test_data3/image_%.5d.jpg",i);
        convert << buff;
        //std::string address = to_string(buff);
        images.push_back(convert.str());
      
        //std::cout << images[11] <<std::endl;

        src = imread(convert.str(), CV_LOAD_IMAGE_COLOR);
        dst.create( src.size(), src.type() );
        
        cv::Mat SImg,IImg;
        cv::cvtColor( src, dst, CV_BGR2GRAY );

        cv::Mat grad_x, grad_y;
        cv::Mat abs_grad_x, abs_grad_y;

        /// Gradient X
        cv::Sobel( dst, grad_x, CV_16S, 1, 0, 3, 1, 0, BORDER_DEFAULT );
        cv::convertScaleAbs( grad_x, abs_grad_x );

        /// Gradient Y
        cv::Sobel( dst, grad_y, CV_16S, 0, 1, 3, 1, 0, BORDER_DEFAULT );
        cv::convertScaleAbs( grad_y, abs_grad_y );

        /// Total Gradient (approximate)
        cv::addWeighted( abs_grad_x, 0.5, abs_grad_y, 0.5, 0, SImg );
       
        //cv::imshow("Sobel Operator", SImg);

        cv::Size s = SImg.size();

        if(s.width < s.height)
        {
          side_size = s.width -1;
        }
        else
        {
          
          side_size = s.height -1;
        }
        prev_ysize = s.height;
        prev_xsize = s.width;
        pxx = 1; pyy = 1;

        cv::imshow("Sobel Operator", SImg);
        cv::integral(SImg,IImg);

        int density = 0;
        
        //ROS_INFO("source size h: %d , w: %d , iimg size h: %d , w: %d",SImg.rows, SImg.cols, IImg.rows, IImg.cols);
        //cv::imshow("density squares", IImg);
        for (int k = 0; k < 26; k++)
        {
          
          int max_density = 0;
          int y_limit = pyy+prev_ysize-1-side_size;
          int x_limit = pxx+prev_xsize-1-side_size;

         // ROS_INFO("square = %d", k);
         // ROS_INFO("y limit is  = %d", y_limit);d
         // ROS_INFO("x limit is  = %d", x_limit);
          for(int y=pyy;y<=y_limit;y++)
          {
            for(int x=pxx;x<=x_limit;x++)
            {
              //density = mean(mean(eI1(y:y+side_size, x:x+side_size)));
              //ROS_INFO("Y AND X : %d %d", y,x);
              int rb=0, lt=0, lb=0, rt=0;
              rb = IImg.at<int>(y+side_size,x+side_size);
              if (x>1 && y>1)
                  lt = IImg.at<int>(y-1,x-1);
              else
                  lt = 0;
              if (x>1)
                  lb = IImg.at<int>(y+side_size,x-1);
              else
                  lb = 0;
              if (y>1)
                  rt = IImg.at<int>(y-1,x+side_size);
              else
                  rt = 0;
              //ROS_INFO("rb = %f,lt = %f,lb = %f,rt = %f", rb,lt,lb,rt);
              density = rb + lt - rt - lb;
              
              if (density > max_density)
              {
                max_density = density;
                xx = x;
                yy = y;
                //ROS_INFO("xx: %d,yy: %d",xx,yy);
              }    
            } //end x
          }//end y
          
          //std::cout<<"side size : " <<side_size <<endl;
          //std::cout<<"p1 x : "<<xx<<" y : "<<xx+side_size<<" | p2 x : "<<yy<<" y : " <<yy<<endl;
          line( src, Point(xx,yy), Point(xx+side_size, yy), Scalar(0,255,0), 1, CV_AA);
          //std::cout<<"p1 x : "<<xx<<" y : "<<xx+side_size<<" | p2 x : "<<yy+side_size<<" y : " <<yy+side_size<<endl;
          line( src, Point(xx, yy+side_size), Point(xx+side_size, yy+side_size), Scalar(0,255,0), 1, CV_AA);
          //std::cout<<"p1 x : "<<xx<<" y : "<<xx<<" | p2 x : "<<yy<<" y : " <<yy+side_size<<endl;
          line( src, Point(xx, yy), Point(xx, yy+side_size), Scalar(0,255,0), 1, CV_AA);
          //std::cout<<"p1 x : "<<xx+side_size<<" y : "<<xx+side_size<<" | p2 x : "<<yy<<" y : " <<yy+side_size<<endl;
          line( src, Point(xx+side_size, yy), Point(xx+side_size, yy+side_size), Scalar(0,255,0), 1, CV_AA);
          
          prev_ysize = side_size;
          prev_xsize = side_size;
          pxx = xx; pyy = yy;
          side_size = floor(0.95*side_size);
          
          vp.x = xx+(side_size/2)+1;
          vp.y = yy+(side_size/2)+1;

          circle(src,cvPoint(vp.x,vp.y),3,Scalar(0,0,220),-1,8,0);
          ostringstream convert1;
          convert1 << k;
          std::string text = convert1.str();
          putText(src, text, cvPoint(vp.x-2,vp.y-3),FONT_HERSHEY_COMPLEX_SMALL, 0.5, cvScalar(200,200,250), 0.5, CV_AA);
        }//end k for loop

        vp.x = xx+(side_size/2)+1;
        vp.y = yy+(side_size/2)+1;

        circle(src,cvPoint(vp.x,vp.y),3,Scalar(220,0,0),-1,8,0);
        
        ostringstream convert2;
          
        char buff2[256] = "";
        sprintf(buff2,"x:%d,y:%d",vp.x,vp.y);
        convert2 << buff2;
        ROS_INFO("x:%d,y:%d",vp.x,vp.y);

        putText(src, "VP", cvPoint(vp.x+5,vp.y-4),FONT_HERSHEY_COMPLEX_SMALL, 0.5, cvScalar(220,0,0), 0.5, CV_AA);
        cv::imshow("vanishp", src);  
        time = clock() - time;
        double ms = double(time) / CLOCKS_PER_SEC * 1000;
        timeelapsed[i] = ms;
        myfile << timeelapsed[i] << endl; 
        cout << "Time elapsed: " << ms << endl; 
        cvWaitKey(2); 
         if(i == 284)
        {
          std::cout << "Done" <<std::endl;
          ros::shutdown();
        }
      }
      myfile.close(); 
    }
  }
};
 

 
 
int main(int argc, char** argv)
{
  ros::init(argc, argv, "density_vanishing_point");
  //densityVP ic;
  ros::Time::init();
  ros::Rate loop_rate(10);
  int count = 1;
  while (ros::ok())
  {
    //ROS_INFO("count %d", count);
    densityVP dvp;
  ros::spin();
  loop_rate.sleep();
  count++;
  }
  return 0;
}