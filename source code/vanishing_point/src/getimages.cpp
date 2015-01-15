#include <ros/ros.h>
#include <stdio.h>
#include <iostream>
#include "std_msgs/String.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>     //make sure to include the relevant headerfiles
#include <opencv2/highgui/highgui.hpp>
#include <opencv/cv.h>
#include <opencv/highgui.h>
//#include <cv_bridge/CvBridge.h>
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


static const std::string OPENCV_WINDOW = "Image window";
 
/*here is a simple program which demonstrates the use of ros and opencv to do image manipulations on video streams given out as image topics from the monocular vision
of robots,here the device used is a copyardrone(quad-rotor).*/
 
using namespace std;
using namespace cv;
namespace enc = sensor_msgs::image_encodings;
 
static const char WINDOW[] = "Image window";
static int counter = 1;
static int sizeX=640/33, sizeY=360/22;
int countP=0,maxi=0, maxj=0,pre_maxi=0, pre_maxj=0;
int edgeThresh = 1;
int lowThreshold;
int const max_lowThreshold = 100;
int ratio = 3;
int kernel_size = 3;
char* window_name = "Edge Map";
Point pre_vp;
 
class simplecanny
{
  ros::NodeHandle nh_;
  ros::NodeHandle n;
  ros::Publisher pub ;

  image_transport::ImageTransport it_;    
  image_transport::Subscriber image_sub_; //image subscriber 
  //image_transport::Publisher image_pub_; //image publisher(we subscribe to ardrone image_raw)
  std_msgs::String msg;

  cv::Mat src, src_gray;
  cv::Mat dst, detected_edges;
  //int imagecounter=1

 
    
public:
 simplecanny()
    : it_(nh_)
  {
 
   /* cv::namedWindow("Input");
    cvResizeWindow("Input", 320, 240);
    cv::namedWindow("Gaussian");
    cvResizeWindow("Gaussian", 320, 240);
    cv::namedWindow("grayscale");
    cvResizeWindow("grayscale", 320, 240);*/
    cv::namedWindow("PreProcessed");
    cvResizeWindow("PreProcessed", 320, 240);
    cv::namedWindow("detected_lines");
    cvResizeWindow("detected_lines", 320, 240);
    cv::namedWindow("HoughTransform");
    cvResizeWindow("HoughTransform", 320, 240);
    cv::namedWindow("GridImage");
    cvResizeWindow("GridImage", 320, 240);
    cv::namedWindow("vanishp");
    cvResizeWindow("vanishp", 320, 240);
 
    cvStartWindowThread();

    // image_sub_ = it_.subscribe("/ardrone/image_raw", 1, &simplecanny::imageCb, this);
    
    //image_sub_ = it_.subscribe("/test/image", 1, &simplecanny::imageCb, this);
    image_sub_ = it_.subscribe("/ardrone/front/image_raw", 1, &simplecanny::imageCb, this);
    //image_pub_= it_.advertise("/drone/Image",1);
 
      
  }
 
  ~simplecanny()
  {
    cv::destroyWindow(WINDOW);
  }
 
  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
      ROS_INFO("counter value = %d", counter);
      
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
      
      //cv::imshow("Input", cv_ptr->image);
      
      
      //vector<string> images;
      src = cv_ptr->image.clone();
      double secs_start =ros::Time::now().toSec();
      ofstream myfile; 
      ofstream myfile2; 
      //myfile2.open ("/home/devvrat/time.txt"); 
      //myfile.open ("/home/devvrat/VP.txt"); 
      //double timeelapsed[284];
      //for (int i=1;i<285;i++)
      //{
        //auto begin = chrono::high_resolution_clock::now(); 
        //clock_t time = clock();
        ostringstream convert;
        
        char buff[256] = "";
        sprintf(buff,"/home/devvrat/test_data5/image_%.5d.jpg",counter);
        convert << buff;
        //std::string address = to_string(buff);
        //images.push_back(convert.str());
      
      //std::cout << images[11] <<std::endl;

      //src = imread(convert.str(), CV_LOAD_IMAGE_COLOR);
      //dst.create( src.size(), src.type() );
      //ostringstream convert;
      //convert << counter;

      //std::string s = to_string(imagecounter);
      //std::string address = "/home/devvrat/test_data2/image_"+convert.str()+".jpg";
      //char buff[256] = "";
      //sprintf(buff,"/home/devvrat/test_data3/image_%.5d.jpg",counter);

      cv::imwrite(buff, src);
      counter++;
      cvWaitKey(2); 
      if(counter == 300)
      {
        //myfile.close();
        //myfile2.close();
        std::cout << "Done" <<std::endl;
        ros::shutdown();
      }
    //}
    //myfile.close(); 
 
}

//********* line Processing *********************************
//here we filter out, the unnecesery lines for the reailway detection Get the VP and the Yaw orientetion
Point LineProcessing(vector<Vec4i> lines, cv::Mat result)
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
    
    
    
    //float angle = 360;
    /*if (VP.x!= -1 && VP.y!=-1)
    {
      //The lines between the rail track showing the orientation of the treck
      ROS_INFO("deleted lines coun %d", deletedLines.size());
      angle=serachYaw(deletedLines, VP, result.cols, result.rows,result);      
    }*/
  // The returned point has the x coordinat of VP and the yaw angle
  //return Point2f(VP.x,angle);
      return VP;
}


// searching for VP between the filterd lines
Point searchVP(vector<Vec4i> lines, cv::Mat result)
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
      cP=crossPoint(lines[0],lines[1]);
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
          cP=crossPoint(lines[i],lines[j]);
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

      vpbox1 = cvPoint(maxi*sizeX,maxj*sizeY);
      vpbox2 = cvPoint((maxi+1)*sizeX,(maxj+1)*sizeY);
      /*//Point vanishP1,vanishP2;
      if(abs(pre_vp.x-vp.x) > 100 ||  abs(pre_vp.y-vp.y) > 100)
      {
        ROS_INFO("VP wrong counter %d",counter);
        vp = pre_vp;
        vpbox1 = cvPoint(pre_maxi*sizeX,pre_maxj*sizeY);
        vpbox2 = cvPoint((pre_maxi+1)*sizeX,(pre_maxj+1)*sizeY);
        //VP = pre_vp;
        //return;
      }
      else
      {
        pre_vp=vp;
        pre_maxj=maxj;
        pre_maxi=maxi;
        vpbox1 = cvPoint(maxi*sizeX,maxj*sizeY);
        vpbox2 = cvPoint((maxi+1)*sizeX,(maxj+1)*sizeY);
      }*/
      counter++;

      /*
      
      //Point vanishP1,vanishP2;
      if(abs(pre_vp.x-vp.x) > 25 ||  abs(pre_vp.y-vp.y) > 25)
      {
        ROS_INFO("VP wrong counter %d",counter);
        vp = pre_vp;
        vpbox1 = cvPoint(pre_maxi*sizeX,pre_maxj*sizeY);
        vpbox2 = cvPoint((pre_maxi+1)*sizeX,(pre_maxj+1)*sizeY);
        //VP = pre_vp;
        //return;
      }
      else
      {
        pre_vp=vp;
        pre_maxj=maxj;
        pre_maxi=maxi;
        vpbox1 = cvPoint(maxi*sizeX,maxj*sizeY);
        vpbox2 = cvPoint((maxi+1)*sizeX,(maxj+1)*sizeY);
      }
      counter++; */
      
      rectangle(result,vpbox1,vpbox2,red,2,8,0); 
      circle(result,cvPoint(vp.x,vp.y),3,Scalar(0,255,0),-1,8,0);
       
      ROS_INFO("got the points %d %d",vp.x,vp.y);              
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





//*******************Crossing points of 2 lines ********
Point * crossPoint(cv::Vec4i l1, cv::Vec4i l2)
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
    }else{
    Point cPt(-1,-1);
    * cP=cPt; 
    }
  return cP;
}


  float serachYaw(vector<Vec4i> lines, Point VP, int maxX, int maxY, cv::Mat result)
{
  ROS_INFO("in search yaw");
  Vec4i l;
  
  //vector<Vec4i>& lineRef = *lines;
  
  float theSum=0.0;
  int nrLines=0;
 // ROS_INFO("num of lines %d", lineRef.size());
  for (size_t i = 0; i < lines.size(); i++)
  { 
    //ROS_INFO("In loop");
    Point A =Point(0, maxY), B= Point(maxX, maxY);
    l = lines[i];
    if (pointInTriangel(A,B,VP,Point(l[0],l[1])) && pointInTriangel(A,B,VP,Point(l[2],l[3])) ) 
    { 
      //ROS_INFO("in if");
      theSum=theSum+(atan2(l[1]-l[3],l[2]-l[0])*180/PI +PI);
      nrLines=nrLines+1;
      line( result, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0,0,0),1);       
    }
  }
  if (nrLines==0) return 0.0;
  return (theSum/nrLines);
}
//**


// like VP bottom corners of imgage.
bool pointInTriangel(Point A, Point B, Point C, Point P )
{
  float v0x=C.x-A.x, v0y=C.y-A.y, v1x=B.x-A.x, v1y=B.y-A.y, v2x=P.x-A.x, v2y=P.y-A.y;
  float dot00 =v0x*v0x+v0y*v0y;
  float dot01 =v0x*v1x+v0y*v1y;
  float dot02 =v0x*v2x+v0y*v2y;
  float dot11 =v1x*v1x+v1y*v1y;
  float dot12 =v1x*v2x+v1y*v2y;
  float invDenom = 1/(dot00 * dot11 - dot01 * dot01);
  float u = (dot11 * dot02 - dot01 * dot12) * invDenom;
  float v = (dot00 * dot12 - dot01 * dot02) * invDenom;
  if (u>=0 && v>=0 && u+v<=1)
    return true;
  else
    return false;
}

  void getCannyImage(const cv::Mat& inputImage, cv::Mat& outputImage)
  {
    // Get a gray image - converting a color image in to gray image
    cv::Mat grayImage, detected_edges;
    //cv::cvtColor(inputImage, grayImage, CV_RGB2GRAY);

    //getting image edge using canny edge detection algo
   // double threshold1 = 20;
   // double threshold2 = 50;
   // int apertureSize = 3;

      /*canny devvratr*/
     /*int edgeThresh = 1;
      int lowThreshold;
      int const max_lowThreshold = 100;
      int ratio = 3;
       int kernel_size = 3;
      char* window_name = "Edge Map";
      cv::namedWindow( window_name, CV_WINDOW_AUTOSIZE );
      cv::cvtColor(inputImage, grayImage, CV_RGB2GRAY);*/
      //cv::createTrackbar( "Min Threshold:", window_name, &lowThreshold, max_lowThreshold, CannyThreshold );
     
    // The smallest of threshold1 and threshold2 is used for edge linking,
    // the largest - to find initial segments of strong edges. Play around
    // with these numbers to get desired result, and/or pre-process the
    // image, e.g. clean up, sharpen, blur).
    //cv::Canny(outputImage, outputImage, threshold1, threshold2, apertureSize);
  } 

  void CannyThreshold(int, void*)
{
  /// Reduce noise with a kernel 3x3
  cv::blur( src_gray, detected_edges, Size(3,3) );

  /// Canny detector
  cv::Canny( detected_edges, detected_edges, lowThreshold, lowThreshold*ratio, kernel_size );

  /// Using Canny's output as a mask, we display our result
  dst = Scalar::all(0);

  src.copyTo( dst, detected_edges);
  imshow( window_name, dst );
 }
};
 

 
 
int main(int argc, char** argv)
{
  ros::init(argc, argv, "simple_canny");
  //ROS_INFO("1");
  //simplecanny ic;
  ros::Time::init();
  ros::Rate loop_rate(10);
  //ROS_INFO("2");
  int count = 1;
  while (ros::ok())
  {
    ROS_INFO("count %d", count);
    simplecanny ic;
  ros::spin();
  loop_rate.sleep();
  count++;
}
ROS_INFO("4");
  return 0;
}