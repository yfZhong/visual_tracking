//Created on: April 12, 2014
//Authors: Yongfeng
#include <iostream>
#include <ros/ros.h>
#include <visual_tracking/Parameters/Parameters.h>
#include <visual_tracking/camera/MatPublisher.h>
#include <visual_tracking/camera/ICamera.h>
#include <visual_tracking/camera/Camera.h>
#include <visual_tracking/camera/CameraDummy.h>
#include <visual_tracking/rgb_yuyv/rgb2yuyv.h>

// #include "MatPublisher.h"
// #include "ICamera.h"
// #include "Camera.h"
// #include "CameraDummy.h"
// #include <visual_tracking/rgb_yuyv/rgb2yuyv.h"
using namespace cv;
using namespace std;

int main( int argc, char** argv ) { 
  
  ros::init(argc, argv, "cameraInput");
  params.Init();
  
  ICamera *cam;
  bool dummy = 0;
    if (dummy == true)
    {
	    cam = new CameraDummy();
    }
    else
    { 
	    cam = new Camera();
    }
    
       
    if (false == cam->InitCameraDevice(true))
    {
	    ROS_ERROR("Failed to initialize Camera!");
	    return false;
    }
    ROS_INFO("Camera Started");
    

    
    
  ros::Rate loop_rate(20);
   while (ros::ok())
    {
      

      
	  double confidence = cam->TakeCapture();
	  
	  if (confidence < 0.75 && !cam->IsReady())
	  {
		  return false;
	  }
	  
	  
	    
	  ros::spinOnce();
	  // sleep, to keep 50ms delay
	  loop_rate.sleep();
    }
  
  
  
  
  delete cam;
   return 0;
  
}




// void handleHeadingData( const robotcontrol::RobotHeadingConstPtr& msg)
// 	{
// 		robotcontrol::RobotHeading headingData=*msg;
// 	}