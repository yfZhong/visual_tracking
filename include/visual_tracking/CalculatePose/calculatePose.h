// by Yongfeng

#ifndef CALCULATE_POSE_H
#define  CALCULATE_POSE_H

#include <Eigen/Dense>
#include <vector>
#include <iostream>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/PoseStamped.h>
#include <visual_tracking/Parameters/camera_parameters.h>
#include <visual_tracking/Projection/ForwardProjection.h>


using namespace std;
using namespace  vision;


namespace vision
      {   
	
	class PoseCalculator
	{
	  
	
	public:
	   PoseCalculator(){};
	   ~PoseCalculator(){};
	   
	   
	   Eigen::Matrix4d getRelativeTransform(std::vector<cv::Point2f> imgPts,  std::vector<cv::Point3f> worldPts);
           void calculatePose(std::vector<cv::Point2f> imgPts,  std::vector<cv::Point3f> worldPts, ForwardProjection & forw_Proj, geometry_msgs::PoseStamped& cameraPose,geometry_msgs::PoseStamped& robotPose);
	   
	  
	};



};

#endif