#ifndef SINGLE_PARTICLE_H
#define SINGLE_PARTICLE_H


#include <iostream>
#include <vector>
#include <math.h>
#include <Eigen/Dense>

#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>


#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include <geometry_msgs/Pose.h>

#include <visual_tracking/Parameters/camera_parameters.h>
#include <visual_tracking/Parameters/globaldefinitions.h>
#include <visual_tracking/Projection/ForwardProjection.h>
#include <visual_tracking/Projection/BackwardProjection.h>
#include <visual_tracking/IcpPointMatcher/pointMatcher.h>
#include <visual_tracking/CalculatePose/calculatePose.h>
#include <visual_tracking/Projection/DistortionModel.h>
// #include <visual_tracking/Tools/tools.h"

using namespace std;
using namespace cv;

namespace vision
{        
    
    struct Model{
	ForwardProjection forw_Proj;
  //       BackwardProjection backw_Proj;
	PointMatcher pointMatcher;
	PoseCalculator poseCalculator;
    };

    class SingleParticle{
      public:
	
	SingleParticle();
	
	void run(std::vector<cv::Point> &detectedPoints, geometry_msgs::Pose &p, Model& model);
	void cameraPoseInit();
	
	
	geometry_msgs::PoseStamped getCameraPose();
	std::vector<cv::Point> getUndistPoints();
	std::vector<cv::Point2f> getImgPts();
	std::vector<cv::Point3f> getWorldPts();
	int getInlierNum();
      protected:
	 geometry_msgs::PoseStamped cameraPose;
	 std::vector<cv::Point> undistPoints;
	 std::vector<cv::Point2f> imgPts;// 2d image points
	 std::vector<cv::Point3f> worldPts;//3D world points
	 

	 
    };


}

#endif