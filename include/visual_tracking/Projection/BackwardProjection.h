#ifndef PROJECTIMG2WORLD_H
#define PROJECTIMG2WORLD_H


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
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseArray.h>


#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>



#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>

#include <visualization_msgs/MarkerArray.h>
#include <visual_tracking/Parameters/camera_parameters.h>
#include <visual_tracking/Parameters/globaldefinitions.h>
#include "DistortionModel.h"
// #include "tools.h"

using namespace std;
using namespace cv;

namespace vision
{        
    

    class BackwardProjection{
      public:

	BackwardProjection();
	void robotPoseInit();
	void setRobotPose(geometry_msgs::Pose p);
	void setCurTime(ros::Time t);
	void setTfEgorot2Cam();
	void onInit(ros::Time t );
	
	void undistortP(cv::Point pIn, cv::Point & pOut);
	void projectImgPoint2WorldCord(cv::Point pIn, cv::Point3f &pOut);
	
	vector<cv::Point3f> p_world;
	
	
	
	
        void projectImgPoint2WorldCord_vec(vector<cv::Point> pIn, vector<cv::Point3f> &pOut);
	bool projectWordPoint2Img(cv::Point3f pIn, cv::Point & pOut);
	
	void TfRobotPose2CamPose(geometry_msgs::PoseArray& robotPoseArr, geometry_msgs::PoseArray &cameraPoseArr);
	void TfRobotPose2CamPose(geometry_msgs::PoseStamped &robotPose, geometry_msgs::PoseStamped &cameraPose);
// 	void mainLoop();
	float timeToshift;
	tf::Transform tf_Egorot2Cam;
	tf::TransformListener listener;

      protected:
	ros::NodeHandle node;
	
	double fx , fy, cx, cy; 
	
        int W, H, siX, siY, offsetx, offsety;

	tf::Transform tf_World2Cam;
	tf::Matrix3x3  rotation_World2Cam;
	tf::Vector3  translation_World2Cam;
	
	tf::Matrix3x3  rotation_Cam2World;
	tf::Vector3  translation_Cam2World;
      
	Mat cameraMatrix, distCoeffs;
	
	ros::Time cur_time_stamp;
	
	visualization_msgs::MarkerArray marker;
         
	
	geometry_msgs::Pose rbPose;
    };


}

#endif