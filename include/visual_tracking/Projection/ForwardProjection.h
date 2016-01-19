#ifndef PROJECTWORLD2IMG_H
#define PROJECTWORLD2IMG_H


#include <iostream>
#include <vector>
#include <math.h>
#include <Eigen/Dense>
#include <utility>

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



#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>

#include <visualization_msgs/MarkerArray.h>
#include <visual_tracking/Parameters/camera_parameters.h>
#include <visual_tracking/Parameters/globaldefinitions.h>
#include "DistortionModel.h"
// #include <visual_tracking/frameGrabber.h"
#include <visual_tracking/Tools/m_Math.h>


using namespace std;
using namespace cv;
using namespace vision;

namespace vision
{        
    

    class ForwardProjection{ 
      public:
		
	ForwardProjection();
	
	void onInit(ros::Time t );
	void robotPoseInit();
	void setRobotPose(geometry_msgs::Pose p);
	void setCurTime(ros::Time t);
	void setTfEgorot2Cam();
	
        void generateFieldLines();
	void generateFieldPoints();
	void tfFieldPoints2CamCord();
	void projectFieldPoints2Image();
	void generateFieldLineInImage();
	bool projectWordPoint2Img(cv::Point3f pIn, cv::Point & pOut);
        void projectImgPoint2WorldCord(cv::Point p, cv::Point3f& pOut);
	void TfRobotPose2CamPose(geometry_msgs::PoseStamped &robotPose, geometry_msgs::PoseStamped &cameraPose);
	void TfCamPose2RobotPose(geometry_msgs::PoseStamped &cameraPose, geometry_msgs::PoseStamped &robotPose);
	void mainLoop();
	
	
	void GetModelLineComps();
	

	ModelLine_Buffer  * m_ModelLine_Buffer;
	
	
	
	FieldInfo fieldInfo;
	std::vector< Line_w > Field_Lines;
	std::vector< Line > Field_Lines_Img;
	
	tf::TransformListener listener;
        tf::Transform tf_Egorot2Cam;
	float timeToshift;
	
// 	ros::Publisher points_pub_cam;
	
	std::vector<pair<tf::Vector3, int > > PointsInWorldCord;
	std::vector<pair<tf::Vector3, int > > PointsInCamCord;

	std::vector<cv::Point> ModelPointsInImg;
	std::vector<pair<cv::Point, int > >ModelPointsInImgWithId;
// 	std::vector<cv::Point3f> ModelPointsInWorldCord;
	
	
	
	~ForwardProjection(){    
		m_ModelLine_Buffer->clear();
		delete m_ModelLine_Buffer;
	  
	}
		 
	
      protected:
	ros::NodeHandle node;
	ros::Time cur_time_stamp;
	
	int seq;

	double fx , fy, cx, cy; 
        int W, H, siX, siY, offsetx, offsety;
	
	float SamplePointDist_WorldCord;

	tf::Transform tf_World2Cam;
	tf::Matrix3x3  rotation_World2Cam;
	tf::Vector3  translation_World2Cam;
	tf::Matrix3x3  rotation_Cam2World;
	tf::Vector3  translation_Cam2World;
	
	geometry_msgs::Pose rbPose;
    };


}

#endif