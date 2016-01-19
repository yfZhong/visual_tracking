// by Yongfeng

#ifndef LOST_TRACK_RECOVER_H
#define  LOST_TRACK_RECOVER_H

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
#include <visual_tracking/Parameters/globaldefinitions.h>
#include <visual_tracking/IcpPointMatcher/pointMatcher.h>
#include <visual_tracking/CalculatePose/calculatePose.h>
#include <visual_tracking/Projection/DistortionModel.h>

#include <visual_tracking/Parameters/Parameters.h>
#include <visual_tracking/LineMatcher/findLines.h>
#include <visual_tracking/LineMatcher/findNodes.h>
#include <visual_tracking/DataAssociation/dataAssociation.h>

#include <visual_tracking/LineMatcher/lineMatcher.h>
#include <visual_tracking/Tools/m_Math.h>
#include <robotcontrol/RobotHeading.h>
#include <visual_tracking/frameGrabber.h>

#include <gsl/gsl_rng.h>
#include <gsl/gsl_randist.h>

#include <visual_tracking/DataAssociation/dataAssociation.h>


using namespace std;
using namespace  vision;


namespace vision
      {   
	
	class LostTrackerRecover
	{
	  
	
	public:
	   LostTrackerRecover();
	   ~LostTrackerRecover();
	   
	  geometry_msgs::Pose LostTrackerRecover::SamplingRandomPose();
	  
	  float fieldA;
	  float fieldB;
	  void getFieldInfo( FieldInfo &_fieldInfo );
	  geometry_msgs::Pose SamplingRandomPose();
	  geometry_msgs::Pose SamplingPoseAround(geometry_msgs::Pose center, float range);
	   
// 	  void robotPoseUpdate(geometry_msgs::Pose &p,FrameGrabber & CamFrm, PointMatcher &pointMatcher,FindNodes &NodeFinder, double &error);
	  
	  
	  
// 	  gsl_rng * m_rng;// random value generator
// 	  gsl_rng* allocate_rng();
	   
	  
	};



}

#endif