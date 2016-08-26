// by Yongfeng

#ifndef HILLCLIMBING_H
#define  HILLCLIMBING_H

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
using namespace Eigen;

namespace vision
      {   
	
	class HillClimbing
	{
	  
	
	public:
	   HillClimbing();
	   ~HillClimbing();
	   

	  DataAssociation AssociateData;
	  geometry_msgs::Pose currentPose;
	  float currentError;
	  float currentAvgError;
	  float preError;
	  float preAvgError;
	  
	  
	  float fieldA;
	  float fieldB;
	  void getFieldInfo( FieldInfo &_fieldInfo );
	  geometry_msgs::PoseArray hypothesisArray;
	  vector<float> hypothesisWeights;
	  	   
	  double fx , fy, cx, cy; 
	  int siX, siY, H, W;
	  
	  PoseCalculator EPnP_poseUpdate;
	  
	  
	  
	  
	  
	  //hill climbing method
	  void hillClimbingLoop(geometry_msgs::Pose &pose,FrameGrabber & CamFrm ,FindNodes &NodeFinder);
	  void robotPoseUpdate(geometry_msgs::Pose &p,FrameGrabber & CamFrm,FindNodes &NodeFinder, float &error);
	  bool hypothesisEvalueate(float conf, FrameGrabber & CamFrm,FindNodes &NodeFinder);
	  
	  
	  
	  
	  
	  //EPnP method
	  void EPnPLoop(geometry_msgs::Pose &pose,FrameGrabber & CamFrm ,FindNodes &NodeFinder);
	  bool EPnP( DataAssociation &AssociateData,FrameGrabber & CamFrm ,FindNodes &NodeFinder,geometry_msgs::Pose &p);
	  bool hypothesisEvalueate2(float conf, FrameGrabber & CamFrm,FindNodes &NodeFinder);
	   
	  //covariance for KF
	  void calculateCov(geometry_msgs::Pose &pose, FrameGrabber & CamFrm, FindNodes &NodeFinder);
	   

	  
	  float doModelMatching(geometry_msgs::Pose &p,FrameGrabber & CamFrm,FindNodes &NodeFinder);
	  void correctHeading( geometry_msgs::Pose &pose );
	  float getDistance( geometry_msgs::Pose &pose1, geometry_msgs::Pose &pose2 );
	  float getYawDiff( geometry_msgs::Pose &pose1, geometry_msgs::Pose &pose2 );
	  void MovingStepInit();
	  void getNeighbors(geometry_msgs::Pose p,int idx,  vector< geometry_msgs::Pose >& poseArray);
	  geometry_msgs::Pose SamplingPoseAround(geometry_msgs::Pose center, float conf);
	   
	   
	    
	  //by Hafez
	  ros::NodeHandle nodeHandle;
	  robotcontrol::RobotHeading headingData;
	  ros::Subscriber heading_sub_robotstate;
	  void handleHeadingData( const robotcontrol::RobotHeadingConstPtr& msg) //in radian
	  {
		  headingData=*msg;
	  }
	  double headingOffset;
	  double getHeading();
	 
	  
	  double moving_step[6];
	  double min_moving_step[6];

	  double cov[6];
	  
	  double first_10_error[10];
	  double first_10_error_avg;
	  int n, m;
	  
	  double conf_error_lost;
	  int lost_frame_num;
	  
	  int SeqNum;
	  
	  gsl_rng * m_rng;// random value generator
	  gsl_rng* allocate_rng();
	  double random(double start,double end);
	  float random(float start,float end);
	  
	  
	  int count;
	  float confidence;
	  
	  
	  float robot_rot_height;
	  
	  
	  
	};



}

#endif