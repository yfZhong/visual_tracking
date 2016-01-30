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
	   
	  void MovingStepInit();
	   
	  void robotPoseUpdate(geometry_msgs::Pose &p,FrameGrabber & CamFrm, PointMatcher &pointMatcher,FindNodes &NodeFinder, double &error);
	  
	  void getNeighbors(geometry_msgs::Pose p,int idx,  vector< geometry_msgs::Pose >& poseArray);
	  
	  void calculateError(geometry_msgs::Pose &p,FrameGrabber & CamFrm, double(&error)[2] , PointMatcher &pointMatcher,FindNodes &NodeFinderC);
	  
	  void mainLoop(geometry_msgs::Pose &p,FrameGrabber & CamFrm, PointMatcher &pointMatcher,FindNodes &NodeFinder);
	  
	  void calculateCov(geometry_msgs::Pose &p, FrameGrabber & CamFrm, PointMatcher &pointMatcher,FindNodes &NodeFinder);
	  
	  void calculateErrorForCov(geometry_msgs::Pose &p,FrameGrabber & CamFrm, double &error, double& K, PointMatcher &pointMatcher,FindNodes &NodeFinder);
	  
	  
	  
	  
	  
	  DataAssociation AssociateData;
	  float calculateError(geometry_msgs::Pose &p,FrameGrabber & CamFrm,FindNodes &NodeFinder);
	  float calculateAvgError(geometry_msgs::Pose &p,FrameGrabber & CamFrm,FindNodes &NodeFinder);
	  float calculateErrorForCov(geometry_msgs::Pose &p,FrameGrabber & CamFrm, FindNodes &NodeFinder, float &K );// K ----num of correspondence
	  void calculateCov(geometry_msgs::Pose &pose, FrameGrabber & CamFrm, FindNodes &NodeFinder);
	  void robotPoseUpdate(geometry_msgs::Pose &p,FrameGrabber & CamFrm,FindNodes &NodeFinder, float &error);
	  float getConfidence(geometry_msgs::Pose &p,FrameGrabber & CamFrm,FindNodes &NodeFinder);
	  
	   float doModelMatching(geometry_msgs::Pose &p,FrameGrabber & CamFrm,FindNodes &NodeFinder);
	   bool Least_Square( DataAssociation &AssociateData,FrameGrabber & CamFrm ,FindNodes &NodeFinder,geometry_msgs::Pose &p);
	   void Least_Square2(geometry_msgs::Pose &pose,FrameGrabber & CamFrm ,FindNodes &NodeFinder);
	   void IterativeLeastSquare(geometry_msgs::Pose &pose,FrameGrabber & CamFrm ,FindNodes &NodeFinder);
	   bool hypothesisEvalueate2(float conf, FrameGrabber & CamFrm,FindNodes &NodeFinder);
	   double fx , fy, cx, cy; 
	   int siX, siY, H, W;
	   PoseCalculator Least_Square_poseUpdate;
	  
	  
	  void mainLoop(geometry_msgs::Pose &pose,FrameGrabber & CamFrm ,FindNodes &NodeFinder);
	  
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
	  geometry_msgs::Pose SamplingPoseAround(geometry_msgs::Pose center, float conf);
	  bool hypothesisEvalueate(float conf, FrameGrabber & CamFrm,FindNodes &NodeFinder);
	  void correctHeading( geometry_msgs::Pose &pose );
	   float getDistance( geometry_msgs::Pose &pose1, geometry_msgs::Pose &pose2 );
	   float getYawDiff( geometry_msgs::Pose &pose1, geometry_msgs::Pose &pose2 );
	  
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
	 
	  
	  
// 	  m_Math::RadianAngleDiff(yaw, getHeading()) 
// 	  RadianAngleDiff(double yaw1, double yaw2)
	 
	  
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
	  
	  
	  
	};



}

#endif