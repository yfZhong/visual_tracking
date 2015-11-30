#ifndef PARTICLE_FILTER_H
#define PARTICLE_FILTER_H


#include <iostream>
#include <vector>
#include <math.h>
#include <Eigen/Dense>
#include <Eigen/Eigenvalues>
#include <iomanip>

#include <stdio.h>      /* printf, scanf, puts, NULL */
#include <stdlib.h>     /* srand, rand */
#include <time.h>       /* time */


#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>


#include <visual_tracking/Parameters/camera_parameters.h>
#include <visual_tracking/Parameters/globaldefinitions.h>
#include <visual_tracking/Projection/ForwardProjection.h>
#include <visual_tracking/Projection/BackwardProjection.h>
#include <visual_tracking/IcpPointMatcher/pointMatcher.h>
#include <visual_tracking/CalculatePose/calculatePose.h>
#include <visual_tracking/Projection/DistortionModel.h>
// #include <visual_tracking/Tools/tools.h"
#include <visual_tracking/Parameters/Parameters.h>
#include <visual_tracking/LineMatcher/findLines.h>
#include <visual_tracking/LineMatcher/lineMatcher.h>
#include <visual_tracking/Tools/m_Math.h>
#include <robotcontrol/RobotHeading.h>

#include <gsl/gsl_rng.h>
#include <gsl/gsl_randist.h>


#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "PclClustering.h"



using namespace std;
using namespace cv;

namespace vision
{        
    

    class ParticleFilter{
      public:
	ParticleFilter();
	~ParticleFilter();
	
	void robotPoseInit();
	void robotPoseUpdate(geometry_msgs::PoseArray &particles);
	void setdetectedLines(std::vector< Line > LinesOnImg_ ,float Conf);
	void setCurTime(ros::Time t);
	

	void InitSampling();// sampling particleNum random samples
	geometry_msgs::Pose SamplingRandomPose();// sampling a single random sample
	void updateWeights(); // updating weights according to line-matching performance 
	void ReSampling();// resample particleNum samples(randomSampleNum random samples  and  (particleNum-randomSampleNum) smaples sampling according to weights )
	void systematic_resampling(int num, std::vector<int>& Idxlist);// sampling (particleNum-randomSampleNum) number of samples using Stochastic universal sampling 
	void predict();
	void run();
	
	
	std::vector< Line > LinesOnImg;//output of findLines
	float MeasConf;
	ros::Time cur_time_stamp;

	geometry_msgs::PoseArray particles_ego_rot;
	geometry_msgs::PoseArray particles_camera;
	geometry_msgs::PoseArray particles_tmp;
	vector<geometry_msgs::Pose> particlesKeepUnchanged;
	vector<geometry_msgs::Pose> particlesForUpdate;
	int particleNum;
	int particleNumKeepUnchanged;
	int particleNumForUpdate;
	int randomSampleNum;
	
	
	
        std::vector<double> weights;
	
	 
	LineMatcher lineMatcher;
	std::vector<int> resampleIdx;//output of function systematic_resampling()
	
	geometry_msgs::PoseStamped robotPose;
	geometry_msgs::PoseStamped cameraPose;
	
	ros::NodeHandle nodeHandle;
	robotcontrol::RobotHeading headingData;
	ros::Subscriber heading_sub_robotstate;
	void handleHeadingData( const robotcontrol::RobotHeadingConstPtr& msg) //in radian
	{
		headingData=*msg;
	}
	double headingOffset;
	double getHeading();
	
	
	gsl_rng * m_rng;// random value generator
	gsl_rng* allocate_rng();
        int SeqNum;
        bool reSamplingAccordingWeights;
	
	
      private:
	
	double random(double,double);
	double randomab(double a,double b);
	float random(float start,float end);
	

    };


}

#endif