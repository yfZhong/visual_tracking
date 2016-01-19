#ifndef KALMANFILTER_H
#define KALMANFILTER_H

#include <ros/ros.h>
#include <iostream>
#include <stdio.h>
#include <vector>
#include <string>
#include <iomanip>
#include <sstream>
#include <fstream>      // std::fstream
#include <locale>
#include <math.h>
#include <Eigen/Dense>
#include <queue>

#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>


#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>

#include <visual_tracking/Parameters/Parameters.h>
#include <robotcontrol/RobotHeading.h>
// #include <robotcontrol/State.h>
#include <gait_msgs/GaitOdom.h>
#include <visual_tracking/Tools/m_Math.h>


const double PI = 3.141592653589793;
const double TAU = 6.283185307179587;

using namespace std;
using namespace Eigen;

namespace vision
{ 
    class M_KalmanFilter{
      public:
	
	M_KalmanFilter();
	
	~M_KalmanFilter(){};
	
	void correction(const geometry_msgs::PoseStamped& msg , float conf);
	void prediction();
	void setCurTime(ros::Time t);
	void getMeasurementCov(double (&cov)[6]);
	void robotPoseInit();
	void mainLoop(const geometry_msgs::PoseStamped& msg , float conf);
	void getRobotPose(geometry_msgs::PoseStamped & _robotPoseS ){_robotPoseS = robotPoseS; }
	
// 	void updateTransform();
	
	double clampRotation(double rotation);

	
	Eigen:: Matrix<float, 6, 6> estimate_cov;
	Eigen:: Matrix<float, 6, 1> estimate_pose;
	geometry_msgs::PoseWithCovarianceStamped estimate_pose_cov;
	Eigen:: Matrix<float, 6, 1> predict_pose;
	double speed_x, speed_y, speed_yaw;
	
	bool useMotionOdom;
	
	geometry_msgs::PoseStamped  robotPoseS;
	
	
	

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
	
	
	uint32_t lastOdomID;
	ros::Subscriber odom_sub;
	Point3d lastOdom;
	Point3d globalPos;
	void prediction2(const gait_msgs::GaitOdomConstPtr & msg);
	cv::Point2f RotateAroundPoint(cv::Point2f pQuery, double alpha);
        void RotateAroundPoint(Point2d pQuery, double alpha, Point2d &res);
        void RotateCoordinateAxis(double alpha, Point2d p, Point2d &res);
	cv::Point2f RotateCoordinateAxis(double alpha, cv::Point2f p);

      protected:
	
	ros::Time cur_time_stamp;
	ros::Time pre_meas_time; 
	int init;
	float measurementCov[6];
	
	int unexpectPose_n;
	
	vector<gait_msgs::GaitOdomConstPtr> msg_array;
	
// 	double sensorTimeout_;
	
    };

};

#endif
