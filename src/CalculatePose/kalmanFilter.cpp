
// #include "kalmanFilter.h"
#include <visual_tracking/CalculatePose/kalmanFilter.h>
using namespace vision;


M_KalmanFilter::M_KalmanFilter()/*:listener(ros::Duration(3.0))*/{
      robotPoseInit();
     
      predict_pose.setZero();
      
      pre_meas_time = ros::Time::now();
      cur_time_stamp = ros::Time::now();
      init = 1;
      lastOdomID = 0;
      globalPos.x = 0;
      globalPos.y = 0;
      globalPos.z = 0;
      lastOdom.x = 0;
      lastOdom.y = 0;
      lastOdom.z = 0;
      heading_sub_robotstate = nodeHandle.subscribe("/robotmodel/robot_heading", 1, &M_KalmanFilter::handleHeadingData, this);
      odom_sub = nodeHandle.subscribe("/gait/odometry", 1, &M_KalmanFilter::dead_reckoning_callback, this);
      headingOffset = params.icp.HeadingOffset->get() ;

}
 
void M_KalmanFilter::robotPoseInit(){
  
   estimate_pose<<params.location.x->get(),params.location.y->get(),params.location.z->get(),
                 params.orientation.x->get(),params.orientation.y->get(),params.orientation.z->get();
  
}


//from Hafez
double M_KalmanFilter::getHeading()
{       headingOffset = params.particleFilter.HeadingOffset->get() ;
        return m_Math::CorrectAngleRadian360(headingData.heading + headingOffset );
}

	
	
//from Hafez	
void M_KalmanFilter::RotateAroundPoint(Point2d pQuery, double alpha, Point2d &res) {
	Point2d pCenter(0, 0);
	Point2d p = pQuery - pCenter;
	Point2d resRot;
	RotateCoordinateAxis(-alpha, p, resRot);
	res = resRot + pCenter;
}

/// <summary>
/// Rotates the a pQuery arount pCenter whith alpha degree
/// visual_trackingsummary>
cv::Point2f M_KalmanFilter::RotateAroundPoint(cv::Point2f pQuery, double alpha) {
	cv::Point2f pCenter;
	cv::Point2f p = pQuery - pCenter;
	return RotateCoordinateAxis(-alpha, p) + pCenter;
}


void M_KalmanFilter::RotateCoordinateAxis(double alpha, Point2d p, Point2d &res) {
	alpha = m_Math::Degree2Radian(alpha);
	res = Point2d((float) (p.x * cos(alpha) - p.y * sin(alpha)),
			(float) (p.x * sin(alpha) + p.y * cos(alpha)));
}


/// <summary>
/// Rotates the coordinate axises and returns the new coordinate of insetreted point
/// visual_trackingsummary>
/// <param name="alpha">angle (in degree) to rotate the coordinate axisesvisual_trackingparam>
/// <param name="p">the point in old coordinate axisvisual_trackingparam>
/// <returns>the point in new coordinate axisvisual_trackingreturns>
cv::Point2f M_KalmanFilter:: RotateCoordinateAxis(double alpha, cv::Point2f p) {
	alpha = m_Math::Degree2Radian(alpha);
	return cv::Point2f((float) (p.x * cos(alpha) - p.y * sin(alpha)),
			(float) (p.x * sin(alpha) + p.y * cos(alpha)));
}


void M_KalmanFilter::setCurTime(ros::Time t){
//     if(t<cur_time_stamp){timeToshift = 0;}
//     else{timeToshift = params.projection.timeToShift->get();}
    cur_time_stamp = t;
   
}

//correction
void M_KalmanFilter::measurementSub( const  geometry_msgs::PoseStamped& msg, float conf ){
       
	Eigen:: Matrix<float, 6, 6> measurement_cov;
	Eigen:: Matrix<float, 6, 1> measurement_pose;
	
        double roll, pitch, yaw;
	tf::Quaternion q(  msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z,  msg.pose.orientation.w);
	q.normalized();
	tf::Matrix3x3 m(q);
	m.getRPY(roll, pitch, yaw); 
	
	measurement_pose<< msg.pose.position.x, msg.pose.position.y, msg.pose.position.z, roll,  pitch, yaw;
	

	ros::Duration timeInterval(cur_time_stamp- pre_meas_time);
	  
	if(/*timeInterval.toSec() > 1.0 ||*/ init==1){
	      estimate_cov << 5.5, 0.0, 0.0, 0.0, 0.0, 0.0,
			      0.0, 5.5, 0.0, 0.0, 0.0, 0.0,
			      0.0, 0.0, 5.5, 0.0, 0.0, 0.0,
			      0.0, 0.0, 0.0, 5.5, 0.0, 0.0,
			      0.0, 0.0, 0.0, 0.0, 5.5, 0.0,
			      0.0, 0.0, 0.0, 0.0, 0.0, 5.5;
	      estimate_pose = measurement_pose;

	      predict_pose = estimate_pose;

// 	      pre_estimate_cov = estimate_cov;

	      init = 0;
	}
	else if(timeInterval.toSec() <= 0){
	      pre_meas_time = cur_time_stamp;
	      return;
	}
	else{
	  
	  
	      measurement_cov.setZero();
      
	      if( fabs(roll) > 0.5 * 0.5*M_PI  || fabs(pitch) > 0.5* 0.5*M_PI || /*m_Math::RadianAngleDiff(yaw, getHeading()) > M_PI/2.0||*/
		  measurement_pose(2,0) >1.5 || measurement_pose(2,0) < 0.1 || 
		  sqrt(pow(measurement_pose(0,0)-estimate_pose(0,0),2) +pow(measurement_pose(0,0)-estimate_pose(1,0),2) 
		      +pow(measurement_pose(1,0)-estimate_pose(1,0),2)) >3.0){
		    measurement_cov(0,0) = measurement_cov(1,1) = measurement_cov(2,2) 
		    = measurement_cov(3,3) = measurement_cov(4,4) =measurement_cov(5,5) = 9999999;
		    cout<<"Bad Measurement"<<endl;
	      }
	      else{
		  double conf1= conf;
		  measurement_cov(0,0) = (1 -std::min(conf1, 0.9))* params.icp.KFPositionCov->get();
		  measurement_cov(1,1) = (1 -std::min(conf1, 0.9))* params.icp.KFPositionCov->get();
		  measurement_cov(2,2) = (1 -std::min(conf1, 0.9))* params.icp.KFPositionCov->get();
		  
		  measurement_cov(3,3) = (1 -std::min(conf1, 0.9))* params.icp.KFOrientationCov->get();
		  measurement_cov(4,4) = (1 -std::min(conf1, 0.9))* params.icp.KFOrientationCov->get();
		  measurement_cov(5,5) = (1 -std::min(conf1, 0.9))* params.icp.KFOrientationCov->get();  
	      }
	

	    //Kalman Filter
	    Eigen:: Matrix<float, 6, 6> K = estimate_cov * ((estimate_cov + measurement_cov ).inverse());
	    estimate_pose = predict_pose + K * (measurement_pose - predict_pose);
	    Eigen::Matrix<float, 6, 6> I = Eigen:: Matrix<float, 6, 6>::Identity();
	    estimate_cov = (I - K) * estimate_cov;
	    

	}
     
	
	estimate_pose_cov.header = msg.header;
	estimate_pose_cov.pose.pose.position.x = estimate_pose(0,0);
	estimate_pose_cov.pose.pose.position.y = estimate_pose(1,0);
	estimate_pose_cov.pose.pose.position.z = estimate_pose(2,0);
      
	tf::Quaternion q2;//(estimate_pose(3,0), estimate_pose(4,0), estimate_pose(5,0));
	
	estimate_pose(3,0) = clampRotation(estimate_pose(3,0));
	estimate_pose(4,0) = clampRotation(estimate_pose(4,0));
	estimate_pose(5,0) = clampRotation(estimate_pose(5,0));
	
	q2.setRPY (estimate_pose(3,0), estimate_pose(4,0), estimate_pose(5,0));
	

	estimate_pose_cov.pose.pose.orientation.x = q2.x();
	estimate_pose_cov.pose.pose.orientation.y = q2.y();
	estimate_pose_cov.pose.pose.orientation.z = q2.z();
	estimate_pose_cov.pose.pose.orientation.w = q2.w();
	    

      
	for(int i = 0;i<36;i++){
	    estimate_pose_cov.pose.covariance[i] = estimate_cov(i/6, i%6);
	}

	predict_pose = estimate_pose;

      
}


//prediction
void M_KalmanFilter::dead_reckoning_callback(const gait_msgs::GaitOdomConstPtr & msg)
	{   
	    ros::Duration timeInterval(cur_time_stamp - pre_meas_time);
	    if( timeInterval.toSec() < 0.5 &&timeInterval.toSec()>0 ){
		Point3d curOdom;
		curOdom.x = msg->odom2D.x;
		curOdom.y = msg->odom2D.y;
		curOdom.z = msg->odom2D.theta;

		if (lastOdomID == msg->ID)
		{
			Point3d diffOdom = curOdom - lastOdom;
			globalPos += diffOdom;
			Point2d diffOdom2D(diffOdom.x,diffOdom.y);
			Point2d diffOdom2DCancelOdomRotation=RotateAroundPoint(diffOdom2D,m_Math::Radian2Degree(curOdom.z));

			Point2d diffAbsolut=RotateAroundPoint(diffOdom2DCancelOdomRotation,-m_Math::Radian2Degree(getHeading()));
			predict_pose(0, 0) += diffAbsolut.x;
	                predict_pose(1, 0) += diffAbsolut.y;
			predict_pose(5, 0) += diffOdom.z;
			predict_pose(5, 0) = clampRotation(predict_pose(5, 0));
			
// 			cout<<"diffAbsolut.x  "<<diffAbsolut.x<<"        diffAbsolut.y  "<<diffAbsolut.x<<"   diffOdom.z "<<diffOdom.z<<endl;
			params.location.x->set(predict_pose(0, 0));
	                params.location.y->set(predict_pose(1, 0));
			params.location.z->set(predict_pose(2,0));
			params.orientation.x->set(predict_pose(3,0));
			params.orientation.y->set(predict_pose(4,0));
			params.orientation.z->set(predict_pose(5,0));

// 			Localization.x += diffAbsolut.x;
// 			Localization.y += diffAbsolut.y;

		}

		lastOdom = curOdom;
		lastOdomID = msg->ID;
	    }
	    
	    pre_meas_time = cur_time_stamp;
}

 
// void M_KalmanFilter::updateTransform(){
// //       static tf::TransformBroadcaster br;
// //       tf::Transform transform;
// //       transform.setOrigin( tf::Vector3( state_(StateMemberX),  state_(StateMemberY),  state_(StateMemberZ)));
// //       tf::Quaternion quat;
// //       quat.setRPY(state_(StateMemberRoll), state_(StateMemberPitch), state_(StateMemberYaw));
// //       transform.setRotation(quat);
// //       br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map_slam", "base_link"));
// //       br.sendTransform(tf::StampedTransform(transform, ros::Time(0), "world", "ego_floor"));
//       
// }
// 
// 
// 
// 
double M_KalmanFilter::clampRotation(double rotation){
      while (rotation > PI)
      {
	rotation -= TAU;
      }

      while (rotation < -PI)
      {
	rotation += TAU;
      }

      return rotation;
  
}

