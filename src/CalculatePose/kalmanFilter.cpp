
// #include "kalmanFilter.h"
#include <visual_tracking/CalculatePose/kalmanFilter.h>
using namespace vision;


M_KalmanFilter::M_KalmanFilter()/*:listener(ros::Duration(3.0))*/{
      robotPoseInit();
     
      
      pre_meas_time = ros::Time::now();
      cur_time_stamp = ros::Time::now();
      init = 1;
      speed_x =0; 
      speed_y=0; 
      speed_yaw=0;
      useMotionOdom = params.debug.useMotionOdom->get();
      
      unexpectPose_n =0;
      
      
      lastOdomID = 0;
      globalPos.x = 0;
      globalPos.y = 0;
      globalPos.z = 0;
      lastOdom.x = 0;
      lastOdom.y = 0;
      lastOdom.z = 0;
      
      curOdomID =0;
      previousOdom.x =  params.location.x->get();;
      previousOdom.y =  params.location.y->get();;
      previousOdom.z =  params.orientation.z->get();
      currentOdom.x =   params.location.x->get();;
      currentOdom.y =   params.location.y->get();;
      currentOdom.z =   params.orientation.z->get();
      
      heading_sub_robotstate = nodeHandle.subscribe("/robotmodel/robot_heading", 1, &M_KalmanFilter::handleHeadingData, this);
     
      if(useMotionOdom){
//       odom_sub = nodeHandle.subscribe("/gait/odometry", 1, &M_KalmanFilter::prediction2, this);}
      odom_sub = nodeHandle.subscribe("/gait/odometry", 1, &M_KalmanFilter::getOdom, this);}
      headingOffset = params.icp.HeadingOffset->get() ;

}
 
void M_KalmanFilter::robotPoseInit(){
   
  
    
   estimate_pose<<params.location.x->get(),params.location.y->get(),params.location.z->get(),
                 params.orientation.x->get(),params.orientation.y->get(),params.orientation.z->get();
		 
    predict_pose = estimate_pose;
    
    
    tf::Quaternion q;//(estimate_pose(3,0), estimate_pose(4,0), estimate_pose(5,0));
    q.setRPY (predict_pose(3,0), predict_pose(4,0), predict_pose(5,0));
    robotPoseS.header.frame_id = "world";
    robotPoseS.pose.position.x = predict_pose(0,0);
    robotPoseS.pose.position.y = predict_pose(1,0);
    robotPoseS.pose.position.z = predict_pose(2,0);
    robotPoseS.pose.orientation.x = q.x();
    robotPoseS.pose.orientation.y = q.y();
    robotPoseS.pose.orientation.z = q.z();
    robotPoseS.pose.orientation.w = q.w();
    
    
    
    estimate_cov << 5, 0.0, 0.0, 0.0, 0.0, 0.0,
		0.0, 5, 0.0, 0.0, 0.0, 0.0,
		0.0, 0.0, 5, 0.0, 0.0, 0.0,
		0.0, 0.0, 0.0, 5, 0.0, 0.0,
		0.0, 0.0, 0.0, 0.0, 5, 0.0,
		0.0, 0.0, 0.0, 0.0, 0.0, 5;

  
}


void M_KalmanFilter::getMeasurementCov(double (&cov)[6]){
  
    for(int idx =0; idx<6; idx++){
      measurementCov[idx]  = cov[idx];
    }
  
  
  
}


//from Hafez
double M_KalmanFilter::getHeading()
{       headingOffset = params.hillclimbing.HeadingOffset->get() ;
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
void M_KalmanFilter::correction(const geometry_msgs::PoseStamped& msg, float conf ){
        ros::Duration timeInterval(cur_time_stamp- pre_meas_time);
//         if( timeInterval.toSec() ==0 ){return;}
       
	Eigen:: Matrix<float, 6, 6> measurement_cov;
	Eigen:: Matrix<float, 6, 1> measurement_pose;
	
        double roll, pitch, yaw;
	tf::Quaternion q(  msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z,  msg.pose.orientation.w);
	q.normalized();
	tf::Matrix3x3 m(q);
	m.getRPY(roll, pitch, yaw); 
	
	measurement_pose<< msg.pose.position.x, msg.pose.position.y, msg.pose.position.z, roll,  pitch, yaw;
	
	  
	if( timeInterval.toSec() < 0||init==1 ){//initialize
	      estimate_cov << 0.5, 0.0, 0.0, 0.0, 0.0, 0.0,
			      0.0, 0.5, 0.0, 0.0, 0.0, 0.0,
			      0.0, 0.0, 0.5, 0.0, 0.0, 0.0,
			      0.0, 0.0, 0.0, 0.5, 0.0, 0.0,
			      0.0, 0.0, 0.0, 0.0, 0.5, 0.0,
			      0.0, 0.0, 0.0, 0.0, 0.0, 0.5;
			      
	      estimate_cov << 5, 0.0, 0.0, 0.0, 0.0, 0.0,
		              0.0, 5, 0.0, 0.0, 0.0, 0.0,
		              0.0, 0.0, 5, 0.0, 0.0, 0.0,
		              0.0, 0.0, 0.0, 5, 0.0, 0.0,
		              0.0, 0.0, 0.0, 0.0, 5, 0.0,
		              0.0, 0.0, 0.0, 0.0, 0.0, 5;
	      estimate_pose = measurement_pose;

	      predict_pose = estimate_pose;

	      init = 0;
	}
	else if(!(params.debug.useKalmanFilter->get())){
// 	      pre_meas_time = cur_time_stamp;
	      return;
	}
	else{
	  
	  
	          measurement_cov.setZero(); 
		  
		  measurement_cov(0,0) = measurementCov[0];
		  measurement_cov(1,1) = measurementCov[1];
		  measurement_cov(2,2) = measurementCov[2];
		  
		  measurement_cov(3,3) = measurementCov[3];
		  measurement_cov(4,4) = measurementCov[4];
		  measurement_cov(5,5) = measurementCov[5]; 
	  

			  
		Eigen:: Matrix<float, 6, 6> K = estimate_cov * ((estimate_cov + measurement_cov ).inverse());
		
		estimate_pose = predict_pose + K * (measurement_pose - predict_pose);
		Eigen::Matrix<float, 6, 6> I = Eigen:: Matrix<float, 6, 6>::Identity();
	        estimate_cov = (I - K) * estimate_cov;
	     
		
		//prevent the covariance to be 0
		estimate_cov(0,0) += 1e-7;
		estimate_cov(1,1) += 1e-7;
		estimate_cov(2,2) += 1e-7;
		
		estimate_cov(3,3) += 1e-7;
		estimate_cov(4,4) += 1e-7;
		estimate_cov(5,5) += 1e-7;
		
	     
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
	    
        robotPoseS.header = msg.header;
        robotPoseS.pose = estimate_pose_cov.pose.pose;
	
	for(int i = 0;i<36;i++){
	    estimate_pose_cov.pose.covariance[i] = estimate_cov(i/6, i%6);
	}
	
      if( timeInterval.toSec() >0 ){
	  speed_x = (estimate_pose(0,0) -predict_pose(0,0))/timeInterval.toSec(); 
	  speed_y = (estimate_pose(1,0) -predict_pose(1,0))/timeInterval.toSec(); 
	  if((estimate_pose(5,0) -predict_pose(5,0)) >M_PI){
	    speed_yaw = (2*M_PI - (estimate_pose(5,0) -predict_pose(5,0)))/timeInterval.toSec(); 
	  }
	  else{
	  speed_yaw = (estimate_pose(5,0) -predict_pose(5,0))/timeInterval.toSec();}
	
	}
	
	
	predict_pose = estimate_pose;

      
}


//prediction
void M_KalmanFilter::prediction()
	{   
	    ros::Duration timeInterval(cur_time_stamp - pre_meas_time);
	    if( timeInterval.toSec() < 0.5 &&timeInterval.toSec()>0 ){
		
	        predict_pose(0, 0) += speed_x *timeInterval.toSec();
		predict_pose(1, 0) += speed_y *timeInterval.toSec();
		predict_pose(5, 0) += speed_yaw *timeInterval.toSec();
		predict_pose(5, 0)  = clampRotation(predict_pose(5, 0));

		
		tf::Quaternion q;//(estimate_pose(3,0), estimate_pose(4,0), estimate_pose(5,0));
	        q.setRPY (predict_pose(3,0), predict_pose(4,0), predict_pose(5,0));
		robotPoseS.pose.position.x = predict_pose(0,0);
		robotPoseS.pose.position.y = predict_pose(1,0);
		robotPoseS.pose.position.z = predict_pose(2,0);
		robotPoseS.pose.orientation.x = q.x();
		robotPoseS.pose.orientation.y = q.y();
		robotPoseS.pose.orientation.z = q.z();
		robotPoseS.pose.orientation.w = q.w();
		estimate_pose_cov.pose.pose = robotPoseS.pose;


	    }
	    
	    pre_meas_time = cur_time_stamp;
}


void M_KalmanFilter::mainLoop(const geometry_msgs::PoseStamped& msg , float conf){
    
  
      if(!useMotionOdom){
         prediction();
      }
      else{
	
	prediction3();
	
      }
      correction( msg, conf );


}


void M_KalmanFilter::getOdom(const gait_msgs::GaitOdomConstPtr & msg){
      curOdomID =  msg->ID;
      currentOdom.x =   msg->odom2D.x;
      currentOdom.y =   msg->odom2D.y;
      currentOdom.z =   msg->odom2D.theta;		
		
}
void M_KalmanFilter::prediction3()//use motionOdom
  {
    ros::Duration timeInterval(cur_time_stamp - pre_meas_time);
    if( timeInterval.toSec() < 1.0 && timeInterval.toSec()>=0 ){
      
      
		if (lastOdomID == curOdomID)
		{
			Point3d diffOdom = currentOdom - previousOdom;
			globalPos += diffOdom;
			Point2d diffOdom2D(diffOdom.x,diffOdom.y);
			Point2d diffOdom2DCancelOdomRotation=RotateAroundPoint(diffOdom2D,m_Math::Radian2Degree(currentOdom.z));

			Point2d diffAbsolut=RotateAroundPoint(diffOdom2DCancelOdomRotation, -m_Math::Radian2Degree(getHeading()));
			predict_pose(0, 0) += diffAbsolut.x  ;
	                predict_pose(1, 0) += diffAbsolut.y  ;
			
			if(params.debug.useYawSpeed->get()){
			    predict_pose(5, 0) += diffOdom.z  ;
			    predict_pose(5, 0) = clampRotation(predict_pose(5, 0));
			}
			

			
			
			tf::Quaternion q;//(estimate_pose(3,0), estimate_pose(4,0), estimate_pose(5,0));
			q.setRPY (predict_pose(3,0), predict_pose(4,0), predict_pose(5,0));
			robotPoseS.pose.position.x = predict_pose(0,0);
			robotPoseS.pose.position.y = predict_pose(1,0);
			robotPoseS.pose.position.z = predict_pose(2,0);
			robotPoseS.pose.orientation.x = q.x();
			robotPoseS.pose.orientation.y = q.y();
			robotPoseS.pose.orientation.z = q.z();
			robotPoseS.pose.orientation.w = q.w();
			estimate_pose_cov.pose.pose = robotPoseS.pose;
			

		}
      
    }
    else{
      
        predict_pose.setZero();
	predict_pose(0, 0) = params.location.x->get();
	predict_pose(1, 0) = params.location.y->get();
	predict_pose(2, 0) = params.location.z->get();
	predict_pose(5, 0) =  params.orientation.z->get();
       
    }
    estimate_pose = predict_pose;
    lastOdomID = curOdomID;
    previousOdom = currentOdom;
    pre_meas_time = cur_time_stamp;
  }

void M_KalmanFilter::prediction2(const gait_msgs::GaitOdomConstPtr & msg)//use motionOdom
	{   
	  
	    ros::Duration timeInterval(cur_time_stamp - pre_meas_time);
	    if( timeInterval.toSec() < 1.0 &&timeInterval.toSec()>0 ){
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

			Point2d diffAbsolut=RotateAroundPoint(diffOdom2DCancelOdomRotation, -m_Math::Radian2Degree(getHeading()));
			predict_pose(0, 0) += (diffAbsolut.x ) ;
	                predict_pose(1, 0) += (diffAbsolut.y ) ;
			
			predict_pose(5, 0) += (diffOdom.z) ;
			predict_pose(5, 0) = clampRotation(predict_pose(5, 0));
			
			tf::Quaternion q;//(estimate_pose(3,0), estimate_pose(4,0), estimate_pose(5,0));
			q.setRPY (predict_pose(3,0), predict_pose(4,0), predict_pose(5,0));
			robotPoseS.pose.position.x = predict_pose(0,0);
			robotPoseS.pose.position.y = predict_pose(1,0);
			robotPoseS.pose.position.z = predict_pose(2,0);
			robotPoseS.pose.orientation.x = q.x();
			robotPoseS.pose.orientation.y = q.y();
			robotPoseS.pose.orientation.z = q.z();
			robotPoseS.pose.orientation.w = q.w();
			estimate_pose_cov.pose.pose = robotPoseS.pose;

		}

		lastOdom = curOdom;
		lastOdomID = msg->ID;
	    }
	    
	    pre_meas_time = cur_time_stamp;
}

 

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

