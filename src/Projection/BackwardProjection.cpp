/*

Authors: Yongfeng

*/

// #include "BackwardProjection.h"

#include <visual_tracking/Projection/BackwardProjection.h>

#include <stdlib.h>     /* srand, rand */



using namespace vision;

BackwardProjection::BackwardProjection():listener(ros::Duration(10)){
  robotPoseInit();
  cur_time_stamp = ros::Time::now();
  cameraMatrix = distortionModel.cameraMatrix;
  distCoeffs = distortionModel.distCoeffs;
  fx = distortionModel.cameraMatrix.at<double>(0, 0);
  fy = distortionModel.cameraMatrix.at<double>(1, 1);
  cx = distortionModel.cameraMatrix.at<double>(0, 2); 
  cy = distortionModel.cameraMatrix.at<double>(1, 2); 
  
  W = params.camera.width->get();
  H = params.camera.height->get();
  siX = params.camera.widthUnDistortion->get();
  siY = params.camera.heightUnDistortion->get();
  offsetx = (siX - W) / 2.;
  offsety = (siY - H) / 2.;
  
  image_transport::ImageTransport it_(node);
  timeToshift = params.projection.timeToShift->get();
//   idx =0;
}

void BackwardProjection::robotPoseInit(){
  
  rbPose.position.x=0;rbPose.position.y=0;rbPose.position.z=0.6;
  rbPose.orientation.x= 0;rbPose.orientation.y= 0;rbPose.orientation.z= 0;rbPose.orientation.w= 1.0;
  
}



void BackwardProjection::setRobotPose(geometry_msgs::Pose p){
   rbPose = p;
  //updating transform
   tf::Transform tf_world2Egorot;
   tf::poseMsgToTF (rbPose, tf_world2Egorot);

   
      tf_World2Cam =  tf_Egorot2Cam*tf_world2Egorot.inverse();
      
      
      rotation_World2Cam = tf_World2Cam.getBasis();
      translation_World2Cam =  tf_World2Cam.getOrigin();
      
      rotation_Cam2World =  tf_World2Cam.inverse().getBasis();
      translation_Cam2World =  tf_World2Cam.inverse().getOrigin();
      


}

 void BackwardProjection::setCurTime(ros::Time t){
   if(t<cur_time_stamp){timeToshift = 0;}
   else{timeToshift = params.projection.timeToShift->get();}
   cur_time_stamp = t;
}

 void BackwardProjection::setTfEgorot2Cam(){
   tf::StampedTransform tfmsg;
         try{
// 		  if(listener.waitForTransform("camera_optical","ego_rot",  ros::Time::now()- ros::Duration(timeToshift), ros::Duration(0.5))){
// 		    listener.lookupTransform("camera_optical","ego_rot",   ros::Time::now() - ros::Duration(timeToshift), tfmsg);
// 		  if(listener.waitForTransform("camera_optical","ego_rot",  cur_time_stamp- ros::Duration(timeToshift), ros::Duration(0.5))){
		    listener.waitForTransform("camera_optical","ego_rot",  cur_time_stamp- ros::Duration(timeToshift), ros::Duration(0.5));
		    listener.lookupTransform("camera_optical","ego_rot",   cur_time_stamp - ros::Duration(timeToshift), tfmsg);
// 		  }
// 		  else{
// 		      geometry_msgs::TransformStamped msg;
// 		      msg.transform.translation.x = -0.0320715;
// 		      msg.transform.translation.y =  0.2620675;
// 		      msg.transform.translation.z = -0.0653293;
// 		      msg.transform.rotation.x = 0.481647;
// 		      msg.transform.rotation.y = -0.471644;
// 		      msg.transform.rotation.z = 0.526234;
// 		      msg.transform.rotation.w = 0.518311;
// 		      tf::transformStampedMsgToTF(msg,tfmsg);
// 		      cout<<"No tf from ego_rot to camera_optical."<<endl;
// 		}
		tf_Egorot2Cam =  static_cast<const tf::Transform&>(tfmsg);
	   
	}
      catch(tf::TransformException& ex){
	  ROS_ERROR("Received an exception trying to transform a pose : %s", ex.what());
      }   
}


void BackwardProjection::onInit(ros::Time t ){
    setCurTime(t);
    setTfEgorot2Cam();
}

void BackwardProjection::projectImgPoint2WorldCord(cv::Point p, cv::Point3f& pOut){
      
      tf::Vector3 pointInCamCordNorm((p.x - (cx + offsetx) ) / fx,
                                     (p.y - (cy + offsety) ) / fy,
				                      1    );
      
      float z = -translation_Cam2World.z()/(rotation_Cam2World[2].dot(pointInCamCordNorm));
      tf::Vector3 pointInCamCord = tf::Vector3( z * pointInCamCordNorm.x(),
						z * pointInCamCordNorm.y(),
						            z);
      
       pOut = cv::Point3f( rotation_Cam2World[0].dot(pointInCamCord) + translation_Cam2World.x(), 
			   rotation_Cam2World[1].dot(pointInCamCord) + translation_Cam2World.y(), 
			                          0);

}

bool BackwardProjection::projectWordPoint2Img(cv::Point3f pIn, cv::Point & pOut){

        tf::Vector3 vIn(pIn.x, pIn.y,pIn.z);
        tf::Vector3 v2( rotation_World2Cam[0].dot(vIn) + translation_World2Cam.x(), 
		        rotation_World2Cam[1].dot(vIn) + translation_World2Cam.y(), 
		        rotation_World2Cam[2].dot(vIn) + translation_World2Cam.z());
        if( v2.z() != 0 ){

	    pOut.x = v2.x()/v2.z() * fx + cx + offsetx ;
	    pOut.y = v2.y()/v2.z() * fy + cy + offsety ;
	    
	    return true;
	}
	else{return false;}
}


void BackwardProjection::TfRobotPose2CamPose(geometry_msgs::PoseArray& robotPoseArr, geometry_msgs::PoseArray &cameraPoseArr){
    cameraPoseArr.header = robotPoseArr.header;
    cameraPoseArr.poses.clear();
    
    geometry_msgs::Pose camPose; 
    for (unsigned int i =0; i< robotPoseArr.poses.size(); ++i){
      
       setRobotPose(robotPoseArr.poses[i]);
       geometry_msgs::Transform msg;
       tf::transformTFToMsg (tf_World2Cam.inverse(), msg);
       camPose.position.x = msg.translation.x;
       camPose.position.y = msg.translation.y;
       camPose.position.z = msg.translation.z;
       camPose.orientation = msg.rotation;
       cameraPoseArr.poses.push_back( camPose);
    }
}

void BackwardProjection::TfRobotPose2CamPose(geometry_msgs::PoseStamped &robotPose, geometry_msgs::PoseStamped &cameraPose){
       cameraPose.header = robotPose.header;
       
       setRobotPose(robotPose.pose);
       geometry_msgs::Transform msg;
       tf::transformTFToMsg (tf_World2Cam.inverse(), msg);
       cameraPose.pose.position.x = msg.translation.x;
       cameraPose.pose.position.y = msg.translation.y;
       cameraPose.pose.position.z = msg.translation.z;
       cameraPose.pose.orientation = msg.rotation;
   
}


void BackwardProjection::undistortP(cv::Point pIn, cv::Point & pOut){
  
        vector<Point3f> contour3f;

	contour3f.push_back(Point3f((pIn.x - (offsetx + cx)) / fx, (pIn.y - (offsety + cy)) / fy, 1));

	vector<Point2f> resCountourFloat;
	
	cv::Mat rVec(3, 1, cv::DataType<double>::type); // Rotation vector
	rVec.at<double>(0) = 0;
	rVec.at<double>(1) = 0;
	rVec.at<double>(2) = 0;

	cv::Mat tVec(3, 1, cv::DataType<double>::type); // Translation vector
	tVec.at<double>(0) = 0;
	tVec.at<double>(1) = 0;
	tVec.at<double>(2) = 0;

	cv::projectPoints(contour3f, rVec, tVec, cameraMatrix, distCoeffs, resCountourFloat);

        pOut = resCountourFloat[0];
  
}


void BackwardProjection::projectImgPoint2WorldCord_vec(vector<cv::Point> pIn, vector<cv::Point3f> &pOut){
      pOut.clear();
//       int idx = 0;
      for(unsigned int i =0; i< pIn.size(); ++i){
// 	  idx= i;
	  cv::Point3f p_world_0;
	  projectImgPoint2WorldCord(pIn[i],p_world_0);
	  pOut.push_back(p_world_0);
	
      }
}

