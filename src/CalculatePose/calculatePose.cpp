// by Yongfeng
// #include "calculatePose.h"

#include <vector>

#include <visual_tracking/CalculatePose/calculatePose.h>




Eigen::Matrix4d PoseCalculator::getRelativeTransform(std::vector<cv::Point2f> imgPts,  std::vector<cv::Point3f> worldPts) {
  cv::Mat rvec, tvec;
  
  
  
  cv::Matx33f cameraMatrix(
                           CamParam::fx,      0,       CamParam::cx,
                           0,            CamParam::fy, CamParam::cy,
                           0,                  0,          1         );
  cv::Vec4f distParam(0,0,0,0); // all 0?
  
  
  Mat inliers;
  
  //EPnP + RANSAC
  if(params.debug.useRansac->get()){
	cv::solvePnPRansac(worldPts, imgPts, cameraMatrix, distParam, rvec, tvec, 1, 100, 10.0, 20, inliers, CV_EPNP);

    
  }else{//EPnP
       cv::solvePnP(worldPts, imgPts, cameraMatrix, distParam, rvec, tvec, 1, CV_EPNP);

  }  
  
  
  
  cv::Matx33d r;
  cv::Rodrigues(rvec, r);
  Eigen::Matrix3d wRo;
  wRo << r(0,0), r(0,1), r(0,2), r(1,0), r(1,1), r(1,2), r(2,0), r(2,1), r(2,2);

  Eigen::Matrix4d T; 
  T.topLeftCorner(3,3) = wRo;
  T.col(3).head(3) << tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2);
  T.row(3) << 0,0,0,1;  

  return T;
}



void PoseCalculator::calculatePose(std::vector<cv::Point2f> imgPts,  std::vector<cv::Point3f> worldPts,ForwardProjection & forw_Proj, geometry_msgs::PoseStamped& cameraPose,geometry_msgs::PoseStamped& robotPose){
     
      cameraPose.header.frame_id = "world";
      cameraPose.header.stamp =  ros::Time::now();
     
      
     if(imgPts.size()>=3){
      Eigen::Matrix4d transform = getRelativeTransform(imgPts, worldPts); 
      

      Eigen::Matrix4d transformInv = transform.inverse();
      
      Eigen::Matrix3d rot = transformInv.block(0,0,3,3);
      Eigen::Quaternion<double> rot_quaternion = Eigen::Quaternion<double>(rot);

      
      cameraPose.pose.position.x = transformInv(0,3);
      cameraPose.pose.position.y = transformInv(1,3);
      cameraPose.pose.position.z = transformInv(2,3);
      cameraPose.pose.orientation.x = rot_quaternion.x();
      cameraPose.pose.orientation.y = rot_quaternion.y();
      cameraPose.pose.orientation.z = rot_quaternion.z();
      cameraPose.pose.orientation.w = rot_quaternion.w();
      
     }
     else{
       
      cameraPose.pose.position.x = 0.0;
      cameraPose.pose.position.y = 0.0;
      cameraPose.pose.position.z = 0.0;
      cameraPose.pose.orientation.x = 0.0;
      cameraPose.pose.orientation.y = 0.0;
      cameraPose.pose.orientation.z = 0.0;
      cameraPose.pose.orientation.w = 1.0;

    }
    
     forw_Proj.TfCamPose2RobotPose(cameraPose, robotPose);

}


