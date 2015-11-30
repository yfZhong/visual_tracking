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
  cv::solvePnP(worldPts, imgPts, cameraMatrix, distParam, rvec, tvec);
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
/*      
      if(1){
	    tf::Quaternion q(rot_quaternion.x(), rot_quaternion.y(), rot_quaternion.z(), rot_quaternion.w());
	    q.normalized();
	    tf::Matrix3x3 m(q);
	    double roll0, pitch0, yaw0;
	    m.getRPY(roll0, pitch0, yaw0); 
	    cout<<"2 "<< roll0<< "," << pitch0<< "," << yaw0<<endl;
	
      }*/
      

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






// void getRelativeTranslationRotation(Eigen::Matrix4d T, Eigen::Vector3d& trans, Eigen::Matrix3d& fixed_rot) {
//   
//   // converting from camera frame (z forward, x right, y down) to
//   // object frame (x forward, y left, z up)
//   Eigen::Matrix4d M;
//     M <<
//       0,  0, 1, 0,
//       -1, 0, 0, 0,
//       0, -1, 0, 0,
//       0,  0, 0, 1;
//     Eigen::Matrix4d MT = M*T;
//     
// 
//     Eigen::Matrix3d rot;
//     
//     // translation vector from camera to the April tag
//     trans = MT.col(3).head(3);
//     // orientation of April tag with respect to camera: the camera
//     // convention makes more sense here, because yaw,pitch,roll then
//     // naturally agree with the orientation of the object
//     rot = T.block(0,0,3,3);
//     
//     Eigen::Matrix3d F;
//     F <<
//       1, 0,  0,
//       0,  -1,  0,
//       0,  0,  1;
//    fixed_rot = F*rot;
//   
//   
// }
// 
// 
// 
// 
// 
// 
// 
// 
// void draw(cv::Mat& image,vector<pair<int,int> > p) {
//   // use corner points detected by line intersection
//   std::pair<int,int> p1 = p[0];
//   std::pair<int,int> p2 = p[1];
//   std::pair<int,int> p3 = p[2];
//   std::pair<int,int> p4 = p[3];
// 
//   // plot outline
//   cv::line(image, cv::Point2f(p1.first, p1.second), cv::Point2f(p2.first, p2.second), cv::Scalar(255,0,0,0),1 );
//   cv::line(image, cv::Point2f(p2.first, p2.second), cv::Point2f(p3.first, p3.second), cv::Scalar(0,255,0,0) ,1);
//   cv::line(image, cv::Point2f(p3.first, p3.second), cv::Point2f(p4.first, p4.second), cv::Scalar(0,0,255,0),1 );
//   cv::line(image, cv::Point2f(p4.first, p4.second), cv::Point2f(p1.first, p1.second), cv::Scalar(255,0,255,0),1);
// 
//   // mark center
//   //cv::circle(image, cv::Point2f(cxy.first, cxy.second), 8, cv::Scalar(0,0,255,0), 2);
// 
//   // print ID
//   std::ostringstream strSt;
//   //strSt << "#" << id;
//   //cv::putText(image, strSt.str(),
//   //            cv::Point2f(cxy.first + 10, cxy.second + 10),
//   //            cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0,0,255));
// }
// 
// double standardRad(double t) {
//   if (t >= 0.) {
//     t = fmod(t+PI, TWOPI) - PI;
//   } else {
//     t = fmod(t-PI, -TWOPI) + PI;
//   }
//   return t;
// }
// 
// 
// void wRo_to_euler(const Eigen::Matrix3d& wRo, double& yaw, double& pitch, double& roll) {
//     yaw = standardRad(atan2(wRo(1,0), wRo(0,0)));
//     double c = cos(yaw);
//     double s = sin(yaw);
//     pitch = standardRad(atan2(-wRo(2,0), wRo(0,0)*c + wRo(1,0)*s));
//     roll  = standardRad(atan2(wRo(0,2)*s - wRo(1,2)*c, -wRo(0,1)*s + wRo(1,1)*c));
// }
// 
// //calculate yaw, pitch, roll;
// void rotMat2RPY(){
//   
// 	//
// 	//rotatioon matrix to yaw, pitch, roll;
// 	//
// 	tf::Matrix3x3 q(//0.426549923806777,  -0.016647544117955,  -0.904310799324697,
// 			//-0.011483610039421,  0.999650322539005,  -0.023819306205813,
// 			//0.904391115171359,   0.020544875821102,   0.426209595007668);
// 	
// 			//0.436855698485042,  -0.031921087052746,  -0.898965039866688,
// 			//-0.016903848996969,   0.998902371732617,  -0.043684226398561,
// 			//0.899372758421242,   0.034279672523582,   0.435836604086259);
// 	
// 	 
// 	      0.998, -0.040, -0.041,  
//               0.039,  0.999, -0.018, 
//               0.041,  0.016,  0.999); 
// 
//              //t = < 0.010, -0.014, -0.020 >
// 	
// 	
// 	double yaw, pitch, roll;
// 	
// 	tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
// 	
// 	cout<<"RPY-----------------------\n: "<< roll<<","<<pitch<<", "<<yaw<<endl;
//         cout<<"-------------------------\n: ";
// }
