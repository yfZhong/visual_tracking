
// #include "singleParticle.h"
#include <visual_tracking/ParticleFilter/singleParticle.h>

using namespace vision;


SingleParticle::SingleParticle(){
  cameraPoseInit();
     
}

void SingleParticle::cameraPoseInit(){
    cameraPose.header.frame_id = "world";
    cameraPose.header.stamp =  ros::Time::now();
    cameraPose.pose.position.x = 0.0;
    cameraPose.pose.position.y = 0.0;
    cameraPose.pose.position.z = 0.6;
    cameraPose.pose.orientation.x = 0.0;
    cameraPose.pose.orientation.y = 0.0;
    cameraPose.pose.orientation.z = 0.0;
    cameraPose.pose.orientation.w = 1.0;
}

// void SingleParticle::run(vector<vector<int> > &detectedPoints, geometry_msgs::Pose &p, Model& model){
 void SingleParticle::run(std::vector<cv::Point> &undistDetectedPoints, geometry_msgs::Pose &p, Model& model){
	    // ******************************************* //
	    // * 4. Calculate the projected model points * //model points are stored in forw_Proj.points_in_image_disto
	    // ******************************************* //
	      model.forw_Proj.setRobotPose(p);
	      model.forw_Proj.mainLoop();

// 	    // ******************************* //
// 	    // * 5. ICP for data association * // 	 return pointMatcher.correspondences
// 	    // ******************************* //
	      model.pointMatcher.matcher(undistDetectedPoints, model.forw_Proj.ModelPointsInImg);
	      
	      
	      
// 	      
// 	      
// 	      
// 	    // ***************************************************************** //
// 	    // * 6. storing the correspondences and calculating the camera pose* //
// 	    // ***************************************************************** //


            const int W = params.camera.width->get();
	    const int H = params.camera.height->get();
	    const int siX = params.camera.widthUnDistortion->get();
	    const int siY = params.camera.heightUnDistortion->get();
	    int offsetx = (siX - W) / 2.;
	    int offsety = (siY - H) / 2.;
	    
	    for ( int i = 0; i<model.pointMatcher.inlierNum; ++i ){
	      imgPts.push_back(cv::Point(model.pointMatcher.correspondences[i*8 + 0]- offsetx , model.pointMatcher.correspondences[i*8 + 1]-offsety ));
	    }

	      

	    for (unsigned int i = 0; i<model.pointMatcher.inlierIdx.size(); ++i ){
		worldPts.push_back(cv::Point3f(model.forw_Proj.ModelPointsInWorldCord[model.pointMatcher.inlierIdx[i]].x,
					      model.forw_Proj.ModelPointsInWorldCord[model.pointMatcher.inlierIdx[i]].y,
					      model.forw_Proj.ModelPointsInWorldCord[model.pointMatcher.inlierIdx[i]].z));
	      
	    }
            
// 	    model.poseCalculator.calculatePose(imgPts, worldPts, cameraPose);
		

	}
	
geometry_msgs::PoseStamped SingleParticle::getCameraPose(){
  return cameraPose;

}

// vector<int> SingleParticle::getCorrespondences(){
//   
//   return model.pointMatcher.correspondences;
//   
// }
// 
// std::vector<cv::Point>  SingleParticle::getUndistPoints(){
//   
//   return undistPoints;
//   
// }
// 
std::vector<cv::Point2f> SingleParticle::getImgPts(){
  return imgPts;
  
}
std::vector<cv::Point3f> SingleParticle::getWorldPts(){
  
  return worldPts;
  
}
// 
// vector<vector<int> > SingleParticle::getProjectedPointsInImgDistored(){
//   
//   return forw_Proj.points_in_image_disto;
//   
// }
// 
// int SingleParticle::getInlierNum(){
//   
//   return pointMatcher.inlierNum;
// }
	
	