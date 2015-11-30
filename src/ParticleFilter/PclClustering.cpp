// #include "PclClustering.h"


#include <visual_tracking/ParticleFilter/PclClustering.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/filters/conditional_removal.h>
#include <sensor_msgs/PointCloud2.h>


std::pair<Eigen::Matrix4d, Eigen::Vector4d> vision::computeEigenValuesAndVectors(Eigen::Matrix4d mat ){
    
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix4d> eigensolver(mat);
    Eigen::Vector4d eigenValues = Eigen::Vector4d::Identity();
    Eigen::Matrix4d eigenVectors = Eigen::Matrix4d::Zero();	
    
    if (eigensolver.info () == Eigen::Success)
    {
	eigenValues = eigensolver.eigenvalues();
	eigenVectors = eigensolver.eigenvectors();
// 	cout<<eigenValues.transpose() <<endl;
	
    }
    else
	{ROS_WARN_THROTTLE(1, "failed to compute eigen vectors/values. Is the covariance matrix correct?");}
    return std::make_pair (eigenVectors, eigenValues);

}

bool vision::computePose(geometry_msgs::PoseArray& particles, geometry_msgs::Pose &pout){
 
      if(particles.poses.size()>0){
	  int num = particles.poses.size();
	  //positions
	  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
	  cloud->width = num;
	  cloud->height  = 1;
	  cloud->is_dense = false;
	  cloud->points.resize( cloud->width * cloud->height); 
          //quaternions
// 	  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, 0, num, 4>quats;
// 	  Eigen::Matrix<double, num, 4> quats;
	  Eigen::Matrix<double, Eigen::Dynamic, 4> quats(num,4);
	/*  quats.resize(num, 4);*/ 
	  for(int j =0; j<num; ++j){
	      geometry_msgs::Pose p = particles.poses[j];
	      cloud->points[j].x = p.position.x;
	      cloud->points[j].y = p.position.y; 
	      cloud->points[j].z = p.position.z; 
// 	      cout<<resampleIdx[j]<<"  " << cloud->points[j].x<<"  "<< cloud->points[j].y<<"  "<< cloud->points[j].z<<endl;
	      quats(j,0) = p.orientation.x;
	      quats(j,1) = p.orientation.y;
	      quats(j,2) = p.orientation.z;
	      quats(j,3) = p.orientation.w;
	  }
	  
	  //clustering the particles according to the position
	  int _min = 0.1 * num;
	  int _max = num;
	  double tolerance = 0.05; //5cm
	  
	  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
	  tree->setInputCloud (cloud);
// 	  std::vector< int > k_indices;
// 	  std::vector< float > k_sqr_distance;
//           tree->nearestKSearch(pcl::PointXYZ(0,0,0), 1, k_indices, k_sqr_distance);

	  
	  std::vector<pcl::PointIndices> cluster_indices;
	  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
	  ec.setClusterTolerance (tolerance);
	  ec.setMinClusterSize (_min);
	  ec.setMaxClusterSize (_max);
	  ec.setSearchMethod (tree);
	  ec.setInputCloud (cloud);
	  ec.extract (cluster_indices);
          
// 	  std::cout << "cluster_indices.size: "<<cluster_indices.size()<<std::endl;
	  
	  //calculating the average position and quaternion on the largest cluster
	  if(cluster_indices.size() >0){
	      
	      std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); 
	      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
	      
	      //computer position
	      for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit){
		  cloud_cluster->points.push_back (cloud->points[*pit]); //*
		  cloud_cluster->width = cloud_cluster->points.size();
		  cloud_cluster->height = 1;
		  cloud_cluster->is_dense = true;
	      }
	      
// 	      std::cout << "The largest cluster contains "<< float(cloud_cluster->points.size ())/float(num) *1  << " pct of particles." << std::endl;

	      Eigen::Vector4f  centroid;
	      pcl::compute3DCentroid(*cloud_cluster,centroid);
// 	      std::cout << centroid.x()<<" "<< centroid.y()<<" "<< centroid.z()<<" "<<std::endl;
	      pout.position.x = centroid.x();
	      pout.position.y = centroid.y();
	      pout.position.z = centroid.z();
	      
	      
	      
	      
	       //computer quaternion
	      Eigen::Matrix4d quatMat= Eigen::Matrix4d::Zero();
	      for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit){
		 
		  Eigen:: Matrix<double, 1, 4> quat = quats.row( *pit );
		  quatMat = quat.transpose()*quat + quatMat;
	      }
	      quatMat = 1.0/double(num) * quatMat;
	      
	      std::pair<Eigen::Matrix4d, Eigen::Vector4d> eigenVectorsValues(vision::computeEigenValuesAndVectors(quatMat));
	    
	      int maxEigenValueIdx=0; double value = (eigenVectorsValues.second(0,0));
	      for(int i = 1; i<4;i++){
		if((eigenVectorsValues.second(i,0)) > value ){
		  maxEigenValueIdx = i; 
		  value = (eigenVectorsValues.second(i,0)) ;
		}
	      }
	      Eigen::Vector4d out= eigenVectorsValues.first.col(maxEigenValueIdx);
	      geometry_msgs::Quaternion qout;
	      qout.x = (out(0,0)); qout.y = (out(1,0)); qout.z = (out(2,0)); qout.w = (out(3,0));
// 	      qout.x = fabs(out(0,0)); qout.y = fabs(out(1,0)); qout.z = fabs(out(2,0)); qout.w = fabs(out(3,0));
	      
	      pout.orientation= qout;
	      
	      
	      
	      return true;
	  }
	  else return false;
      }
      else{return false;}
}
