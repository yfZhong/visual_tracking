#ifndef PCLCLUSTERING_H
#define PCLCLUSTERING_H

#include <ros/ros.h>
#include <iostream>
#include <stdio.h>
#include <vector>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <Eigen/Dense>
#include <Eigen/Eigenvalues>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>

#include <math.h>
#include <pcl/common/common.h>
#include <pcl/common/angles.h>
#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>


using namespace std;

namespace vision
{  
  bool computePose(geometry_msgs::PoseArray &particles,  geometry_msgs::Pose &p);
  std::pair<Eigen::Matrix4d, Eigen::Vector4d> computeEigenValuesAndVectors(Eigen::Matrix4d mat );

}




#endif

