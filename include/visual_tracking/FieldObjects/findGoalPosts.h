#ifndef FINDGOALPOSTS_H
#define FINDGOALPOSTS_H

#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <algorithm>    // std::min_element, std::max_element
#include <vector>
#include <math.h>
#include <boost/timer/timer.hpp>
#include <Eigen/Dense>
#include <Eigen/Eigenvalues>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/eigen.hpp>


#include <visual_tracking/frameGrabber.h>
#include <visual_tracking/Tools/m_Math.h>
#include <visual_tracking/Projection/DistortionModel.h>

using namespace std;
// using namespace vision;
using namespace cv;



namespace vision
{ 
  class FindGoalPosts
	{
	public:
	  FindGoalPosts();
	  void findGoalPosts(Mat rawHSV, Mat B_Channel,Mat goalBinaryImg, vector<Point> fieldHull);
	  int H,W;
// 	  BackwardProjection backw_Proj;
	  
	  
	};
  
  
}









#endif


