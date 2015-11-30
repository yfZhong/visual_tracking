#ifndef FRAMEGRABBER_H
#define FRAMEGRABBER_H


#include <iostream>
#include <string>
#include <fstream>
#include <vector>
#include <algorithm>

#include <ros/ros.h>
#include <ros/package.h>
#include <ros/console.h>

#include <tf/transform_datatypes.h>
#include "Parameters/globaldefinitions.h"

#include "Parameters/Parameters.h"
#include "Projection/ForwardProjection.h"

using namespace std;

namespace vision
{
	
  
	/*! FrameGrabber class */
	class FrameGrabber
	{
		public:

			//! Timestamp
			/*! This variable contains the time stamp of the current frame. */
// 			double time;
			std_msgs::Header header;
			
			int H, W, siX, siY;
			
			
			std::vector<float > weightedWhiteValues;

			ForwardProjection forw_Proj;
			
			
			Mat rawHSV;
			Mat Brightness_Channel;
			Mat GreenBinary;
			Mat binaryImgs[3];// ball, goal, obstacle
			Mat fieldConvexHullMat;
			vector<cv::Point> fieldConvexHullPoints;
			vector< vector <cv::Point > > ObstacleConvexHull;
			
			
			int m_Top;
	
			unsigned int imagecounter;
			int skeletonPixelMatrix    [ ORG_IMAGE_HEIGHT ][ ORG_IMAGE_WIDTH ];
// 			int *skeletonPixelMatrix;
			int MaximunNodeNum;

			
	        /**
		* Constructor.
		* The constructor will initialize all the necesary variables 
		*/
		FrameGrabber():
		               weightedWhiteValues(ORG_IMAGE_HEIGHT * ORG_IMAGE_WIDTH,0)
			       {
			       
			      W = params.camera.width->get();
			      H = params.camera.height->get();
			      siX = params.camera.widthUnDistortion->get();
			      siY = params.camera.heightUnDistortion->get();
			       imagecounter = 0;
			       m_Top = 0;
			       

			       rawHSV = Mat::zeros(H,W, CV_8UC1);
			       GreenBinary = Mat::zeros(H,W, CV_8UC1);
			       fieldConvexHullMat = Mat::zeros(H,W, CV_8UC1);
			       Brightness_Channel = Mat::zeros(H,W, CV_8UC1);
			       binaryImgs[BALL_C]= Mat::zeros(H,W, CV_8UC1);
			       binaryImgs[GOAL_C]= Mat::zeros(H,W, CV_8UC1);
			       binaryImgs[BLACK_C]= Mat::zeros(H,W, CV_8UC1);
			       MaximunNodeNum =0;
// 			       skeletonPixelMatrix=  Mat::zeros( cv::Size(W,H) , CV_32F);
		}
		
		/**
		* Destructor.
		* The destructor will clean all the vectors that were used during the soccer vision execution.
		*/		
		~FrameGrabber(){

			weightedWhiteValues.clear();
			ObstacleConvexHull.clear();

		}
		

	};
	
}	
#endif
