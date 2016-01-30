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
			
// 			int H, W, siX, siY;
			enum{
			    H = ORG_IMAGE_HEIGHT,
			    W = ORG_IMAGE_WIDTH,
			    siX = UNDISTOTED_IMAGE_WIDTH, 
			    siY = UNDISTOTED_IMAGE_HEIGHT,
			    subW = SUB_SAMPLING_WIDTH,
			    subH = SUB_SAMPLING_HEIGHT,
			
		        };
			
// 			std::vector<float > weightedWhiteValues;
			float  weightedWhiteValues  [ H ][ W ];
                        
			ForwardProjection forw_Proj;
			
			
			Mat rawHSV;
			Mat Brightness_Channel;
			Mat GreenBinary;
			
			Mat fieldConvexHullMat;
			vector<cv::Point> fieldConvexHullPoints;
			vector< vector <cv::Point > > ObstacleConvexHull;
			vector< vector <cv::Point > > WhiteObjectConvexHull;
			
			int m_Top;
	
			unsigned int imagecounter;
// 			int **skeletonPixelMatrix;
			int **skeletonPixelMatrix_sub;
			int MaximunNodeNum;

			
	        /**
		* Constructor.
		* The constructor will initialize all the necesary variables 
		*/
		FrameGrabber() {

			       imagecounter = 0;
			       m_Top = 0;

			       rawHSV = Mat::zeros(H,W, CV_8UC1);
			       
			       fieldConvexHullMat = Mat::zeros(H,W, CV_8UC1);
			       Brightness_Channel = Mat::zeros(H,W, CV_8UC1);
			       GreenBinary = Mat::zeros(H,W, CV_8UC1);
			      
			       MaximunNodeNum =0;			       
			       
// 			       skeletonPixelMatrix = new int*[siY];
// 			       for(int j=0;j<siY;j++)
// 			       {
// 			       skeletonPixelMatrix[j] = new int[siX]();        
// 			       }
			       
			       
			       skeletonPixelMatrix_sub = new int*[subH];
			       for(int j=0;j<subH;j++)
			       {
			       skeletonPixelMatrix_sub[j] = new int[subW]();        
			       }
			       
			       
		}
		
		/**
		* Destructor.
		* The destructor will clean all the vectors that were used during the soccer vision execution.
		*/		
		~FrameGrabber(){
		        fieldConvexHullPoints.clear();
			ObstacleConvexHull.clear();
	
			
// 			for (int i=0;i<siY;i++)
// 			{
// 			delete[] skeletonPixelMatrix[i]; 
// 			}                     
// 			delete[] skeletonPixelMatrix; 
			
			
			for (int i=0;i<subH;i++)
			{
			delete[] skeletonPixelMatrix_sub[i]; 
			}                     
			delete[] skeletonPixelMatrix_sub; 
			

		}
		

	};
	
}	
#endif
