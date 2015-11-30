#ifndef FINDLINES_H
#define FINDLINES_H

#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <algorithm>    // std::min_element, std::max_element
#include <vector>
#include <math.h>
#include <boost/timer/timer.hpp>
#include <Eigen/Dense>
#include <Eigen/Eigenvalues>
#include <queue>
#include <utility>      // std::pair

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/eigen.hpp>

#include <limits>

#include <visual_tracking/frameGrabber.h>
#include <visual_tracking/Parameters/globaldefinitions.h>
#include <visual_tracking/rgb_yuyv/yuv2rgb.h>
#include <visual_tracking/rgb_yuyv/yuvClasses.h>
#include <visual_tracking/Tools/tools.h>
#include <visual_tracking/Projection/DistortionModel.h>
#include <visual_tracking/Projection/BackwardProjection.h>
#include <visual_tracking/Tools/m_Math.h>


using namespace std;
using namespace vision;
using namespace cv;



namespace vision
{        
        
	class FindLines
	{
	  
	
	public: 
	  	FindLines();
		~FindLines();
		
		enum{
		        H = ORG_IMAGE_HEIGHT,
			W = ORG_IMAGE_WIDTH,
			siX = UNDISTOTED_IMAGE_WIDTH, 
			siY = UNDISTOTED_IMAGE_HEIGHT,
			
		};
		
		
		float DIAGONAL_ANGLE_RNAGE ;
		 // * Line Structure * //

		
		std::vector<cv::Point> detectedPoins,undistortedDetectedPoints;
		std::vector<pair<cv::Point, int> > detectedPoinsWithType;
		
		typedef std::vector< Line > Line_Buffer;
		Line_Buffer LinesOnImg;
		Line_Buffer LinesOnImg_After_Merged;
		float MeasConf;
		
		
// 		int H_data  [ H ][ W ];
// 		int V_data  [ H ][ W ];
// 		int diagonal_data45  [ H ][ W ];
// 		int diagonal_data135  [ H ][ W ];
		
		
		void findSkeletons(FrameGrabber & CamFrm);
// 		void applyLineFilter(/*in*/cv:: Mat Brightness_Channel,/*out*/std::vector< float > & weightedWhiteValues);
		void applyLineFilter(/*in*/cv:: Mat Brightness_Channel,/*out*/float ( & weightedWhiteValues )[ H ][ W ]  );
		void RetrieveSkeleton (/* in */cv:: Mat &fieldConvectHull, /* in */ const float ( & weightedWhiteValues )[ H ][ W ]  , /* out */ std::vector<cv::Point> &detectedPoins);
		void removeObstacleBoarder (/* in */ vector< vector <cv::Point > > ObstacleContours  ,/* in_out */ std::vector<pair<cv::Point, int> > & _detectedPoinsWithType );
		
		void regionGlow(const float ( & matrix )[ H ][ W ] , Mat & visited, cv:: Mat & _fieldConvexHull, std::vector<pair<cv::Point, int> > & _detectedPoinsWithType );
		
		Mat visited ;
		std::queue<Point> neighbors;
		void push_back_neighbors(int i, int j);
		

		
		
		// ****************************************************************************** //
                void Smooth ( /* in_out */ float ( & matrix )[ H ][ W ]  );
	
		void findLines(FrameGrabber & CamFrm);
		void findhoughLines(std::vector<cv::Point> & undistortedPoints);
		
	private:
	        bool debug_line_detector;
		int m_Top;
		
		void MergeLinesOnImg(std::vector< Line > m_LineBuffer_Before_Merge, std::vector< Line >& m_LineBuffer_After_Merge);
		Line MergeTwoLinesOnImg(Line line1, Line line2, int id);
		float getMeasurementConvexhullArea(std::vector< Line >& Lines);
		float getMeasurementConfidence(std::vector< Line >& Lines);
	
		
	
		double fpsData;
	       
	};


}
#endif