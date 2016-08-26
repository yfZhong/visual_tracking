/*

Authors: Yongfeng

*/
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
#include <ros/ros.h>

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

#include <visual_tracking/IcpPointMatcher/kdtree.h>
// #include <visual_tracking/LineMatcher/findNodes.h>
#include <gait_msgs/GaitOdom.h>
#include <robotcontrol/RobotHeading.h>


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
		

		std::vector<cv::Point> detectedPoins,undistortedDetectedPoints;
		std::vector<pair<cv::Point, int> > detectedPoinsWithType;
		

// 		vector< cv::Rect > squars_rect;
		std::vector<cv::Point> pure_detectedPoins,pure_undistortedDetectedPoints;
		
		typedef std::vector< Line > Line_Buffer;
		Line_Buffer LinesOnImg;
		Line_Buffer LinesOnImg_After_Merged;
		float MeasConf;
		
		
		void findBoundingRects(FrameGrabber & CamFrm);
		void applyLineFilter(/*in*/cv:: Mat& Brightness_Channel,/* in */cv:: Mat & fieldConvexHull,/*in*/cv:: Mat & greenBinary,/*out*/float ( & weightedWhiteValues )[ H ][ W ]  );
		void RetrieveSkeleton (/* in */cv:: Mat &fieldConvectHull, /* in */ const float ( & weightedWhiteValues )[ H ][ W ]  , /* out */ std::vector<cv::Point> &detectedPoins);
		
		void removeObstacleBoarder (/* in */ vector< vector <cv::Point > > ObstacleContours  ,/* in_out */ std::vector<pair<cv::Point, int> > & _detectedPoinsWithType );
		
		void regionGlow(const float ( & matrix )[ H ][ W ] , Mat & visited , cv:: Mat & _fieldConvexHull, std::vector<pair<cv::Point, int> > & _detectedPoinsWithType );
		void regionGlow(const float ( & matrix )[ H ][ W ] , Mat & visited , cv:: Mat & _fieldConvexHull, std::vector<cv::Point> & detectedPoins);
		Mat skeletonMatrix_tmp ;
		std::queue<Point> neighbors;
		
		

		
		vector< pair<cv::Point, int> > squars;
		Rectangle_Buffer Rectangles;
		
		void findSquares(FrameGrabber & CamFrm);
		void findRectangle_Buffer();
	        bool SortbyK(pair<cv::Point, int> & a, pair<cv::Point, int> & b);
		
		
		// ****************************************************************************** //
                void Smooth ( /* in_out */ float ( & matrix )[ H ][ W ]  );
		void push_back_neighbors(int i, int j);
		bool SortFuncDescending(cv::Point i, cv::Point j);
		
	private:
	        bool debug_line_detector;
		int m_Top;
	
		
	
		double fpsData;
		

		int count;

		int num_change_min_ske;
		int delta1;
		int mean_b;
		int min_skele_shift;
		
		
		
	};


}
#endif