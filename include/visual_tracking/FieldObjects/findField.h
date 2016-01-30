#ifndef FINDFIELD_H
#define FINDFIELD_H


#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <algorithm>    
#include <vector>
#include <math.h>

#include <visual_tracking/frameGrabber.h>
#include <visual_tracking/Parameters/Parameters.h>
#include <visual_tracking/Projection/DistortionModel.h>
#include <visual_tracking/Tools/tools.h>
#include <visual_tracking/Tools/m_Math.h>

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace std;
using namespace vision;
using namespace cv;

namespace vision
{
  //by Hafez
  class LineSegment
	{
	public:
	  cv::Point2f P1,P2;
	  LineSegment();
	  LineSegment(cv::Point2f P1_,cv::Point2f P2_): P1(P1_), P2(P2_){};
	  cv::Point2f GetMiddle();
	  bool SortbyDistance(const Point2f & a, const Point2f &b);
	  vector<Point2f> GetMidPoints( int count /*Means that lines count will be 2^count*/);
	};
    
  
  
  class FindField
	{
	  public: 
	    FindField();
	    ~FindField(){};
	    

	    int MAX_DESTANCE_FROM_BOTTOM_OF_IMAGE;
	    
	    bool FindFieldConvexHull(/* in */ cv::Mat &GreenBinary, /* out */Mat &fieldConvectHull,vector<cv::Point> &fieldConvexHullPoints,/* out */int&  m_Top );
	   
	    void ColorClassification(const Mat &srcHsvImg, const Mat &tmplateGrayImg,Mat *dstGrayImgs, hsvRangeC *ranges, bool *inTemplate, int size);
            
	    
// 	    vector<cv::Point> fieldConvexHullPoints;
	    
	    vector<cv::Point>  fieldConvexHullPointsUndistort;

	private:
	    int  m_Top;
	    
	    unsigned int H, W;
	    int siX, siY;

// 	    void bresenham(/* int */ int x1, int y1, int x2, int y2, /* out */std::vector< int > & m_FieldBoundary );
	    
	    
	    bool SortFuncDescending(vector<cv::Point> i, vector<cv::Point> j);

	    int Bottom(Rect rec);
	  
	};
  
  
  
}





#endif