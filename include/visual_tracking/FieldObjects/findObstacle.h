
#ifndef FINDOBSTACLe_H
#define FINDOBSTACLe_H

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

#include <visual_tracking/Parameters/Parameters.h>

using namespace cv;

class FindObstacle
{
public:

  
  	bool SortFuncDescending(vector<cv::Point> i, vector<cv::Point> j){
		return contourArea(i, false) > contourArea(j, false);
	}

	vector<cv::Point> GetObstacleContours(Mat & obstacleMask,vector< vector <cv::Point > > &resInRawContours)
	{
		const int minArea = params.obstacle.minArea->get();
		vector<cv::Point>  res;
		vector<vector<cv::Point> > obstContours;

		cv::findContours(obstacleMask.clone()/*To have binaryFrame after this function*/,
			obstContours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);

		std::sort(obstContours.begin(), obstContours.end(), bind(&FindObstacle::SortFuncDescending, this, _1, _2));

		for (size_t i = 0; i < obstContours.size(); i++) // iterate through each contour.
		{
			cv::Rect rec = boundingRect(obstContours[i]);
			double area = contourArea(obstContours[i]);
			cv::Point btnPoint(rec.x + rec.width / 2, rec.y + rec.height);

			cv::Point topPoint(rec.x + rec.width / 2, rec.y);
			if (area < minArea)
				continue;

			vector<cv::Point> tmpContour = obstContours[i];
			cv::approxPolyDP(tmpContour, tmpContour, cv::arcLength(tmpContour, true) * 0.003, true);
			cv::convexHull(tmpContour,tmpContour, false);
			
			resInRawContours.push_back(tmpContour);
			res.push_back(btnPoint);
		}

		return res;
	}



// 	inline bool Init(){return true;};
};

#endif