
#ifndef FINDWHITEOBJECT_H
#define FINDWHITEOBJECT_H

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

class FindWhiteObjects
{
public:

  
  	bool SortFuncDescending(vector<cv::Point> i, vector<cv::Point> j){
		return contourArea(i, false) > contourArea(j, false);
	}

	vector<cv::Point> GetwhiteSegments(Mat & Brightness_Channel, const Mat &tmplateGrayImg,  vector< vector <cv::Point > > &resInRawContours)
	{
		const int minArea = params.obstacle.minArea->get();
		vector<cv::Point>  res;
		vector<vector<cv::Point> > obstContours;
                cv::Mat binary = Mat::zeros(Brightness_Channel.rows ,Brightness_Channel.cols, CV_8UC1);
		
// 		cv::threshold(binary, binary, params.line.MinBrightnessValue->get(), 255,THRESH_BINARY );
// 			cv::imshow("white binary",binary);
			
		const int srcSize = Brightness_Channel.rows * Brightness_Channel.cols;
		
		uchar* tmplateGrayImg_D = tmplateGrayImg.data;
		uchar* brightness = Brightness_Channel.data;
		
		int idx =0;
		for (int i = 0; i < srcSize; i++)
		{
		    if (tmplateGrayImg_D[0] >= 254 && brightness[0] >  params.line.MinBrightnessValue->get())
		    {
		      binary.data[idx] = 255;
		    }
		   tmplateGrayImg_D += 1;
		   idx ++;
		   brightness +=1;
		}
			
			
			
	cv::imshow("white binary",binary);		
// 	imwrite( "/home/yvonne/Desktop/pic/field/GreenBinary.jpg", GreenBinary );
	cv::waitKey(1);
		
		
		cv::findContours(binary/*To have binaryFrame after this function*/,
			obstContours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);

		std::sort(obstContours.begin(), obstContours.end(), bind(&FindWhiteObjects::SortFuncDescending, this, _1, _2));

		for (size_t i = 0; i < obstContours.size(); i++) // iterate through each contour.
		{
			
			double area = contourArea(obstContours[i]);
			if (area < minArea)continue;
			
			vector<cv::Point> tmpContour = obstContours[i];
// 			cv::approxPolyDP(tmpContour, tmpContour, cv::arcLength(tmpContour, true) * 0.003, true);
// 			cv::convexHull(tmpContour,tmpContour, false);
			resInRawContours.push_back(tmpContour);
			
// 			cv::Rect rec = boundingRect(obstContours[i]);
// 			cv::Point btnPoint(rec.x + rec.width / 2, rec.y + rec.height);
// 			cv::Point topPoint(rec.x + rec.width / 2, rec.y);
// 			res.push_back(btnPoint);
		}

		return res;
	}



// 	inline bool Init(){return true;};
};

#endif