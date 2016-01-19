//DistortionModel.hpp
// Created on: May 12, 2015
//     Author: Hafez Farazi <farazi@ais.uni-bonn.de>
#ifndef DISTORTIONMODEL_H
#define DISTORTIONMODEL_H
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <ros/ros.h>
#include <visual_tracking/Parameters/Parameters.h>

using namespace cv;

class DistortionModel
{
private:
  
  
	void undistortP_slow(const vector<Point> contour, vector<Point> &resCountour);
	void ModifoedOpenCVUndistortPoint( const CvMat* _src, CvMat* _dst, const CvMat* _cameraMatrix,
	                   const CvMat* _distCoeffs,
	                   const CvMat* matR, const CvMat* matP );

	void ModifoedOpenCVUndistortPoint( InputArray _src, OutputArray _dst,
	                          InputArray _cameraMatrix,
	                          InputArray _distCoeffs,
	                          InputArray _Rmat,
	                          InputArray _Pmat );
	Mat distortionModel;
	

	bool undistortP_normalized_slow(const vector<Point> contour,
			vector<Point2f> &resCountour);
	bool distortP_normalized_slow(const vector<Point3f> contour,
			vector<Point2f> &resCountour);
	
	double Radian2Degree(double r);
	
	double GetDistance(Point2d p);

public:
	float getDiagonalAngleView();
	Mat cameraMatrix, distCoeffs;
	Mat unDistortionModel;
	bool Init();
	void unDistortionModelInit();
	void CreateUndistort(const Mat &rawImg,Mat &res);
	void CreateUndistortFull(const Mat &rawImg,Mat &res);
	bool UndistortP(const vector<Point> contour, vector<Point> &resCountour);
	bool UndistortP( Point pIn,Point &pOut);

	bool DistortP(const vector<Point> contour,
			vector<Point> &resCountour);
	bool DistortPFull(const vector<Point> contour,
			vector<Point> &resCountour);
};

extern DistortionModel distortionModel;

#endif