//ICamera.h
// Created on: Apr 19, 2015
//     Author: Hafez Farazi <farazi@ais.uni-bonn.de>
// #pragma once

#ifndef ICAMERA_H
#define ICAMERA_H

#include <opencv2/opencv.hpp>
#include <sys/ioctl.h>
#include <linux/videodev2.h>
#include <stdio.h>
#include <fcntl.h>
#include <ros/ros.h>
#include <visual_tracking/Parameters/Parameters.h>
#include "MatPublisher.h"

using namespace cv;

class ICamera
{
public:
	Mat rawImage;
	virtual ~ICamera(){};
	virtual bool IsDummy()=0;
	virtual bool IsReady()=0;
	virtual bool InitCameraDevice(bool)=0;
	virtual double TakeCapture()=0;
	ros::Time captureTime;
};

#endif