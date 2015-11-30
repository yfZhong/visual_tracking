//Camera.h
// Created on: Apr 19, 2015
//     Author: Hafez Farazi <farazi@ais.uni-bonn.de>
// #pragma once
#ifndef CAMERA_H
#define CAMERA_H

#include <opencv2/opencv.hpp>
#include <sys/ioctl.h>
#include <linux/videodev2.h>
#include <stdio.h>
#include <fcntl.h>
#include <ros/ros.h>
#include <visual_tracking/Parameters/Parameters.h>
#include "MatPublisher.h"
#include "ICamera.h"
using namespace cv;

class Camera:public ICamera
{
private:
	VideoCapture *cap;
	char devStr[255];
	char paramStr[512];
	char paramDefStr[512];
	int devNumber;
	MatPublisher takenImg_pub;
public:
	inline Camera():takenImg_pub("/vision/takenImg")
	{
		devNumber=params.camera.devNumber->get();
		sprintf(devStr, "/dev/video%d", devNumber);
		sprintf(paramStr,"v4l2ctrl -d /dev/video%d -l /nimbro/share/launch/config/vision/logitechConfig.txt ",devNumber);
		sprintf(paramDefStr,"v4l2ctrl -d /dev/video%d -l /nimbro/share/launch/config/vision/logitechConfig_default.txt ",devNumber);
		cap = new VideoCapture(devNumber); // open the camera
// 		cap->set(CV_CAP_PROP_MODE, 3 ); //CV_CAP_MODE_YUYV
// 		std::cout<<"CV_CAP_PROP_MODE   "<<cap->get(CV_CAP_PROP_MODE )<<std::endl;
// 	        std::cout<<std::endl;
	}
	inline virtual ~Camera()
	{
		// the camera will be deinitialized
		delete cap;
	}
	inline bool IsReady()
	{
		return true;
	}
	inline bool IsDummy()
	{
		return false;
	}
	bool InitCameraDevice(bool);
	double TakeCapture();
};

#endif