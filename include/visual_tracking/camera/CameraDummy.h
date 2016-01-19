//Cameradummy.h
// Created on: Apr 19, 2015
//     Author: Hafez Farazi <farazi@ais.uni-bonn.de>
// #pragma once
#ifndef CAMERADUMMY_H
#define CAMERADUMMY_H
#include <opencv2/opencv.hpp>
#include <sys/ioctl.h>
#include <stdio.h>
#include <fcntl.h>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <visual_tracking/Parameters/Parameters.h>
#include "MatPublisher.h"
#include "ICamera.h"

using namespace cv;
using namespace boost::timer;

class CameraDummy:public ICamera
{
private:
	ros::NodeHandle nodeHandle;
	image_transport::Subscriber sub;
	image_transport::ImageTransport it_;
// 	image_transport::Publisher yuyv_pub;
	int capNumber;
public:
	inline CameraDummy() :
			it_(nodeHandle), capNumber(0)
	{
		sub = it_.subscribe("/vision/takenImg", 1, &CameraDummy::imageCallback,this);
// 		yuyv_pub = it_.advertise("/vision/yuyvImg", 1);
	}
	inline virtual ~CameraDummy()
	{
	}
	void imageCallback(const sensor_msgs::ImageConstPtr& msg)
	{
	  try
		{
			cv_bridge::CvImagePtr cv_ptr;
// 			
// // 			cout<<msg->encoding<<endl;
			cv_ptr = cv_bridge::toCvCopy(msg,
					sensor_msgs::image_encodings::BGR8);
			rawImage = cv_ptr->image.clone();
// 
                        captureTime = msg->header.stamp;
			if (params.camera.flipHor->get() && params.camera.flipVer->get())
			{
				flip(rawImage, rawImage, -1);
			}
			else
			{
				if (params.camera.flipVer->get())
				{
					flip(rawImage, rawImage, 0);
				}
				else if (params.camera.flipHor->get())
				{
					flip(rawImage, rawImage, 1);
				}
			}
// 			cout<<"captureNum  "<<capNumber<<endl;
			capNumber++;

// 			Mat tmpI;

// 			cvtColor(rawImage, tmpI, CV_BGR2YCrCb);
//                         unsigned char yuyvImg[tmpI.rows * tmpI.cols * 2];
// 			int index = 0;
// 			for (int i = 0; i < (tmpI.rows * tmpI.cols * 3); i += 6)
// 			{
// 
// 				int Y1 = (unsigned char) tmpI.data[i];
// 				int U = (unsigned char) tmpI.data[i + 1];
// 				int V = (unsigned char) tmpI.data[i + 2];
// 				int Y2 = (unsigned char) tmpI.data[i + 3];
// 
// 				//Storing the YUYV data
// 				yuyvImg[index++] = Y1;
// 				yuyvImg[index++] = U;
// 				yuyvImg[index++] = Y2;
// 				yuyvImg[index++] = V;
// 			}
// 			
// 			cv::Mat yuyv(tmpI.rows, tmpI.cols, CV_8UC2, yuyvImg);
// 			cv_ptr->image =  yuyv;
// 			cv_ptr->encoding =std::string("YUYV");
// 			yuyv_pub.publish(cv_ptr->toImageMsg());
			
		} catch (cv_bridge::Exception& e)
		{
			ROS_ERROR("cv_bridge exception: %s", e.what());
			return;
		}

	}
	inline bool IsReady()
	{
		return capNumber>1;
	}
	inline bool IsDummy()
	{
		return true;
	}
	bool InitCameraDevice(bool);
	double TakeCapture();
};

#endif
