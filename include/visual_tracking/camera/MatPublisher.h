//MatPublisher.h
// Created on: Apr 21, 2015
//     Author: Hafez Farazi <farazi@ais.uni-bonn.de>
// #pragma once
#ifndef MATPUBLISHER_H
#define MATPUBLISHER_H


#include <ros/ros.h>
#include <std_msgs/String.h>
#include <ros/console.h>
#include <opencv2/opencv.hpp>
#include <sstream>
#include <boost/timer/timer.hpp>
#include <inttypes.h>
#include <time.h>
#include <std_msgs/Int64.h>
#include <string.h>
#include <image_transport/image_transport.h>
#include <image_geometry/pinhole_camera_model.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/distortion_models.h>
#include <sys/ioctl.h>
#include <linux/videodev2.h>
#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
// #include <vision_module/Tools/LineSegment.hpp>
// #include <vision_module/Tools/General.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
// #include <config_server/parameter.h>
#include <std_srvs/Empty.h>
#include <boost/format.hpp>

using namespace cv;
using namespace std;
class MatPublisher
{
private:
	ros::NodeHandle nodeHandle;
	cv_bridge::CvImage msg;
	image_transport::ImageTransport it;
	image_transport::Publisher img_pub;
	static int number;
	const int currentNumber;
	bool enable;
public:
	enum imgColorType
	{
		gray, hsv, bgr, yuyv
	};
	inline MatPublisher(string name = "", bool _enable = true) :
			it(nodeHandle), currentNumber(number++), enable(_enable)
	{
		if (!enable)
			return;
		char cName[255];
		if (name.length() < 1)
		{
			sprintf(cName, "/vision/debug/img%d", currentNumber);
		}
		else
		{
			sprintf(cName, "%s", name.c_str());
		}
		img_pub = it.advertise(cName, 1);
		msg.header.frame_id = "/ego_floor";

	}
	inline void publish(Mat img, imgColorType t, ros::Time now =
			ros::Time::now())
	{
		if (!enable || img_pub.getNumSubscribers() < 1)
			return;
		Mat res;
		switch (t)
		{
		case gray:
			msg.encoding = sensor_msgs::image_encodings::MONO8;
			res = img;
			break;
		case hsv:
			msg.encoding = sensor_msgs::image_encodings::BGR8;
			cvtColor(img, res, CV_HSV2BGR);
			break;
			
		case bgr:
			msg.encoding = sensor_msgs::image_encodings::BGR8;
			res = img;
			break;
		case yuyv:	
		default:
			msg.encoding = sensor_msgs::image_encodings::YUV422;
			Mat tmpI;
			cvtColor(img, tmpI, CV_BGR2YCrCb);
			
			res.create(tmpI.rows ,tmpI.cols,CV_8UC2);
//                         unsigned char yuyvImg[ tmpI.rows * tmpI.cols * 2];
			int index = 0;
			for (int i = 0; i < (tmpI.rows * tmpI.cols * 3); i += 6)
			{

				int Y1 = (unsigned char) tmpI.data[i];
				int U = (unsigned char) tmpI.data[i + 1];
				int V = (unsigned char) tmpI.data[i + 2];
				int Y2 = (unsigned char) tmpI.data[i + 3];

				//Storing the YUYV data
				res.data[index++] = Y1;
				res.data[index++] = U;
				res.data[index++] = Y2;
				res.data[index++] = V;
			}

			break;
		}
		msg.header.stamp = now;
		msg.image = res;
		img_pub.publish(msg.toImageMsg());
	}

	inline bool thereAreListeners()
	{
		return (img_pub.getNumSubscribers() > 0);
	}
};


#endif
