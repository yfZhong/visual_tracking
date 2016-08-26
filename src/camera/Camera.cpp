//DistotionModel.cpp
// Created on: April 12, 2014
//    

//Authors: Yongfeng
#include <visual_tracking/camera/Camera.h>

bool Camera::InitCameraDevice(bool firstTime)
{
	if (!firstTime)
	{
		delete cap;
		cap = new VideoCapture(devNumber); // open the camera
// 		cap->set(CV_CAP_PROP_MODE, 3 ); //CV_CAP_MODE_YUYV
// 		std::cout<<"CV_CAP_PROP_MODE   "<<cap->get(CV_CAP_PROP_MODE )<<std::endl;
// 	        std::cout<<std::endl;
	}
	if (!cap->isOpened())  // check if we succeeded
	{
		ROS_WARN_THROTTLE(10, "Cant Open Camera Device!");
		return false;
	}

	if (cap->set(CV_CAP_PROP_FRAME_WIDTH, params.camera.width->get())
			| cap->set(CV_CAP_PROP_FRAME_HEIGHT, params.camera.height->get()))
	{
		ROS_WARN_THROTTLE(10, "Cant set image size values!");
	}


//The following if, is a hacky thing because some times some logitechConfig.txt made the capture too long so "select timeout" appears!
//	if (-1 == system(paramDefStr))
//	{
//		ROS_WARN_THROTTLE(10, "Cant find or set v4l2-ctrl values!");
//	}

	if (-1 == system(paramStr))
	{
		ROS_WARN_THROTTLE(10, "Cant find or set v4l2-ctrl values!");
	}


	return true;
}

double Camera::TakeCapture()
{
	if (params.camera.devNumber->get() != devNumber)
	{
		devNumber = params.camera.devNumber->get();
		sprintf(devStr, "/dev/video%d", devNumber);
		sprintf(paramStr,"v4l2ctrl -d /dev/video%d -l /nimbro/share/launch/config/vision/logitechConfig.txt",devNumber);
		sprintf(paramDefStr,"v4l2ctrl -d /dev/video%d -l /nimbro/share/launch/config/vision/logitechConfig_default.txt ",devNumber);
		usleep(1000000); //To make sure that camFd is closed!
		InitCameraDevice(false); // Re Init camera
	}

	int camFd = open(devStr, O_RDONLY); //Just for check wheter the camera is presented or not
	if (camFd != -1)
	{
		close(camFd); //Camera is ok!
	}
	else
	{
		usleep(1000000); //To make sure that camFd is closed!
		InitCameraDevice(false); // Re Init camera
		return -1;
	}
        captureTime=ros::Time::now();
	*cap >> rawImage;
	
	if (rawImage.empty())
	{
		ROS_WARN_THROTTLE(10, "Failed to get capture!");
		return -1;
	}
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
	takenImg_pub.publish(rawImage,MatPublisher::bgr);
	return 1; // TODO: it should corespond with gyro!
}

