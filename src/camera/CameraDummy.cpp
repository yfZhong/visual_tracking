
// Created on: April 12, 2014
//    

//Authors: Yongfeng
#include <visual_tracking/camera/CameraDummy.h>
bool CameraDummy::InitCameraDevice(bool first)
{
	return true;
}

double CameraDummy::TakeCapture()
{
	if(capNumber <=1)
	{
		return -1;
	}

	if (rawImage.empty())
	{
		ROS_WARN_THROTTLE(10, "Failed to get capture!");
		return -1;
	}

	return 1; // TODO: it should corespond with gyro!
}

