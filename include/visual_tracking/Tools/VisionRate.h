//DistotionModel.cpp
// Created on: April 12, 2014
//    

//Authors: Yongfeng
// #pragma once

#include <boost/asio/steady_timer.hpp>
#include <boost/asio/deadline_timer.hpp>

// #define SECONDTONANO  1000000000
class VisionRate
{
public:
	VisionRate(double rate,bool _steay=true);
	virtual ~VisionRate();
	void sleep();

private:
	boost::asio::io_service io_service;
	boost::asio::steady_timer timer;
	boost::asio::deadline_timer timer2;
	int timeTowait;
	bool steady;
};

