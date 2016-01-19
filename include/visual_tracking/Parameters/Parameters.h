//Parameters.hpp
// Created on: Apr 20, 2015
//     Author: Hafez Farazi <farazi@ais.uni-bonn.de>
// #pragma once

#ifndef PARAMETERS_H
#define PARAMETERS_H

#include <config_server/parameter.h>
#include <string.h>
#include <opencv2/opencv.hpp>
#include <math.h>

// #define COLORED_OBJECT_COUNT 4
#define IMGWIDTH 640
#define IMGHEIGHT 480
using namespace std;
using namespace cv;

template<class T>
class constParameter
{
private:

public:
	T data;
	constParameter(T in)
	{
		set(in);
	}
	;
	T get()
	{
		return data;
	}
	void set(T in)
	{
		data = in;
	}
};

class LineC
{
public:
  
        config_server::Parameter<float> *Div1;
	config_server::Parameter<float> *Div2;
        config_server::Parameter<int> *MinBrightnessValue;
	config_server::Parameter<int> *MinSkeletonValue;
	config_server::Parameter<int> *LocalOptimalRange;
	config_server::Parameter<int> *ObstacleLineDistance;
	
	config_server::Parameter<int> *HoughLineRho;
	config_server::Parameter<int> *HoughLineVoteThreshold;
	config_server::Parameter<int> *HoughLineMinLineLength;
	config_server::Parameter<int> *HoughLinemaxLineGap;
	config_server::Parameter<int> *HoughLineTheta;
        config_server::Parameter<int> *MaxPointLineDistance;
	
	config_server::Parameter<int> *AngleToMergeImg;
	config_server::Parameter<int> *MinLineSegDistanceImg;
	config_server::Parameter<int> *MinProjectedDistanceImg;
	
	config_server::Parameter<float> *MeasAreaThreshod;
	config_server::Parameter<float> *MeasLineNumThreshod;
	config_server::Parameter<float> *MeasLineLengthThreshod;
	config_server::Parameter<float> *MeasLineMaxAngDifThreshod;
	
	
        config_server::Parameter<int> *AngleToMerge;
	config_server::Parameter<float> *MinLineSegDistance;
	config_server::Parameter<float> *MinProjectedDistance;
	
	config_server::Parameter<int> *MinLineLength;
	
	

};

class SquareC
{
      public:
	 config_server::Parameter<int> *MaxNeighborDist;
	 config_server::Parameter<int> *Max_K;
	 config_server::Parameter<int> *MinPointNum;
  
  
};


class FieldC
{
public:
        config_server::Parameter<bool> *BonnField;
	config_server::Parameter<int> *MaxDistanceFromBottomOfImage;
	config_server::Parameter<int> *minArea;
	config_server::Parameter<int> *maxContourCount;

};

class GraphNodeC
{
public:
	config_server::Parameter<int> *LineWidth;
	config_server::Parameter<int> *SmallAngle;
	config_server::Parameter<int> *MaxDist;
	config_server::Parameter<int> *MinDistForMerge;
	config_server::Parameter<int> *MergeJointDist;
	config_server::Parameter<int> *MinCmpLineLength;
	config_server::Parameter<int> *Min_adj;
	config_server::Parameter<int> *DistanceCompToMerge;
	config_server::Parameter<int> *AngleToMerge;
	config_server::Parameter<float> *MaxLineSegDistance;
	config_server::Parameter<float> *MaxProjectedDistance;
	config_server::Parameter<float> * MaxDistanceForMergeNode1;
	config_server::Parameter<float> * MaxDistanceForMergeNode2;
	config_server::Parameter<float> *SamplePointDist;
	config_server::Parameter<float> *_NeighborRadius;
	config_server::Parameter<float> *_MaxDistanceForConnect;
	
	config_server::Parameter<int> *_MaxCloseNum;
	config_server::Parameter<int> *_MinAngle;
	config_server::Parameter<int> *_Min_adj;
	config_server::Parameter<int> *_MinAngleForSameComp;
	config_server::Parameter<float> *_MinCmpLineLength;
	config_server::Parameter<float> *_MaxLineSegDistance;
	config_server::Parameter<float> *_MaxProjectedDistance;
	config_server::Parameter<int> *_MaxAngleDiff;
	
	config_server::Parameter<int> *_AvgAngleDiff;
	config_server::Parameter<float> *	_MaxAvgProjectedDistance;
	config_server::Parameter<float> *	_MaxDistanceForMerge;
	

	

};


class cameraC
{
public:
// 	config_server::Parameter<float> *aspW;
// 	constParameter<float> *aspH;
	config_server::Parameter<bool> *flipVer;
	config_server::Parameter<bool> *flipHor;
	config_server::Parameter<int> *devNumber;
	constParameter<int> *width;
	constParameter<int> *height;
	constParameter<int> *widthUnDistortion;
	constParameter<int> *heightUnDistortion;
	config_server::Parameter<bool> *useBagfile;
// 	config_server::Parameter<float> *diagonalAngleView;
};


class vectorC
{
public:
	config_server::Parameter<float> *x;
	config_server::Parameter<float> *y;
	config_server::Parameter<float> *z;
};
// 
// class vectorCC
// {
// public:
// 	constParameter<float> *x;
// 	constParameter<float> *y;
// 	constParameter<float> *z;
// };

class calibC
{
public:
	config_server::Parameter<string> *filePath;
};

class debugC
{
public:
	config_server::Parameter<bool> *showFieldHull;
	config_server::Parameter<bool> *showSkeletonPixels;
	config_server::Parameter<bool> *showNodeGraph;
	config_server::Parameter<bool> *showTangentLine;
	config_server::Parameter<bool> *showBoundingRects;
	config_server::Parameter<bool> *showModelLines;
	config_server::Parameter<bool> *showHoughLines;
	config_server::Parameter<bool> *showMergedLines;
        config_server::Parameter<bool> *showAssociateLines;
        config_server::Parameter<bool> *showCamPose;
	config_server::Parameter<bool> *showRobPose;
	config_server::Parameter<bool> *publishTime;	
	config_server::Parameter<bool> *showLineInfo;
	config_server::Parameter<bool> *showFieldModel;
	config_server::Parameter<bool> *showCorrespondence;
	config_server::Parameter<bool> *useKalmanFilter;
	config_server::Parameter<bool> *useMotionOdom;
};

// class CameraCalibratorC
// {
// public:
// 	vector<Point> clicked;
// 	vector<Point2f> rvizClicked;
// 	vector<Point3d> cameraLocation, opticalAngle;
// 	bool IsReady()
// 	{
// 		return (rvizClicked.size() == clicked.size()
// 				&& clicked.size() == opticalAngle.size()
// 				&& opticalAngle.size() == cameraLocation.size()
// 				&& rvizClicked.size() > 0);
// 	}
// };

class hsvRangeC
{
public:
	config_server::Parameter<int> *h0;
	config_server::Parameter<int> *h1;
	config_server::Parameter<int> *s0;
	config_server::Parameter<int> *s1;
	config_server::Parameter<int> *v0;
	config_server::Parameter<int> *v1;

};


class GoalC
{
public:
  
	config_server::Parameter<int> *lineVoteDouble;
	config_server::Parameter<int> *erode;
	config_server::Parameter<int> *dilate;
	config_server::Parameter<int> *MinLineLength;
	config_server::Parameter<int> *DistanceToMerge;
	config_server::Parameter<int> *MaxOutField;
	config_server::Parameter<int> *jumpMax;
	config_server::Parameter<int> *minDoubleLength;
	config_server::Parameter<int> *minContinuesColor;
	config_server::Parameter<int> *minEdgeValue;
	config_server::Parameter<int> *cannyThreadshold;
	
};

class ObstacleC
{
public:
  
	config_server::Parameter<int> *minArea;
	
};
class IcpC
{
public:
	config_server::Parameter<int> *IcpInlierDistThreshold;
	config_server::Parameter<float> *SamplePointDist_WorldCord;

	config_server::Parameter<float> *AvgDistThresholdForConf;
	config_server::Parameter<float> *InlierNumThresholdForConf;
	config_server::Parameter<float> *HeadingOffset;

};


class projectionC
{
public:
        config_server::Parameter<float> *timeToShift;
};

class DataAssociationC
{
public:
 
	config_server::Parameter<int> *MaxAngDiff;
	config_server::Parameter<float> *MaxErrorForInlier;
	config_server::Parameter<float> *_MinCompLengthForModelError;
	config_server::Parameter<int> *_MaxAngleChange;

	
};



class HillClimbingC
{
public:
        config_server::Parameter<int> *HeadingOffset;
	config_server::Parameter<float> *MovingStepCoefficient;
	config_server::Parameter<int> *MaxIterationNum;
	config_server::Parameter<float> *ErrorThreshold1;
	config_server::Parameter<float> *ErrorThreshold2;
	config_server::Parameter<int> *MaxHypothesisNum;
	

};


class KalmanFilterC
{
public:
 
	config_server::Parameter<int> *ErrorThreshod;
	config_server::Parameter<int> *KFPositionCov;
	config_server::Parameter<int> *KFOrientationCov;
};



class ParticleFilterC
{
public:
 
	config_server::Parameter<int> *ParticleNum;
	config_server::Parameter<float> *RandomSamplePct;
	config_server::Parameter<float> *MinSumOfLineLengthOnImage;
        config_server::Parameter<float> *HeadingOffset;
};


class ParametersC
{
public:
// 	CameraCalibratorC camCalibrator;
        hsvRangeC ballhsv, fieldhsv, linehsv, goalhsv,obstaclehsv;
	LineC line;
	GoalC goal;
	FieldC field;
	ObstacleC obstacle;
	cameraC camera;
	vectorC location, orientation;
	calibC calib;
	debugC debug;
	projectionC projection;
	DataAssociationC dataAssociation;
	HillClimbingC hillclimbing;
	KalmanFilterC kalmanFilter;
	IcpC icp;
	ParticleFilterC particleFilter;
	GraphNodeC graphNode;
	SquareC square;
	void Init()
	{
	  
	        ballhsv.h0 = new config_server::Parameter<int>("/vision/ballhsv/h0", 0, 1, 180, 0);
		ballhsv.h1 = new config_server::Parameter<int>("/vision/ballhsv/h1", 0, 1, 180, 0);
		ballhsv.s0 = new config_server::Parameter<int>("/vision/ballhsv/s0", 0, 1, 255, 0);
		ballhsv.s1 = new config_server::Parameter<int>("/vision/ballhsv/s1", 0, 1, 255, 0);
		ballhsv.v0 = new config_server::Parameter<int>("/vision/ballhsv/v0", 0, 1, 255, 0);
		ballhsv.v1 = new config_server::Parameter<int>("/vision/ballhsv/v1", 0, 1, 255, 0);
	  
	        goalhsv.h0 = new config_server::Parameter<int>("/vision/goalhsv/h0", 0, 1, 180, 0);
		goalhsv.h1 = new config_server::Parameter<int>("/vision/goalhsv/h1", 0, 1, 180, 0);
		goalhsv.s0 = new config_server::Parameter<int>("/vision/goalhsv/s0", 0, 1, 255, 0);
		goalhsv.s1 = new config_server::Parameter<int>("/vision/goalhsv/s1", 0, 1, 255, 0);
		goalhsv.v0 = new config_server::Parameter<int>("/vision/goalhsv/v0", 0, 1, 255, 0);
		goalhsv.v1 = new config_server::Parameter<int>("/vision/goalhsv/v1", 0, 1, 255, 0);
	  
	        obstaclehsv.h0 = new config_server::Parameter<int>("/vision/obstaclehsv/h0", 0, 1, 360, 0);
		obstaclehsv.h1 = new config_server::Parameter<int>("/vision/obstaclehsv/h1", 0, 1, 360, 0);
		obstaclehsv.s0 = new config_server::Parameter<int>("/vision/obstaclehsv/s0", 0, 1, 255, 0);
		obstaclehsv.s1 = new config_server::Parameter<int>("/vision/obstaclehsv/s1", 0, 1, 255, 0);
		obstaclehsv.v0 = new config_server::Parameter<int>("/vision/obstaclehsv/v0", 0, 1, 255, 0);
		obstaclehsv.v1 = new config_server::Parameter<int>("/vision/obstaclehsv/v1", 0, 1, 255, 0);
	        
		linehsv.h0 = new config_server::Parameter<int>("/vision/linehsv/h0", 0, 1, 180, 0);
		linehsv.h1 = new config_server::Parameter<int>("/vision/linehsv/h1", 0, 1, 180, 0);
		linehsv.s0 = new config_server::Parameter<int>("/vision/linehsv/s0", 0, 1, 255, 0);
		linehsv.s1 = new config_server::Parameter<int>("/vision/linehsv/s1", 0, 1, 255, 0);
		linehsv.v0 = new config_server::Parameter<int>("/vision/linehsv/v0", 0, 1, 255, 0);
		linehsv.v1 = new config_server::Parameter<int>("/vision/linehsv/v1", 0, 1, 255, 0);
		
		
		fieldhsv.h0 = new config_server::Parameter<int>("/vision/fieldhsv/h0", 0, 1, 180, 34);
		fieldhsv.h1 = new config_server::Parameter<int>("/vision/fieldhsv/h1", 0, 1, 180, 85);
		fieldhsv.s0 = new config_server::Parameter<int>("/vision/fieldhsv/s0", 0, 1, 255, 60);
		fieldhsv.s1 = new config_server::Parameter<int>("/vision/fieldhsv/s1", 0, 1, 255, 255);
		fieldhsv.v0 = new config_server::Parameter<int>("/vision/fieldhsv/v0", 0, 1, 255, 0);
		fieldhsv.v1 = new config_server::Parameter<int>("/vision/fieldhsv/v1", 0, 1, 255, 0);
		
	  
	        field.BonnField= new config_server::Parameter<bool>("/vision/field/BonnField", true);
		field.MaxDistanceFromBottomOfImage = new config_server::Parameter<int>("/vision/field/MaxDistanceFromBottomOfImage", 0, 5,480, 200);
		field.minArea = new config_server::Parameter<int>("/vision/field/minArea", 0, 1, IMGWIDTH*IMGHEIGHT, 500);
		field.maxContourCount = new config_server::Parameter<int>("/vision/field/maxContourCount", 0, 1, 100, 4);
		
		obstacle.minArea = new config_server::Parameter<int>("/vision/obstacle/minArea", 0, 10, IMGWIDTH*IMGHEIGHT, 500);
	  
		
		line.Div1 = new config_server::Parameter<float>("/vision/line/Div1", 0, 0.1, 1.0, 0.5);
                line.Div2 = new config_server::Parameter<float>("/vision/line/Div2", 0, 0.1, 1.0,  0.8);
	        line.MinBrightnessValue = new config_server::Parameter<int>("/vision/line/MinBrightnessValue", 0, 2, 255,  30);
		
	        line.MinSkeletonValue = new config_server::Parameter<int>("/vision/line/MinSkeletonValue", 0, 2, 255,  30);
		line.LocalOptimalRange = new config_server::Parameter<int>("/vision/line/LocalOptimalRange", 0, 1, 100,  0);
		line.ObstacleLineDistance = new config_server::Parameter<int>("/vision/line/ObstacleLineDistance", -100, 1, 100, -10);
		
		line.HoughLineRho = new config_server::Parameter<int>("/vision/line/HoughLineRho", 1, 1, 20, 4);
		line.HoughLineVoteThreshold = new config_server::Parameter<int>("/vision/line/HoughLineVoteThreshold", 0, 5, 50, 5);
		line.HoughLineMinLineLength = new config_server::Parameter<int>("/vision/line/HoughLineMinLineLength", 0, 5, 100, 30);
		line.HoughLinemaxLineGap = new config_server::Parameter<int>("/vision/line/HoughLinemaxLineGap", 0, 5, 100, 30);
		line.HoughLineTheta = new config_server::Parameter<int>("/vision/line/HoughLineTheta", 0, 1, 90, 4);
		
		line.AngleToMergeImg = new config_server::Parameter<int>("/vision/line/AngleToMergeImg", 0, 1, 90, 10);
		line.MinLineSegDistanceImg = new config_server::Parameter<int>("/vision/line/MinLineSegDistanceImg", 0, 1, 500, 100);
		line.MinProjectedDistanceImg = new config_server::Parameter<int>("/vision/line/MinProjectedDistanceImg", 0, 1, 500, 5);
		
		line.MeasAreaThreshod = new config_server::Parameter<float>("/vision/line/MeasAreaThreshod", 1, 50, 200000, 50000);
		line.MeasLineNumThreshod = new config_server::Parameter<float>("/vision/line/MeasLineNumThreshod", 1, 1, 20, 3);
		line.MeasLineLengthThreshod = new config_server::Parameter<float>("/vision/line/MeasLineLengthThreshod", 10, 10, 20000, 200);
		line.MeasLineMaxAngDifThreshod = new config_server::Parameter<float>("/vision/line/MeasLineMaxAngDifThreshod", 1, 2, 90,60 );

		line.MaxPointLineDistance = new config_server::Parameter<int>("/vision/line/MaxPointLineDistance", 0, 5, 100, 50);
		line.AngleToMerge = new config_server::Parameter<int>("/vision/line/AngleToMerge", 0, 5, 90, 15);
		line.MinLineSegDistance = new config_server::Parameter<float>("/vision/line/MinLineSegDistance", 0, 0.05, 5, 1.0);
		line.MinProjectedDistance = new config_server::Parameter<float>("/vision/line/MinProjectedDistance", 0, 0.05, 5, 0.25);
		
                line.MinLineLength = new config_server::Parameter<int>("/vision/line/MinLineLength", 0, 10, 1000, 100);
		
		
		square.MaxNeighborDist = new config_server::Parameter<int>("/vision/square/MaxNeighborDist", 2, 1, 10, 5);
		square.Max_K = new config_server::Parameter<int>("/vision/square/Max_K", 2, 1, 10, 6);
		square.MinPointNum = new config_server::Parameter<int>("/vision/square/MinPointNum", 0, 1, 100, 3);
		

		
		
		graphNode.LineWidth = new config_server::Parameter<int>("/vision/graphNode/LineWidth", 0, 1, 100, 15);
		graphNode.SmallAngle = new config_server::Parameter<int>("/vision/graphNode/SmallAngle", 0, 1, 100, 60);
		
		graphNode.MaxDist = new config_server::Parameter<int>("/vision/graphNode/MaxDist", 0, 10, 10000, 1800);
		graphNode.MinDistForMerge = new config_server::Parameter<int>("/vision/graphNode/MinDistForMerge", 0, 10, 40000, 200);
		graphNode.MergeJointDist = new config_server::Parameter<int>("/vision/graphNode/MergeJointDist", 0, 10, 10000, 1800);
		graphNode.MinCmpLineLength = new config_server::Parameter<int>("/vision/graphNode/MinCmpLineLength", 0, 10, 40000, 400);
		graphNode.Min_adj = new config_server::Parameter<int>("/vision/graphNode/Min_adj", 5, 10, 10000, 100);
		graphNode.DistanceCompToMerge = new config_server::Parameter<int>("/vision/graphNode/DistanceCompToMerge", 0, 1, 10000, 25);
		graphNode.AngleToMerge = new config_server::Parameter<int>("/vision/graphNode/AngleToMerge", 0, 5, 90, 15);
		graphNode.MaxLineSegDistance = new config_server::Parameter<float>("/vision/graphNode/MaxLineSegDistance", 0, 10, 10000, 100);
		graphNode.MaxProjectedDistance = new config_server::Parameter<float>("/vision/graphNode/MaxProjectedDistance", 0, 1, 1000, 25);
		graphNode.MaxDistanceForMergeNode1 = new config_server::Parameter<float>("/vision/graphNode/MaxDistanceForMergeNode1", 0, 10, 10000, 64);
		graphNode.MaxDistanceForMergeNode2 = new config_server::Parameter<float>("/vision/graphNode/MaxDistanceForMergeNode2", 0, 10, 10000, 64);
		graphNode.SamplePointDist = new config_server::Parameter<float>("/vision/graphNode/SamplePointDist", 0, 1, 100, 5);
		
		graphNode._MaxCloseNum= new config_server::Parameter<int>("/vision/graphNode/_MaxCloseNum", 1, 1,30, 10);
		graphNode._NeighborRadius = new config_server::Parameter<float>("/vision/graphNode/_NeighborRadius", 3, 1,30, 15);
		graphNode._MinAngle = new config_server::Parameter<int>("/vision/graphNode/_MinAngle", 10, 1,90, 45);
		graphNode._Min_adj = new config_server::Parameter<int>("/vision/graphNode/_Min_adj", 1, 10, 10000, 100);
		graphNode._MinAngleForSameComp = new config_server::Parameter<int>("/vision/graphNode/_MinAngleForSameComp", 100, 1, 180, 165);
		graphNode._MaxDistanceForConnect = new config_server::Parameter<float>("/vision/graphNode/_MaxDistanceForConnect", 0, 1, 100, 40);
		graphNode._MinCmpLineLength = new config_server::Parameter<float>("/vision/graphNode/_MinCmpLineLength", 0, 1, 400, 20);
		graphNode._MaxLineSegDistance = new config_server::Parameter<float>("/vision/graphNode/_MaxLineSegDistance", 0, 10, 1000, 100);
		graphNode._MaxProjectedDistance = new config_server::Parameter<float>("/vision/graphNode/_MaxProjectedDistance", 0, 1, 1000, 25);
		graphNode._MaxAngleDiff = new config_server::Parameter<int>("/vision/graphNode/_MaxAngleDiff", 0, 5, 90, 15);
		graphNode._AvgAngleDiff = new config_server::Parameter<int>("/vision/graphNode/_AvgAngleDiff", 0, 5, 90, 15);
		graphNode._MaxAvgProjectedDistance = new config_server::Parameter<float>("/vision/graphNode/_MaxAvgProjectedDistance", 0, 1, 1000, 25);
		graphNode._MaxDistanceForMerge = new config_server::Parameter<float>("/vision/graphNode/_MaxDistanceForMerge", 0, 1, 4000, 80);
		

		dataAssociation.MaxAngDiff = new config_server::Parameter<int>("/vision/dataAssociation/MaxAngDiff", 0, 5, 90, 45);
		dataAssociation.MaxErrorForInlier = new config_server::Parameter<float>("/vision/dataAssociation/MaxErrorForInlier", 0, 10, 10000, 100);
		dataAssociation._MinCompLengthForModelError = new config_server::Parameter<float>("/vision/dataAssociation/_MinCompLengthForModelError", 0, 5, 1000, 100);
		dataAssociation._MaxAngleChange = new config_server::Parameter<int>("/vision/dataAssociation/_MaxAngleChange", 0, 5, 90, 10);
		
		
		hillclimbing.HeadingOffset = new config_server::Parameter<int>("/vision/hillclimbing/HeadingOffset", 0, 1, 1, 0);
		hillclimbing.MovingStepCoefficient= new config_server::Parameter<float>("/vision/hillclimbing/MovingStepCoefficient", 0.0001, 0.1, 2, 1);
		hillclimbing.MaxIterationNum = new config_server::Parameter<int>("/vision/hillclimbing/MaxIterationNum", 0, 1, 20, 5);
		hillclimbing.ErrorThreshold1 = new config_server::Parameter<float>("/vision/hillclimbing/ErrorThreshold1", 1, 1, 200, 5);
		hillclimbing.ErrorThreshold2 = new config_server::Parameter<float>("/vision/hillclimbing/ErrorThreshold2", hillclimbing.ErrorThreshold1->get(), 1, 300, 50);
		hillclimbing.MaxHypothesisNum = new config_server::Parameter<int>("/vision/hillclimbing/MaxHypothesisNum", 1, 1, 200, 20);
		
		
		

		
		
		goal.lineVoteDouble = new config_server::Parameter<int>("/vision/goal/VoteDouble", 0, 1,100 , 40);
		goal.erode = new config_server::Parameter<int>("/vision/goal/erode", 0,1, 20, 0);
		goal.dilate = new config_server::Parameter<int>("/vision/goal/dilate", 0, 1, 20, 0);
		goal.MinLineLength = new config_server::Parameter<int>("/vision/goal/MinLineLength", 1, 1, IMGWIDTH, 40);
		goal.MaxOutField = new config_server::Parameter<int>("/vision/goal/MaxOutField", -40, 1, 40, -10);
		goal.DistanceToMerge = new config_server::Parameter<int>("/vision/goal/DistanceToMerge", 0, 1, 100, 20);
		goal.jumpMax = new config_server::Parameter<int>("/vision/goal/jumpMax", 0, 1, 64, 20);
		goal.minDoubleLength = new config_server::Parameter<int>("/vision/goal/minDoubleLength", 0, 1,480 , 3);
		goal.minContinuesColor= new config_server::Parameter<int>("/vision/goal/minContinuesColor", 0, 1,480 , 10);
		goal.minEdgeValue= new config_server::Parameter<int>("/vision/goal/minEdgeValue", 0, 5, 1000 , 100);
		goal.cannyThreadshold= new config_server::Parameter<int>("/vision/goal/cannyThreadshold", 0, 1, 100 , 21);
		

		
// 		field.MaxContinuousBlank = new config_server::Parameter<int>("/vision/field/MaxContinuousBlank", 0, 1,30, 5);


		camera.width = new constParameter<int>(IMGWIDTH);
		camera.height = new constParameter<int>(IMGHEIGHT);
		camera.widthUnDistortion = new constParameter<int>(0);
		camera.heightUnDistortion = new constParameter<int>(0);
		camera.flipHor = new config_server::Parameter<bool>("/vision/camera/flipHor", false);
		camera.flipVer = new config_server::Parameter<bool>("/vision/camera/flipVer", false);
		camera.devNumber = new config_server::Parameter<int>("/vision/camera/devNumber", 0, 1, 3, 0);
		camera.useBagfile = new config_server::Parameter<bool>("/vision/camera/useBagfile", false);
		
		
                calib.filePath = new config_server::Parameter<string>("/vision/calib/filePath", "/nimbro/share/visual_tracking/config/cCFile.yml");


	        debug.showFieldHull= new config_server::Parameter<bool>("/vision/debug/showFieldHull", true);
		debug.showSkeletonPixels= new config_server::Parameter<bool>("/vision/debug/showSkeletonPixels", true);
		debug.showBoundingRects= new config_server::Parameter<bool>("/vision/debug/showBoundingRects", true);
		debug.showNodeGraph= new config_server::Parameter<bool>("/vision/debug/showNodeGraph", true);
		debug.showTangentLine= new config_server::Parameter<bool>("/vision/debug/showTangentLine", true);
		debug.showModelLines= new config_server::Parameter<bool>("/vision/debug/showModelLines", true);
		
		
		debug.showHoughLines= new config_server::Parameter<bool>("/vision/debug/showHoughLines", true);
		debug.showMergedLines= new config_server::Parameter<bool>("/vision/debug/showMergedLines", true);
		debug.showAssociateLines= new config_server::Parameter<bool>("/vision/debug/showAssociateLines", true);
		debug.showFieldModel= new config_server::Parameter<bool>("/vision/debug/showFieldModel", true);
		debug.showCamPose= new config_server::Parameter<bool>("/vision/debug/showCamPose", true);
		debug.showRobPose= new config_server::Parameter<bool>("/vision/debug/showRobPose", true);
		debug.publishTime= new config_server::Parameter<bool>("/vision/debug/publishTime", true);
		debug.showCorrespondence= new config_server::Parameter<bool>("/vision/debug/showCorrespondence", true);
		debug.useKalmanFilter= new config_server::Parameter<bool>("/vision/debug/useKalmanFilter", true);
		debug.useMotionOdom= new config_server::Parameter<bool>("/vision/debug/useMotionOdom", false);
		
		
		
		
		
		
		debug.showLineInfo= new config_server::Parameter<bool>("/vision/debug/showLineInfo", false);
		
		
		projection.timeToShift = new config_server::Parameter<float>("/vision/projection/timeToShift", 0, 0.005, 3, 0.20);
		
		kalmanFilter.ErrorThreshod = new config_server::Parameter<int>("/vision/kalmanFilter/ErrorThreshod", 1, 1, 100, 5);
		kalmanFilter.KFPositionCov = new config_server::Parameter<int>("/vision/kalmanFilter/KFPositionCov", 0, 1, 100000000, 1000000);
		kalmanFilter.KFOrientationCov = new config_server::Parameter<int>("/vision/kalmanFilter/KFOrientationCov", 0, 1, 100000000, 1000000);
		
		
		particleFilter.ParticleNum = new config_server::Parameter<int>("/vision/particleFilter/ParticleNum", 0, 10, 5000, 200);
		particleFilter.RandomSamplePct = new config_server::Parameter<float>("/vision/particleFilter/RandomSamplePct", 0, 0.02, 1, 0.01);
                particleFilter.MinSumOfLineLengthOnImage = new config_server::Parameter<float>("/vision/particleFilter/MinSumOfLineLengthOnImage", 0, 5,1000, 100);
	        particleFilter.HeadingOffset = new config_server::Parameter<float>("/vision/particleFilter/HeadingOffset", 0, 0.1,2*M_PI, M_PI);
	  
		
		icp.IcpInlierDistThreshold  = new config_server::Parameter<int>("/vision/icp/IcpInlierDistThreshold", 0, 10, 60000, 2000);
		icp.SamplePointDist_WorldCord = new config_server::Parameter<float>("/vision/icp/SamplePointDist_WorldCord", 0, 0.1, 10.0, 0.1);
		
		icp.AvgDistThresholdForConf = new config_server::Parameter<float>("/vision/icp/AvgDistThresholdForConf", 1, 1, 400, 40);
		icp.InlierNumThresholdForConf = new config_server::Parameter<float>("/vision/icp/InlierNumThresholdForConf", 1, 1, 4000, 50);
		icp.HeadingOffset = new config_server::Parameter<float>("/vision/icp/HeadingOffset", 0, 0.1,2*M_PI, M_PI);
		
		
		
		location.x = new config_server::Parameter<float>("/vision/location/x", -5, 0.005, 5, 0);
		location.y = new config_server::Parameter<float>("/vision/location/y", -4, 0.005, 4, 0);
		location.z = new config_server::Parameter<float>("/vision/location/z",  0, 0.005, 2, 0.6);

		orientation.x = new config_server::Parameter<float>("/vision/orientation/x", -0.5*M_PI, 0.001, 0.5*M_PI, 0);
		orientation.y = new config_server::Parameter<float>("/vision/orientation/y", -0.5*M_PI, 0.001, 0.5*M_PI, 0);
		orientation.z = new config_server::Parameter<float>("/vision/orientation/z", -1.0*M_PI, 0.001, 1.0*M_PI, 0);
		
	}
};

extern ParametersC params;

#endif