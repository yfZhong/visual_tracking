#ifndef M_MATH_H
#define M_MATH_H

#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <algorithm>    // std::min_element, std::max_element
#include <vector>
#include <math.h>
#include <limits>
#include <utility>
#include <Eigen/Dense>
#include <Eigen/Eigenvalues>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/eigen.hpp>

#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <visual_tracking/Parameters/globaldefinitions.h>
#include <../../src/nimbro_robotcontrol/contrib/rbdl/source/addons/luamodel/lua-5.2.1/src/llimits.h>

using namespace std;

using namespace cv;



namespace vision
{
  
  	struct Line{
	    int id;
	    float len;
	    Vec2i s, e;
	    float k;
	    float ang;// ［-0.5*M_PI, 0.5*M_PI）
	    float conf;
	    int nodeId_s, nodeId_e;
	    
	    Line( int _id,  float l, Vec2i  _s, Vec2i _e, float _k,float _ang, float _conf);
	    Line( int _id,  Vec2i  _s, Vec2i _e);
	    Line( int _id,  Vec2i  _s, Vec2i _e, int _nodeId_s, int _nodeId_e);
	    Line(){ id = -1;};
	    Vec2i getMidPoint();
	    Line PerpendicularLineSegment( Vec2i mid, float len);
	    bool Clip(Rect boundry);
	};
  
	
	//lines in 3d world
	
	 struct Line_w{
	    int id;
	    float len_w;
	    Vec3f s_w, e_w;
	    float k_w;
	    float ang_w;// ［-0.5*M_PI, 0.5*M_PI）
	    float conf;
	    Line_w( int id_, float l_w, Vec3f _s_w, Vec3f _e_w, float _k_w, float _ang_w, float _conf);
	    Line_w( int _id, Vec3f _s_w, Vec3f _e_w);
	    Line_w(){id = -1; }
	    Vec3f getMidPoint();
	    Line_w PerpendicularLineSegment( Vec3f mid, float len);
	};
	
// 	  struct NodeId{
// 	       int id;
// 	       int Neighbor1;
// 	       int Neighbor2;
// 	       int numOfNeighbor;
// 	  };
// 	   
	
	   struct LinearGraphComponent{
		vector<int>  reachableNodeIds;
		vector<Vec2i>  Points;
		vector<Line>  Lines;
		vector<Line>  UndistortedLines;
		vector<float>  UndistortedNodeAngles; // ［-0.5*M_PI, 0.5*M_PI）
		vector<Line>  TangentLines;
		
		float sumOfLength;
		float sumOfLengthUndistorted;
		float undistortedAngleAvg; // in  radian
		float undistortedAngleChangeAvg;// in  degree
		
	   };
	   typedef vector< LinearGraphComponent > LinearGraph_Buffer;
	   
	   
	   struct ModelLineElement{
	        int id;
	        Line LongLine;
		vector<Line>  UndistortedLines;
		vector<Vec2i>  UndistortedPoints;
		float sumOfLengthUndistorted;
		float undistortedAngle;
	   };
	   typedef vector< ModelLineElement > ModelLine_Buffer;
	   
	
	   
	    struct M_rect{
		cv::Rect rect;
		double area;
		int center_x, center_y;
		int avg_x, avg_y;
		int    undistorted_x_pos;
		int    undistorted_y_pos;
		int num_points;
	    };
	    typedef vector< M_rect > Rectangle_Buffer;
	   
	   
	   
	   
	   
	   
	
	
	
      namespace m_Math
     {      
        void GetProjectivePoint( Vec2i p, Line line, Vec2i & pProject);
	void GetProjectivePoint( Vec2f p, float line_k, Vec2f pOnLine, Vec2f & pProject);
	void GetProjectivePoint( Vec2i p, float line_k, Vec2i pOnLine, Vec2i & pProject);
	float GetProjectiveDistance(Vec2i p, Line line);
	bool isProjectedInsideLineSeg( Vec2i p, Line line);
	float Point2LineSegDistance(Vec2i p, Line line);
	float TwoPointDistance( Vec2i p1, Vec2i p2);
	float dot(Vec2f c1, Vec2f c2);
	float getShortestDistance(Line line1, Line line2);
	Vec2i getClosestPointOnLine(Vec2i &p, Line &line);
// 	bool DiagonalAng( float ang1, float ang2);

	float getKValue(Vec2f c1, Vec2f c2);
	float getAngle(Vec2f c1, Vec2f c2); //［-0.5*M_PI, 0.5*M_PI）
	float getKValue(Vec2i c1, Vec2i c2);
	float getAngle(Vec2i c1, Vec2i c2);
	float getAngle2PI( Vec2i c1, Vec2i c2 );//［-M_PI, M_PI）
	float AngleDiff2PI(float ang1, float ang2);//［-M_PI, M_PI）
	float AngleAvg2PI(float ang1, float ang2);//［-M_PI, M_PI）
	float getKValueFromAngle(float ang);
	float AngDif(float ang1, float ang2);
	float AngAvg(float ang1, float ang2);
	
	
	//methods for 3d point. Assume that the z value is 0

	void GetProjectivePoint( Vec3f p, Line_w line, Vec3f & pProject);
	void GetProjectivePoint( Vec3f p, float line_k, Vec3f pOnLine, Vec3f& pProject);
        float GetProjectiveDistance(Vec3f p, Line_w line);
	void getClosestPointOnLineSeg(Vec3f p1, Line_w line ,Vec3f &pClosest);
	float PointToLineSegDistance(Vec3f p, Line_w line);
	float TwoPointDistance( Vec3f p1, Vec3f p2); 
	bool isInsideLineSeg( Vec3f p, Line_w line);
	bool isProjectedInsideLineSeg( Vec3f p, Line_w line);
	float getKValue(Vec3f c1, Vec3f c2);
	float getAngle(Vec3f c1, Vec3f c2);
	float getShortestDistance(Line_w line1, Line_w line2);
	bool getLineProjectToLineSeg(Line_w line1, Line_w line2,Line_w & seg );
	float dot(Vec3f c1, Vec3f c2);
	
	void GetProjectivePointOnCircle( Vec3f p, float r, Vec3f & pProject);
	float GetProjectiveDistanceOnCircle( Vec3f p, float r);
	
	
	
	double CorrectAngleDegree360(double x);//Normalize to [-180,180):
	double CorrectAngleRadian360(double x);//Normalize to [-PI,PI):
	double CorrectAngleDegree180(double x);//Normalize to [-90,90):
	double CorrectAngleRadian180(double x);//Normalize to [-Pi/2,PI/2):
	double Radian2Degree(double r);
	double Degree2Radian(double d);
	double RadianAngleDiff(double yaw1, double yaw2);

// 	cv::Point2f RotateCoordinateAxis(double alpha, cv::Point2f p);
// 	std::pair<Eigen::Matrix4d, Eigen::Vector4d> computeEigenValuesAndVectors(Eigen::Matrix4d mat );
	
	
	vector<Vec2i> GetMidPoints(Vec2i p1, Vec2i p2, int count  /*Means that lines count will be 2^count*/);
	
	void MergeLinesOnImg(std::vector< Line > m_LineBuffer_Before_Merge, std::vector< Line >& m_LineBuffer_After_Merge, double maxDegree, double maxDistance);
	Line MergeTwoLinesOnImg(Line line1, Line line2,  int id);
	
	void SamplePointsOnLine(Line_w line, float dist, vector<tf::Vector3>  &PointsOnLine);
	void SamplePointsOnLines(Vector<Line_w> lines, float dist, std::vector<pair<tf::Vector3, int > >   &PointsOnLines);
	void SamplePointsOnLine(Line line, float dist, vector<cv::Point>  &PointsOnLine);
	void SamplePointsOnLines(Vector<Line> lines, float dist, vector<cv::Point>  &PointsOnLines);
	
	bool intersect( Line a, Line b ,Vec2i& intersetP);
	
	
	float getTwoRectClosestDist(M_rect a, M_rect b);
	
	
	
      };

  
};





#endif