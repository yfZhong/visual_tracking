// by Yongfeng

#ifndef LINEMATCHER_H
#define  LINEMATCHER_H

#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <algorithm>    // std::min_element, std::max_element
#include <vector>
#include <math.h>
#include <boost/timer/timer.hpp>

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <limits>


#include <visual_tracking/Projection/BackwardProjection.h>
#include <visual_tracking/Parameters/globaldefinitions.h>
#include "findLines.h"
// #include "findGoalPosts.h"
#include <visual_tracking/Tools/m_Math.h>
#include <visual_tracking/frameGrabber.h>

using namespace std;



namespace vision
      {   
	

	
	class LineMatcher
	{
	  
	
	public:
	   LineMatcher();
	   ~LineMatcher(){};
	   FieldInfo fieldInfo;
	   BackwardProjection backw_Proj;
	   std::vector< Line_w > Field_Lines;
	   std::vector< Line_w > Field_Circle_Lines_Vis;
	   
	   std::vector< Line_w > AssociationLines;
	   
	   std::vector< Line > LinesOnImg;
	   std::vector< Line_w > m_LineBuffer_Before_Merge;
	   std::vector< Line_w > m_LineBuffer_After_Merge;
           int  NumProjectedToDifferentLines;
	   float  sumofLineLength;
	   
	   void generateFieldLines();
	   void setMeasurement( std::vector< Line > LinesOnImg_);
	   void ProjectLinesToWorld(std::vector< Line > LinesOnImg, std::vector< Line_w >& m_LineBuffer_Before_Merge );
	   void MergeLines(std::vector< Line_w > m_LineBuffer_Before_Merge, std::vector< Line_w >& m_LineBuffer_After_Merge);
	   Line_w MergeTwoLines(Line_w line1, Line_w line2, int id);
	   void matchLines();
	   void run(std::vector< Line > LinesOnImg_);
	  
	   
	};



}

#endif