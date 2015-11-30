// Soccer Vision Gui
// Author: Julio Pastrana <pastrana@ais.uni-bonn.de>


#ifndef GLOBALDEFINITIONS_H
#define GLOBALDEFINITIONS_H

//Asuming that the original image is 640x480 (WxH)
#define ORG_IMAGE_WIDTH 640
#define ORG_IMAGE_HEIGHT 480
#define UNDISTOTED_IMAGE_WIDTH 1661
#define UNDISTOTED_IMAGE_HEIGHT 1251

#define SUB_SAMPLING_PARAMETER 4
#define SUB_SAMPLING_WIDTH ORG_IMAGE_WIDTH/SUB_SAMPLING_PARAMETER
#define SUB_SAMPLING_HEIGHT ORG_IMAGE_HEIGHT/SUB_SAMPLING_PARAMETER
// #define SUB_SAMPLING_NUM_PIXELS SUB_SAMPLING_WIDTH*SUB_SAMPLING_HEIGHT
// #define MAX_STACK_SIZE (SUB_SAMPLING_WIDTH*SUB_SAMPLING_HEIGHT)

//LUT variables
#define LUT_HEIGHT 256
#define LUT_WIDTH 256

//Defining ColorClasses
#define CC_BLACK	0
#define CC_WHITE	1
#define CC_FIELD	2
#define CC_BALL 	3
#define CC_GOAL		4

#define CC_LAST CC_GOAL
#define CC_NO_CLASS	255

#define PURE_WHITE_Y		10
#define PURE_BLACK_Y		245



//Defining ColorClasses

#define BALL_C 	0
#define GOAL_C	1
#define BLACK_C	2


// #define TEENSIZE_FIELD 0
// #define BONN_FIELD 1
// #if TEENSIZE_FIELD
// const double A = 9;
// const double B = 6;
// const double E = 1;
// const double F = 5;
// const double G = 2.1;
// const double H = 1.50;
// const double D = 2.6;
// #elif BONN_FIELD
// const double A = 5.45;
// const double B = 4.10;
// const double E = 0.60;
// const double F = 3.40;
// const double G = 1.30;
// const double H = 1.20;
// const double D = 2.6;
// #endif



namespace vision
{
  
  	struct FieldInfo
	{
	   double A;
	   double B ;
	   double E ;
	   double F ;
	   double G ;
	   double H ;
	   double D ;
	};
};





#endif
