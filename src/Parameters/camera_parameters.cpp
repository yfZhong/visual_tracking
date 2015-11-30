// Soccer Vision Gui
// Author: Julio Pastrana <pastrana@ais.uni-bonn.de>

// camera_parameters.cpp
// Defines the constants required to distort and undistort points in the camera.
// #include "camera_parameters.h"
#include <visual_tracking/Parameters/camera_parameters.h>

//
// CamParam struct
//

// Notes:
// The constants were obtained using the OpenCV camera calibration (version 2.4.6) using a checkerboard pattern
// fx and fy are the camera focal lengths
// cx and cy specify the camera optical center
// k1 k2 p1 p2 k3 k4 k5 k6 are the distortion parameters
//
// Refer to:
// http://docs.opencv.org/doc/tutorials/calib3d/camera_calibration/camera_calibration.html

// // //according to file cCFile_180.yml
// // 
// // Camera resolution
// const float CamParam::rx = 640;
// const float CamParam::ry = 480;
// 
// // Camera parameters
// const float CamParam::fx =  221.798440893;
// const float CamParam::fy =  221.610275647;
// const float CamParam::cx =  327.622390936;
// const float CamParam::cy =  232.276802317;
// 
// Radial distortion parameters
const float CamParam::k1 = -0.256287819037122;
const float CamParam::k2 =  0.0384571071799440;
const float CamParam::k3 =  0.00565974661973100;
const float CamParam::k4 =  0.0453204862850670;
const float CamParam::k5 =  -0.0678832605487869;
const float CamParam::k6 =  0.0254666472507643;

// Tangential distortion parameters
const float CamParam::p1 =  7.18632832098473e-04;
const float CamParam::p2 = -4.62466522195010e-04;



//according to file cCFile.yml

// Camera resolution
const float CamParam::rx = 640;
const float CamParam::ry = 480;

// Camera parameters
const float CamParam::fx =  2.8775430269517409e+02;
const float CamParam::fy =  2.8704647175683243e+02;
const float CamParam::cx =  3.1809436997059066e+02;
const float CamParam::cy =  2.3856600413872948e+02;

// // Radial distortion parameters
// const float CamParam::k1 =  2.4456600037563642e-01;
// const float CamParam::k2 =  -1.6171368435162804e-02;
// const float CamParam::k3 =  2e-05;
// const float CamParam::k4 =  5.6152521554734169e-01;
// const float CamParam::k5 =  -1.7933690356214439e-03;
// const float CamParam::k6 =  -1.8958988167269407e-03;
// 
// // Tangential distortion parameters
// const float CamParam::p1 =  -4.2838353879770470e-04;
// const float CamParam::p2 =  -1.4603776327410046e-04;


// //according to file cCFile_120.yml
// 
// // Camera resolution
// const float CamParam::rx = 640;
// const float CamParam::ry = 480;
// 
// // Camera parameters
// const float CamParam::fx =  4.2255569483998300e+02;
// const float CamParam::fy =  4.0985790821751448e+02;
// const float CamParam::cx =  3.2201747639213403e+02;
// const float CamParam::cy =  2.3180273620885933e+02;
// 
// // Radial distortion parameters
// const float CamParam::k1 =  -2.0036075976899290e-01;
// const float CamParam::k2 =  -4.7354892572717784e-03;
// const float CamParam::k3 =  -1.0411465651007183e-01;
// const float CamParam::k4 =   2.9523052259431065e-01;
// const float CamParam::k5 =  -1.3996898131451441e-01;
// const float CamParam::k6 =  -1.7893552672690879e-01;
// 
// // Tangential distortion parameters
// const float CamParam::p1 =  -4.3490998969728653e-03;
// const float CamParam::p2 =  -1.7747186344773192e-03;








// Linear distortion extension parameters (calculated based on the other OpenCV-generated constants using calcparams.m)
// const float CamParam::api =  8.0;
// const float CamParam::apo =  14.0;
// const float CamParam::ani = -8.0;
// const float CamParam::ano = -12.0;
// const float CamParam::bpi =  8.0;
// const float CamParam::bpo =  12.0;
// const float CamParam::bni = -8.0;
// const float CamParam::bno = -14.0;
// const float CamParam::mx =  63.1076285607051;
// const float CamParam::bx =  332.989362945858;
// const float CamParam::my =  62.7702450488603;
// const float CamParam::by =  406.812571523206;
// EOF
