

/*

Authors: Yongfeng

*/
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


// EOF
