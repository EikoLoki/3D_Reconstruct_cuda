#include <iostream>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

// 1901 new parameters
const double camera_factor = 1000; // millimeter
const double camera_cx_left = 546.0042;
const double camera_cy_left = 394.5711;
const double camera_fx_left = 1065.3;
const double camera_fy_left = 1070.8;
const double camera_s_left = 1.1642;
const double camera_cx_right = 679.8714;
const double camera_cy_right = 359.9285;
const double camera_fx_right = 1062.9;
const double camera_fy_right = 1068.0;
const double camera_s_right = 2.5731;
const double camera_baseline = 5.35;

const double RD_left[] = {-0.3700, 0.5781, -0.2837};  // Radial Distortion of left camera
const double TD_left[] = {-0.0014, -0.0073};          // Tangential Distortion of left camera
const double RD_right[] = {-0.3991, 0.7110, -0.6933}; // Radial Distortion of right camera
const double TD_right[] = {-0.0040, -0.0069};         // Tangential Distortion of right camera

Mat lMatrix = (Mat_<double>(3, 3) << camera_fx_left,  camera_s_left, camera_cx_left,
                                                  0, camera_fy_left, camera_cy_left,
                                                  0,              0,              1);

Mat rMatrix = (Mat_<double>(3, 3) << camera_fx_right,  camera_s_right, camera_cx_right,
                                                   0, camera_fy_right, camera_cy_right,
                                                   0,               0,               1);

Mat lMatrixx2 = (Mat_<double>(3, 3) << camera_fx_left*2,  camera_s_left*2, camera_cx_left*2,
                                                      0, camera_fy_left*2, camera_cy_left*2,
                                                      0,                0,                1);

Mat rMatrixx2 = (Mat_<double>(3, 3) <<  camera_fx_right*2,  camera_s_right*2, camera_cx_right*2,
                                                        0, camera_fy_right*2, camera_cy_right*2,
                                                        0,                 0,                 1);

Mat distCoeffL = (Mat_<double>(5, 1) << RD_left[0], RD_left[1], TD_left[0], TD_left[1], RD_left[2]);
Mat distCoeffR = (Mat_<double>(5, 1) << RD_right[0], RD_right[1], TD_right[0], TD_right[1], RD_right[2]);

Mat R = (Mat_<double>(3,3) << 0.999715113352754, -0.006517133674422,-0.022961252203662 ,
                            0.006921033962076, 0.999821941548705, 0.017555184039881,
                             0.022842754277588, -0.017709098408671, 0.999582210931430);         // rotation of right camera w.r.t left camera
Mat T = (Mat_<double>(3,1) << -5.403825604029611, 0.133984745864434, -0.098250420949267);       // translation of right camera w.r.t left camera

