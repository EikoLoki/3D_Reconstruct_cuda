#ifndef STEREOCAMERA_H
#define STEREOCAMERA_H

#include <common.h>
#include <config.h>

class StereoCamera {

public:
    /* Initialize all the camera parameters*/
    StereoCamera();

    cv::Mat camera_intrinsic_left;
    cv::Mat camera_intrinsic_right;
    cv::Mat camera_distCoeff_left;
    cv::Mat camera_distCoeff_right;

    cv::Mat R;
    cv::Mat T;
	int width;
	int height;

private:

    // camera parameters
    float camera_fx_left;
    float camera_fy_left;
    float camera_cx_left;
    float camera_cy_left;
    float camera_s_left;
    cv::Mat camera_rd_left;   // radial distortion of left camera
    cv::Mat camera_td_left;   // tangential distortion of left camera

    float camera_fx_right;
    float camera_fy_right;
    float camera_cx_right;
    float camera_cy_right;
    float camera_s_right;
    cv::Mat camera_rd_right;  // radial distortion of right camera
    cv::Mat camera_td_right;  // tangential distortion of right camera

    float camera_baseline;

};


#endif
