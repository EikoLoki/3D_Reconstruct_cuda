#ifndef STEREOCAMERA_H
#define STEREOCAMERA_H

#include <iostream>
#include <memory>
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/ximgproc.hpp>
#include <opencv2/core/cuda.hpp>
#include <opencv2/cudastereo.hpp>
#include <opencv2/cudaimgproc.hpp>
#include <config.h>



class StereoCameraConfig {

public:
    /* Initialize all the camera parameters*/
    StereoCameraConfig();

    void setScale(float scale);

    cv::Mat camera_intrinsic_left;
    cv::Mat camera_intrinsic_right;
    cv::Mat camera_distCoeff_left;
    cv::Mat camera_distCoeff_right;

    cv::Mat R;
    cv::Mat T;
    int width;
    int height;

};



#endif
