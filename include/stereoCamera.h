#ifndef STEREOCAMERA_H
#define STEREOCAMERA_H

#include <common.h>
#include <config.h>

class stereoCamera{
public:
    enum device_num{left, right};

    // camera images
    cv::Mat right_src;
    cv::Mat left_src;

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
    cv::Mat R;
    cv::Mat T;

public:
    /* Initialize all the camera parameters*/
    stereoCamera();
    cv::Mat getIntrinsic(device_num num);
    cv::Mat getDistCoeff(device_num num);
    cv::Mat getRotation();
    cv::Mat getTranslation();

    /* get image from directory*/
    bool getImage(const std::string& filepath, const int number);

};


#endif
