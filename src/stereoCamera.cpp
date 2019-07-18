#include <stereoCamera.h>



StereoCameraConfig::StereoCameraConfig(){
    width = Config::get<int>("camera_width");
    height = Config::get<int>("camera_height");

    camera_intrinsic_left = Config::get<cv::Mat>("camera_lIntrinsic");
    camera_intrinsic_right = Config::get<cv::Mat>("camera_rIntrinsic");
    camera_distCoeff_left = Config::get<cv::Mat>("camera_ldiscoeff");
    camera_distCoeff_right = Config::get<cv::Mat>("camera_rdiscoeff");

//    camera_baseline = CameraConfig::get<double>("camera.bl");
    R = Config::get<cv::Mat>("camera_R");
    T = Config::get<cv::Mat>("camera_T");
    R.convertTo(R, CV_64F);
    T.convertTo(T, CV_64F);


    std::cout << "Stereo camera initialized." << std::endl;
}

void StereoCameraConfig::setScale(float scale){

    width = width * scale;
    height = height * scale;

    camera_intrinsic_left = camera_intrinsic_left * scale;
    camera_intrinsic_left.at<double>(2,2) = 1;

    camera_intrinsic_right = camera_intrinsic_right * scale;
    camera_intrinsic_right.at<double>(2,2) = 1;
}




