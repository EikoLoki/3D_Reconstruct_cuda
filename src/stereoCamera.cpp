#include <stereoCamera.h>

StereoCamera::StereoCamera(){
    width = Config::get<int>("camera.width");
    height = Config::get<int>("camera.height");

    camera_fx_left = Config::get<double>("camera.lfx");
    camera_fy_left = Config::get<double>("camera.lfy");
    camera_cx_left = Config::get<double>("camera.lcx");
    camera_cy_left = Config::get<double>("camera.lcy");
    camera_s_left = Config::get<double>("camera.ls");
    camera_rd_left = Config::get<cv::Mat>("camera.lrd");
    camera_td_left = Config::get<cv::Mat>("camera.ltd");

    camera_fx_right = Config::get<double>("camera.rfx");
    camera_fy_right = Config::get<double>("camera.rfy");
    camera_cx_right = Config::get<double>("camera.rcx");
    camera_cy_right = Config::get<double>("camera.rcy");
    camera_s_right = Config::get<double>("camera.rs");
    camera_rd_right = Config::get<cv::Mat>("camera.rrd");
    camera_td_right = Config::get<cv::Mat>("camera.rtd");

    camera_baseline = Config::get<double>("camera.bl");
    R = Config::get<cv::Mat>("camera.R");
    T = Config::get<cv::Mat>("camera.T");
	R.convertTo(R, CV_64F);
	T.convertTo(T, CV_64F);

	camera_intrinsic_left = (cv::Mat_<double>(3,3) << camera_fx_left, camera_s_left, camera_cx_left,
                                                                  0, camera_fy_left, camera_cy_left,
                                                                  0,              0,              1);

	camera_intrinsic_right = (cv::Mat_<double>(3,3) << camera_fx_right, camera_s_right, camera_cx_right,
                                                                   0, camera_fy_right, camera_cy_right,
                                                                   0,               0,               1);

    camera_distCoeff_left = (cv::Mat_<double>(5,1) << camera_rd_left.at<float>(0), camera_rd_left.at<float>(1),
                                                     camera_td_left.at<float>(0), camera_td_left.at<float>(1),
                                                     camera_rd_left.at<float>(2));

    camera_distCoeff_right = (cv::Mat_<double>(5,1) << camera_rd_right.at<float>(0), camera_rd_right.at<float>(1),
                                                     camera_td_right.at<float>(0), camera_td_right.at<float>(1),
                                                     camera_rd_right.at<float>(2));

    std::cout << "Stereo camera initialized." << std::endl;
}


