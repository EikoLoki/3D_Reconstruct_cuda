#include <stereoCamera.h>

stereoCamera::stereoCamera(){
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
    std::cout << "Stereo camera initialized." << std::endl;
}

cv::Mat stereoCamera::getIntrinsic(stereoCamera::device_num num){
    switch(num){
    case left:{
        cv::Mat Intrinsic = (cv::Mat_<double>(3,3) << camera_fx_left, camera_s_left, camera_cx_left,
                                                                  0, camera_fy_left, camera_cy_left,
                                                                  0,              0,              1);
        return Intrinsic;
    }
    case right:{
        cv::Mat Intrinsic = (cv::Mat_<double>(3,3) << camera_fx_right, camera_s_right, camera_cx_right,
                                                                   0, camera_fy_right, camera_cy_right,
                                                                   0,               0,               1);
        return Intrinsic;
    }
    default:
        std::cerr << "wrong device number! (device can only be left or right)" << std::endl;
    }
}


cv::Mat stereoCamera::getDistCoeff(stereoCamera::device_num num){
    switch(num){
    case left:{
        cv::Mat distCoeff = (cv::Mat_<double>(5,1) << camera_rd_left.at<float>(0), camera_rd_left.at<float>(1),
                                                     camera_td_left.at<float>(0), camera_td_left.at<float>(1),
                                                     camera_rd_left.at<float>(2));
        return distCoeff;
    }
    case right:{
        cv::Mat distCoeff = (cv::Mat_<double>(5,1) << camera_rd_right.at<float>(0), camera_rd_right.at<float>(1),
                                                     camera_td_right.at<float>(0), camera_td_right.at<float>(1),
                                                     camera_rd_right.at<float>(2));
        return distCoeff;
    }
    default:
        std::cerr << "wrong device number! (device can only be left or right)" << std::endl;
    }

}


cv::Mat stereoCamera::getRotation(){
    R.convertTo(R, CV_64F);
    return R;
}

cv::Mat stereoCamera::getTranslation(){
    T.convertTo(T, CV_64F);
    return T;
}


bool stereoCamera::getImage(const std::string& filepath, const int number){
    char left_name[20];
    char right_name[20];
    sprintf(left_name, "left%02d.png",number);
    sprintf(right_name, "right%02d.png",number);

    std::string left_path = filepath + left_name;
    std::string right_path = filepath + right_name;
    std::cout << left_path << std::endl;
    left_src = cv::imread(left_path);
    right_src = cv::imread(right_path);

    if(left_src.empty() || right_src.empty()){
        std::cerr << "file does not exist!" << std::endl;
        return false;
    } else {
        std::cout << "image is loaded successfully!" << std::endl;
        return true;
    }

}
