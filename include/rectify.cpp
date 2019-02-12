#include <rectify.h>

bool Rectify::rectify(stereoCamera camera){
    // get images from camera
    cv::Mat left_raw, right_raw;
    left_raw = camera.left_src;
    right_raw = camera.right_src;

    if (left_raw.size() != right_raw.size()){
        std::cerr << "images should have same resolution, please resize" << std::endl;
        return false;
    }

    cv::Size img_size = left_raw.size();
    cv::Mat IntrinsicL = camera.getIntrinsic(camera.left);
    cv::Mat IntrinsicR = camera.getIntrinsic(camera.right);
    cv::Mat distCoeffL = camera.getDistCoeff(camera.left);
    cv::Mat distCoeffR = camera.getDistCoeff(camera.right);
    cv::Mat R = camera.getRotation();
    cv::Mat T = camera.getTranslation();


    cv::stereoRectify(IntrinsicL,distCoeffL,IntrinsicR,distCoeffR,img_size,
                      R,T,Rl,Rr,Pl,Pr,Q,cv::CALIB_ZERO_DISPARITY,-1,img_size);

    cv::initUndistortRectifyMap(IntrinsicL,distCoeffL,Rl,Pl,img_size,CV_32FC1,mapLx,mapLy);
    cv::initUndistortRectifyMap(IntrinsicR,distCoeffR,Rr,Pr,img_size,CV_32FC1,mapRx,mapRy);

    left_rec = left_raw.clone();
    right_rec = right_raw.clone();
    cv::gpu::remap(left_rec, left_rec, mapLx, mapLy, INTER_LINEAR);
    cv::remap(right_rec, right_rec, mapRx, mapRy, INTER_LINEAR);

    return true;
}
