#ifndef RECTIFY_H
#define RECTIFY_H

#include <common.h>
#include <stereoCamera.h>

class Rectify{
public:
    cv::Mat left_rec;
    cv::Mat right_rec;
private:
    cv::Mat Rl;
    cv::Mat Rr;
    cv::Mat Pl; // Projection matrix for first camera 3x4
    cv::Mat Pr;
    cv::Mat Q; // disparity to depth 4x4

    cv::Mat mapLx;
    cv::Mat mapLy;
    cv::Mat mapRx;
    cv::Mat mapRy;

#if GPU_ON
    cv::cuda::GpuMat left_rec_g;
    cv::cuda::GpuMat right_rec_g;
    cv::cuda::GpuMat mapLx_g;
    cv::cuda::GpuMat mapLy_g;
    cv::cuda::GpuMat mapRx_g;
    cv::cuda::GpuMat mapRy_g; 
#endif

public:
    bool rectify(stereoCamera& camera);

    cv::Mat getQMatrix();
};

#endif
