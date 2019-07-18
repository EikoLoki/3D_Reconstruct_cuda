#ifndef RECTIFY_H
#define RECTIFY_H

#include <common.h>
#include <stereoCamera.h>

class Rectifier{
public:
    Rectifier(StereoCameraConfig& cam) : camera(cam) {
		rectifyPrepare();
	}

#if GPU_ON
    void rectify(cv::cuda::GpuMat& d_left_raw, cv::cuda::GpuMat& d_right_raw, 
				cv::cuda::GpuMat& d_left_rec, cv::cuda::GpuMat& d_right_rec);
#else
	void rectify(cv::Mat& left_raw, cv::Mat& right_raw,
				cv::Mat& left_rec, cv::Mat& right_rec);
#endif

    cv::Mat Q; // disparity to depth 4x4

private:
	
	bool rectifyPrepare();

    StereoCameraConfig& camera;

    cv::Mat Rl;
    cv::Mat Rr;
    cv::Mat Pl; // Projection matrix for first camera 3x4
    cv::Mat Pr;


    cv::Mat mapLx;
    cv::Mat mapLy;
    cv::Mat mapRx;
    cv::Mat mapRy;

#if GPU_ON
    cv::cuda::GpuMat d_mapLx;
    cv::cuda::GpuMat d_mapLy;
    cv::cuda::GpuMat d_mapRx;
    cv::cuda::GpuMat d_mapRy; 
#endif

};

#endif
