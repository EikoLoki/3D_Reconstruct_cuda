#ifndef DEPTH_H
#define DEPTH_H

#include <common.h>

class DepthCalculator{
public:

    // Q is the reprojection matrix
	DepthCalculator(cv::Mat q) : Q(q) {
		Q.convertTo(Q, CV_32F);
	}

#if GPU_ON
	void computeDepth(cv::cuda::GpuMat& d_disparity, cv::cuda::GpuMat& d_depth);
#else
    void computeDepth(cv::Mat& disparity, cv::Mat& depth);
#endif

private:
    cv::Mat Q;
};
#endif
