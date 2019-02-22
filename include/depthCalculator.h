#ifndef DEPTH_H
#define DEPTH_H

#include <common.h>
#include <disparityCalculator.h>
#include <rectifier.h>

class DepthCalculator{
public:
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
