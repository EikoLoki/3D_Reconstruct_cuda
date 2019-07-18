#include <depthCalculator.h>

#if GPU_ON
void DepthCalculator::computeDepth(cv::cuda::GpuMat& d_disparity, cv::cuda::GpuMat& d_depth){
	cv::cuda::reprojectImageTo3D(d_disparity, d_depth, Q, 3);
}

#else
void DepthCalculator::computeDepth(cv::Mat& disparity, cv::Mat& depth){
	cv::reprojectImageTo3D(disparity, depth, Q);
}

#endif
