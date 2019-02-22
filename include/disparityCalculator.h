#ifndef DISPARITY_H
#define DISPARITY_H

#include <common.h>
#include <rectifier.h>
#if GPU_ON
#include <libsgm.h>
#include <libsgm_wrapper.h>
#endif

using namespace cv;

class DisparityCalculator{
public:

#if GPU_ON
    DisparityCalculator(): sgmw(128) {}

	void computeDisparity(cuda::GpuMat& d_left_rec, cuda::GpuMat& d_right_rec, cuda::GpuMat& d_disparity);
#else 
    DisparityCalculator():P1(8),P2(32),SADWindowSize(9), preFilterCap(0),
        		speckleRange(2),speckleWindowSize(200),maxDisparityofImg(10) {	
		sgbm = StereoSGBM::create(0,16,3);	
		updateParameters();
	}
	void computeDisparity(Mat& left_rec, Mat& right_rec, Mat& disparity);

	void updateParameters();
	int P1;
    int P2;
    int preFilterCap;
    int SADWindowSize;
    int speckleWindowSize;
    int speckleRange;
    int maxDisparityofImg; // image.size / max = max pixel difference of disparity

#endif
	


private:

#if GPU_ON
	sgm::LibSGMWrapper sgmw;

#else
	Ptr<StereoSGBM> sgbm;
#endif

};



#endif
