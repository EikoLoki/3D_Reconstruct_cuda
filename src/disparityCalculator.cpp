#include <disparityCalculator.h>


#if GPU_ON
void DisparityCalculator::computeDisparity(cv::cuda::GpuMat& d_left_rec, cv::cuda::GpuMat& d_right_rec, cv::cuda::GpuMat& d_disparity){

    cv::cuda::GpuMat d_left_gray, d_right_gray, d_disparity_scaled;
    cv::cuda::cvtColor(d_left_rec, d_left_gray, CV_BGR2GRAY);
    cv::cuda::cvtColor(d_right_rec, d_right_gray, CV_BGR2GRAY);
	sgmw.execute(d_left_gray, d_right_gray, d_disparity_scaled);

	d_disparity_scaled.convertTo(d_disparity, CV_32F, 1.0 / 16.0);

	if (d_disparity.empty()){
        std::cerr << "compute disparity failed!\n" << std::endl;
    }
}

#else

void DisparityCalculator::updateParameters(){
    //int numberofDisparities = ( left_rec.cols / maxDisparityofImg + 15) & -16;
    //int cn = left_rec.channels();
    int numberofDisparities = 128;
    int cn = 3;
    // parameters set from class
    sgbm->setBlockSize(SADWindowSize);
    sgbm->setPreFilterCap(preFilterCap);
    sgbm->setP1(P1 * cn * SADWindowSize*SADWindowSize);
    sgbm->setP2(P2 * cn * SADWindowSize*SADWindowSize);
    sgbm->setSpeckleRange(speckleRange);
    sgbm->setSpeckleWindowSize(speckleWindowSize);
    sgbm->setNumDisparities(numberofDisparities);

    // default parameters
    sgbm->setMode(cv::StereoSGBM::MODE_SGBM_3WAY);
    sgbm->setDisp12MaxDiff(10);
    sgbm->setUniquenessRatio(10);

}

void DisparityCalculator::computeDisparity(cv::Mat& left_rec, cv::Mat& right_rec, cv::Mat& disparity){
    cv::Mat disparity_scaled;
    sgbm->compute(left_rec, right_rec, disparity_scaled);
    disparity_scaled.convertTo(disparity, CV_32F, 1.0 / 16.0);
	
	if (disparity.empty()){
        std::cerr << "compute disparity failed!\n" << std::endl;
    }
}
#endif



