#include <disparity.h>

void Disparity::callSGBM(Mat &left, Mat &right, Mat& disp){
    Ptr<StereoSGBM> sgbm = StereoSGBM::create(0,16,3);

#if WLS_FILTER
    Mat disparity_l;
    Mat disparity_r;
    Mat disparity_wls;
    Mat left_gray(left);
    Mat right_gray(right);
    cvtColor(left_gray, left, CV_BGR2GRAY);
    cvtColor(right_gray, right, CV_BGR2GRAY);
    Ptr<StereoMatcher> right_matcher = ximgproc::createRightMatcher(sgbm);
    Ptr<ximgproc::DisparityWLSFilter> wls_filter;
#endif

    int numberofDisparities = (left.cols / maxDisparityofImg + 15) & -16;
#if WLS_FILTER
    int cn = left_gray.channels();
#else
    int cn = left.channels();
#endif
    // parameters set from class
    sgbm->setBlockSize(SADWindowSize);
    sgbm->setPreFilterCap(preFilterCap);
    sgbm->setP1(P1 * cn * SADWindowSize*SADWindowSize);
    sgbm->setP2(P2 * cn * SADWindowSize*SADWindowSize);
    sgbm->setSpeckleRange(speckleRange);
    sgbm->setSpeckleWindowSize(speckleWindowSize);
    sgbm->setNumDisparities(numberofDisparities);

    // default parameters
    sgbm->setMode(StereoSGBM::MODE_SGBM_3WAY);
    sgbm->setDisp12MaxDiff(10);
    sgbm->setUniquenessRatio(10);

#if WLS_FILTER
    //wls filter
    wls_filter = ximgproc::createDisparityWLSFilter(sgbm);
    sgbm->compute(left_gray, right_gray, disparity_l);
    right_matcher->compute(right_gray, left_gray, disparity_r);
    wls_filter->setLambda(wls_lambda);
    wls_filter->setSigmaColor(wls_sigma);
    wls_filter->filter(disparity_l,left_gray, disparity_wls, disparity_r);
    disp = disparity_wls;
#else
    sgbm->compute(left,right,disp);
#endif
}

#if GPU_ON
void Disparity::callLibSGM(cv::Mat& left, cv::Mat& right, cv::Mat& disp){
    // set parameters for libSGM
	cvtColor(left,left,CV_BGR2GRAY);
	cvtColor(right,right,CV_BGR2GRAY);
    int numberofDisparities = 128;
    sgm::LibSGMWrapper sgmw{numberofDisparities};
    int rows = left.rows;
    int cols = left.cols;
    int size = rows * cols;
    // call cuda
    uint8_t *d_l;
    uint8_t *d_r;
    cudaMalloc((void **)&d_l, sizeof(uint8_t)*size);
    cudaMalloc((void **)&d_r, sizeof(uint8_t)*size);

    static cudaStream_t stream1;
    cudaStreamCreate(&stream1);
    cudaMemcpyAsync(d_l, left.ptr<uint8_t>(), sizeof(uint8_t)*size, cudaMemcpyHostToDevice, stream1);
    cudaMemcpyAsync(d_r, right.ptr<uint8_t>(), sizeof(uint8_t)*size, cudaMemcpyHostToDevice, stream1);
    cudaStreamSynchronize(stream1);
    cv::cuda::GpuMat d_left(rows,cols,CV_8U,d_l);
    cv::cuda::GpuMat d_right(rows,cols,CV_8U,d_r);
    cv::cuda::GpuMat d_disparity;

    sgmw.execute(d_left, d_right, d_disparity);

    disparity_cuda = d_disparity;
    d_disparity.download(disp);

    cudaFree(d_l);
    cudaFree(d_r);
	cudaStreamDestroy(stream1);
}
#endif

bool Disparity::computeDisparity(Rectify rectify){
    Mat left_rec(rectify.left_rec);
    Mat right_rec(rectify.right_rec);
    Mat disp;
#if GPU_ON
    callLibSGM(left_rec, right_rec, disp);
#else
    callSGBM(left_rec, right_rec, disp);
#endif
    disp.convertTo(disparity, CV_32F, 256.0 / (128.0*16.0));
	cv::cuda::GpuMat d_disparity = disparity_cuda;  // GpuMat need to copy to another mat and then convert values to the former one
	d_disparity.convertTo(disparity_cuda, CV_32F, 256.0 / (128.0*16.0));
	cv::Mat h_disparitycuda;
	disparity_cuda.download(h_disparitycuda);
    if (disparity.empty()){
        std::cerr << "compute disparity failed!\n" << std::endl;
        return false;
    }else{
        std::cout << "disparity type:" << disparity.type() << "\n"
                     "disparity size:" << disparity.size << "\n" << std::endl;
        return true;
    }
}

void Disparity::setP1(int P1){
    this->P1 = P1;
}

void Disparity::setP2(int P2){
    this->P2 = P2;
}

void Disparity::setPreFilterCap(int PFcap){
    preFilterCap = PFcap;
}

void Disparity::setSADWindowSize(int sad_size){
    SADWindowSize = sad_size;
}

void Disparity::setSpeckleWindowSize(int speckle_size){
    speckleWindowSize = speckle_size;
}

void Disparity::setSpeckleRange(int speckle_range){
    speckleRange = speckle_range;
}

void Disparity::setmaxDisparityofImg(int maxDisparityRatio){
    maxDisparityofImg = maxDisparityRatio;
}

int Disparity::getP1(){
    return P1;
}

int Disparity::getP2(){
    return P2;
}
