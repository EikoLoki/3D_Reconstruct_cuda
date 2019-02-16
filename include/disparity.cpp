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

bool Disparity::computeDisparity(Rectify rectify){
    Mat left_rec(rectify.left_rec);
    Mat right_rec(rectify.right_rec);
    Mat disp;
    callSGBM(left_rec, right_rec, disp);
    disp.convertTo(disparity, CV_32F);
    disparity = disparity/16.0;
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
