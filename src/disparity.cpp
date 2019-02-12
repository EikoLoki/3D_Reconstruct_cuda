#include <iostream>
#include <string>

//opencv lib
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/ximgproc.hpp"


using namespace std;
using namespace cv;


void calDisparity(Mat left, Mat right, Mat& disparity)
{
    Ptr<StereoSGBM> sgbm = StereoSGBM::create(0,16,3);
    //wls
    Ptr<StereoMatcher> right_matcher = ximgproc::createRightMatcher(sgbm);
    Ptr<ximgproc::DisparityWLSFilter> wls_filter;
    Mat left_gray, right_gray, left_disparity, right_disparity;
    left_gray = left.clone();
    right_gray = right.clone();
    cvtColor(left_gray,left_gray, CV_BGR2GRAY);
    cvtColor(right_gray, right_gray, CV_BGR2GRAY);



    int numberOfDisparities = ((left.cols / 10) + 15) & -16;
    sgbm->setPreFilterCap(15); //63
    int SADWindowSize = 9;
    int sgbmWinSize = SADWindowSize > 0 ? SADWindowSize : 5;
    sgbm->setBlockSize(sgbmWinSize);
    int cn = left.channels();
    sgbm->setP1(8 * cn*sgbmWinSize*sgbmWinSize);
    sgbm->setP2(32 * cn*sgbmWinSize*sgbmWinSize);
    sgbm->setMode(StereoSGBM::MODE_SGBM_3WAY);
    sgbm->setNumDisparities(numberOfDisparities);
    sgbm->setUniquenessRatio(10);
    sgbm->setSpeckleWindowSize(1600);
    sgbm->setSpeckleRange(2);
    sgbm->setDisp12MaxDiff(10);

    wls_filter = ximgproc::createDisparityWLSFilter(sgbm);
    double start = getTickCount();

    sgbm->compute(left_gray, right_gray, left_disparity);
    right_matcher->compute(right_gray, left_gray, right_disparity);


    wls_filter->setLambda(8000);
    wls_filter->setSigmaColor(1.5);
    wls_filter->filter(left_disparity,left,disparity,right_disparity);
    Mat conf_map = Mat(left.rows, left.cols, CV_8U);
    conf_map = Scalar(255);
    Rect ROI;
    conf_map = wls_filter->getConfidenceMap();
    ROI = wls_filter->getROI();
    Mat filtered;
    ximgproc::getDisparityVis(right_disparity,filtered,1.0);
    imshow("filtered", filtered);;
    waitKey();


    double end = getTickCount();

    printf("cost time: %lf\n", (end - start)*1000 /getTickFrequency());
    //float scalar = 16.0;
    disparity = right_disparity;//scalar;
}



int main(int argc, char** argv)
{
    Mat lrgb,rrgb,disparity;
	
    lrgb = imread("../data/test_data/rectifyL.png", CV_LOAD_IMAGE_COLOR);
    rrgb = imread("../data/test_data/rectifyR.png", CV_LOAD_IMAGE_COLOR);
	if(! lrgb.data )
    {
        cout <<  "Could not open or find the image" << endl ;
        return -1;
    }

    calDisparity(lrgb, rrgb, disparity);
    Mat disp_show(disparity);
    disparity.convertTo(disparity,CV_16U);
    normalize(disp_show, disp_show, 0.1, 65535, NORM_MINMAX, CV_16UC1);
    cout << disp_show.type() << endl;
	namedWindow("disp_show", WINDOW_AUTOSIZE);
	imshow("disp_show",disp_show);
	waitKey(0);
	
	Mat rgb;
    rgb = lrgb;

    FileStorage file("../data/test_data/disparity.ext", cv::FileStorage::WRITE);

    // Write to file!
    file << "disparity" << disparity;
    file.release();

	imwrite("../data/test_data/davinci_rgb.png",rgb);
    imwrite("../data/test_data/davinci_disp_show.png",disp_show);
	
	return 0;

}
