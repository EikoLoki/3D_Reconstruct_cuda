#include <iostream>
#include <string>

//opencv lib
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/core.hpp"


using namespace std;
using namespace cv;


void calDisparity(Mat left, Mat right, Mat& disparity)
{
    Ptr<StereoSGBM> sgbm = StereoSGBM::create(0,16,3);
    int numberOfDisparities = ((left.cols / 6) + 15) & -16;
    sgbm->setPreFilterCap(15); //63
    int SADWindowSize = 9;
    int sgbmWinSize = SADWindowSize > 0 ? SADWindowSize : 5;
    sgbm->setBlockSize(sgbmWinSize);
    int cn = left.channels();
    sgbm->setP1(8 * cn*sgbmWinSize*sgbmWinSize);
    sgbm->setP2(32 * cn*sgbmWinSize*sgbmWinSize);
    sgbm->setMinDisparity(0);
    sgbm->setNumDisparities(numberOfDisparities);
    sgbm->setUniquenessRatio(10);
    sgbm->setSpeckleWindowSize(1600);
    sgbm->setSpeckleRange(2);
    sgbm->setDisp12MaxDiff(10);
    sgbm->compute(left, right, disparity);

    float scalar = 16.0;
    disparity = disparity/scalar;
    //disparity = disparity + 100*one;
	//abs(disparity);
	//disparity.convertTo(disparity, CV_8U, 255/(numberOfDisparities*16));
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
	Mat disp_show(disparity.size(), CV_16UC1);
    disparity.copyTo(disp_show);
	normalize(disp_show, disp_show, 0.1, 65535, NORM_MINMAX, CV_16UC1); 
	namedWindow("disp_show", WINDOW_AUTOSIZE);
	imshow("disp_show",disp_show);
	waitKey(0);
	
	Mat rgb;
    rgb = lrgb;

    FileStorage file("../data/test_data/disparity.ext", cv::FileStorage::WRITE);

    // Write to file!
    file << "disp" << disparity;
    file.release();

	imwrite("../data/test_data/davinci_rgb.png",rgb);
    //imwrite("../data/test_data/davinci_disp.",disparity);
    imwrite("../data/test_data/davinci_disp_show.png",disp_show);
	
	return 0;

}
