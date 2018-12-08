#include <opencv2/opencv.hpp>
#include <opencv2/calib3d/calib3d.hpp>

using namespace std;
using namespace cv;

int main(int argc, char** argv)
{
	Mat left_src = imread(argv[1],-1);
	Mat right_src = imread(argv[2],-1);


    // set left/right camera parameters
    Mat lMatrix = (Mat_<double>(3, 3) << 1094.3,  -0.3222,  559.4790,
                                              0,   1099.5,  374.7841,
                                              0,        0,         1);
    Mat rMatrix = (Mat_<double>(3, 3) << 1083.5,   0.2407,  652.3878,
                                              0,   1090.4,  366.6968,
                                              0,        0,         1);

    Mat lMatrixx2 = (Mat_<double>(3, 3) << 1094.3*2,  -0.3222*2,  559.4790*2,
                                                  0,   1099.5*2,  374.7841*2,
                                                  0,          0,           1);
    Mat rMatrixx2 = (Mat_<double>(3, 3) << 1083.5*2,   0.2407*2,  652.3878*2,
                                                  0,   1090.4*2,  366.6968*2,
                                                  0,          0,           1);
	
    Mat distCoeffL = (Mat_<double>(5, 1) << -0.3760, 0.5438, -0.00053741, -0.0074, -0.1517);
    Mat distCoeffR = (Mat_<double>(5, 1) << -0.3825, 0.5189, -0.0034, -0.0072, -0.0025);


    // step 1: blur left picture
//    Size ksize(3,3);
//    Mat left_filt, right_filt, left_dst, right_dst;
//    Mat element = getStructuringElement(MORPH_RECT,ksize);
//    bilateralFilter(left_src,left_filt,9,50,50);
//    bilateralFilter(right_src,right_filt,9,50,50);
//    erode(left_filt,left_src,element);
//    erode(right_filt,right_src,element);
//    dilate(left_src,left_src,element);
//    dilate(right_src,right_src,element);
//    GaussianBlur(left_src,left_src, ksize,0);
//    GaussianBlur(right_src,right_src,ksize,0);
    resize(left_src,left_src,Size(),2,2);
    resize(right_src,right_src,Size(),2,2);


    // undistort the left/right image
//    Mat l_undist = left_dst.clone();
//    Mat r_undist = right_dst.clone();
//    undistort(left_dst, l_undist, lMatrix, distCoeffL);
//    undistort(right_dst, r_undist, rMatrix, distCoeffR);

//    imwrite("../data/test_data/undistortL.png", l_undist);
//    imwrite("../data/test_data/undistortR.png", r_undist);

	// step 2: Stereo rectify
    Mat R = (Mat_<double>(3,3) << 0.999889955735519, -0.00611744754775240,-0.0135149271075471 ,
                                0.00622112028743159, 0.999951445252447, 0.00764230331190895,
                                 0.0134675195040185, -0.00772554030747400, 0.999879463708184);
    Mat T = (Mat_<double>(3,1) << -5.34590259716249, 0.0778422377075003, -0.481356139651168);

    Mat Rl, Rr, Pl, Pr, Q;

    Size new_imgsize = left_src.size();
    stereoRectify(lMatrixx2, distCoeffL, rMatrixx2, distCoeffR, new_imgsize, R, T, Rl, Rr, Pl, Pr, Q,
                   CALIB_ZERO_DISPARITY,-1, new_imgsize);

    Mat mapLx, mapLy, mapRx, mapRy;

    initUndistortRectifyMap(lMatrixx2, distCoeffL, Rl, Pl, new_imgsize, CV_32FC1, mapLx, mapLy);
    initUndistortRectifyMap(rMatrixx2, distCoeffR, Rr, Pr, new_imgsize, CV_32FC1, mapRx, mapRy);


    Mat rectifyL, rectifyR;
    rectifyL = left_src.clone();
    rectifyR = right_src.clone();

    remap(rectifyL, rectifyL, mapLx, mapLy, INTER_LINEAR);
    remap(rectifyR, rectifyR, mapRx, mapRy, INTER_LINEAR);

//    Mat grid = imread("/home/eikoloki/grid.png", 1);
//    Mat grid_remap_left = grid.clone();
//    Mat grid_remap_right = grid.clone();
//    remap(grid, grid_remap_left, mapLx, mapLy, INTER_LINEAR);
//    remap(grid, grid_remap_right, mapRx, mapRy, INTER_LINEAR);
    imshow("left_dst", left_src);
    imshow("right_dst", right_src);




    Mat overlap_src,overlap_rec;//overlap_undist;
    overlap_src = left_src/2 + right_src/2;
    //overlap_undist = l_undist/2 + r_undist/2;
    overlap_rec = rectifyL/2 + rectifyR/2;
    namedWindow("src_disparity",WINDOW_AUTOSIZE);
    namedWindow("undist_disparity",WINDOW_AUTOSIZE);
    namedWindow("rec_disparity", WINDOW_AUTOSIZE);
    imshow("src_disparity",overlap_src);
    imshow("rec_disparity", overlap_rec);
    //imshow("undist_disparity",overlap_undist);
    waitKey(0);
    imwrite("../data/test_data/rectifyL.png", rectifyL);
    imwrite("../data/test_data/rectifyR.png", rectifyR);
    return 0;

}
