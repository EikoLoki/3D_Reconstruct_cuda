#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d/calib3d.hpp>
//#include <opencv2/gpu/gpu.hpp>

#include <config.h>

using namespace std;
using namespace cv;

int main(int argc, char** argv)
{

    String dir = argv[1];
    int num = atoi(argv[2]);
    for(int i = 1; i <= num; i++){
        char left_name[10];
        char right_name[10];
        sprintf(left_name, "left%02d.png",i);
        sprintf(right_name, "right%02d.png",i);
        String left_path = dir +"/raw/" + left_name;
        String right_path = dir + "/raw/" +right_name;
        cout << left_path << endl;
        Mat left_src = imread(left_path,-1);
        Mat right_src = imread(right_path,-1);
        // step 2: Stereo rectify


        Mat Rl, Rr, Pl, Pr, Q;
        Size new_imgsize = left_src.size();
        stereoRectify(lMatrix, distCoeffL, rMatrix, distCoeffR, new_imgsize, R, T, Rl, Rr, Pl, Pr, Q,
                       CALIB_ZERO_DISPARITY,-1, new_imgsize);

        Mat mapLx, mapLy, mapRx, mapRy;

        initUndistortRectifyMap(lMatrix, distCoeffL, Rl, Pl, new_imgsize, CV_32FC1, mapLx, mapLy);
        initUndistortRectifyMap(rMatrix, distCoeffR, Rr, Pr, new_imgsize, CV_32FC1, mapRx, mapRy);

        // CPU remap
        Mat rectifyL, rectifyR;
        rectifyL = left_src.clone();
        rectifyR = right_src.clone();

        remap(rectifyL, rectifyL, mapLx, mapLy, INTER_LINEAR);
        remap(rectifyR, rectifyR, mapRx, mapRy, INTER_LINEAR);


        /* GPU remap
        gpu::GpuMat rawL(left_src);
        gpu::GpuMat rawR(right_src);
        gpu::GpuMat mapLx_gpu(mapLx);
        gpu::GpuMat mapLy_gpu(mapLy);
        gpu::GpuMat mapRx_gpu(mapRx);
        gpu::GpuMat mapRy_gpu(mapRy);
        gpu::GpuMat rectifyL_gpu, rectifyR_gpu;

        gpu::remap(rawL, rectifyL_gpu, mapLx_gpu, mapLx_gpu, INTER_LINEAR);
        gpu::remap(rawR, rectifyR_gpu, mapRx_gpu, mapRy_gpu, INTER_LINEAR);

        Mat rectifyL, rectifyR;
        rectifyL_gpu.download(rectifyL);
        rectifyR_gpu.download(rectifyR);
        */

        /*
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
        */
        String rectify_dir = dir + "/rectify/";
        String rectifyL_path = rectify_dir + left_name;
        String rectifyR_path = rectify_dir + right_name;
        imwrite(rectifyL_path, rectifyL);
        imwrite(rectifyR_path, rectifyR);
    }
    return 0;
}




// set left/right camera parameters
/* old param
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
*/

/* old param
Mat R = (Mat_<double>(3,3) << 0.999889955735519, -0.00611744754775240,-0.0135149271075471 ,
                            0.00622112028743159, 0.999951445252447, 0.00764230331190895,
                             0.0134675195040185, -0.00772554030747400, 0.999879463708184);
Mat T = (Mat_<double>(3,1) << -5.34590259716249, 0.0778422377075003, -0.481356139651168);
*/
