/* this exmple is used to compute all the disparity map and depth map of give MICCAI challenge
 * 2019 dataset
 * Author: @eikoloki, date:07/18/2019
 */

#include <iostream>
#include <fstream>
#include <experimental/filesystem>

#include<common.h>
#include <disparityCalculator.h>
#include <depthCalculator.h>
#include <CloudSaver.h>


int main(int argc, char **argv){

    if (argc !=2){
        std::cerr << "please input ./<executable> <rootpath>" << std::endl;
        return -1;
    }

    //get the rootpath as default
    std::string rootpath;
    rootpath = argv[1];
    std::string left_path = rootpath + "/left";
    std::string right_path = rootpath  + "/right";
    std::string scene_path = rootpath + "/scene_sgm";

    cv::Mat Q = (cv::Mat_<double>(4,4) << 1.0, 0.0, 0.0, -642.1846313476562,
                                          0.0, 1.0, 0.0, -520.726001739502,
                                          0.0, 0.0, 0.0, 1035.0333251953125,
                                          0.0, 0.0, 0.24134424391349485, 0.0);
    DisparityCalculator DispC;
    DepthCalculator DepthC(Q);
    CloudSaver CloudS;


    int fileIndex = 0;
    while (true){

        char left_file[20];
        char right_file[20];
        char scene_file[20];
        sprintf(left_file, "/left_%06d.png", fileIndex);
        sprintf(right_file, "/right_%06d.png", fileIndex);
        sprintf(scene_file, "/scene_%06d.ext", fileIndex);
        std::string left = left_path + left_file;
        std::string right = right_path + right_file;
        std::string scene = scene_path + scene_file;

        cv::Mat left_img = cv::imread(left);
        cv::Mat right_img = cv::imread(right);

        if(left_img.empty()){
            std::cout << "keyframe load over" << std::endl;
            break;
        }

        cv::cuda::GpuMat d_left_img(left_img);
        cv::cuda::GpuMat d_right_img(right_img);
        cv::cuda::GpuMat d_disp, d_depth;

        cv::Mat disp, depth;

        DispC.computeDisparity(d_left_img, d_right_img, d_disp);
        DepthC.computeDepth(d_disp, d_depth);
        d_disp.download(disp);
        d_depth.download(depth);

        std::cout<< "disp type: " << disp.type() << ",disp size: " << disp.size << std::endl;
        disp.convertTo(disp, CV_8U);
        cv::imshow("disp", disp);
        cv::waitKey();
        std::cout << scene << std::endl;
        CloudS.saveCloudToEXT(depth, scene, "cloud");

        std::cout << "scene " << fileIndex << " finished" << std::endl;
        fileIndex ++;

    }

    return 0;
}
