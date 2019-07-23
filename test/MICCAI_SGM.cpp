/* this exmple is used to compute all the disparity map and depth map of give MICCAI challenge
 * 2019 dataset
 * Author: @eikoloki, date:07/18/2019
 */

#include <iostream>
#include <fstream>
#include <json/json.h>

#include<common.h>
#include <disparityCalculator.h>
#include <depthCalculator.h>
#include <CloudSaver.h>

cv::Mat getQMat(std::string repro_file){
    std::ifstream repro_data(repro_file);
    Json::Value matrix;
    Json::Reader reader;
    reader.parse(repro_data, matrix);
    float q_0_0 = matrix["reprojection-matrix"][0][0].asFloat();
    float q_0_3 = matrix["reprojection-matrix"][0][3].asFloat();
    float q_1_1 = matrix["reprojection-matrix"][1][1].asFloat();
    float q_1_3 = matrix["reprojection-matrix"][1][3].asFloat();
    float q_2_3 = matrix["reprojection-matrix"][2][3].asFloat();
    float q_3_2 = matrix["reprojection-matrix"][3][2].asFloat();
    cv::Mat Q  = (cv::Mat_<float>(4,4) << q_0_0, 0.0, 0.0, q_0_3,
                                   0.0, q_1_1, 0.0, q_1_3,
                                   0.0, 0.0, 0.0, q_2_3,
                                   0.0, 0.0, q_3_2, 0.0);
    //std::cout << Q << std::endl;
    return Q;
}




int main(int argc, char **argv){

//    if (argc !=2){
//        std::cerr << "please input ./<executable> <rootpath>" << std::endl;
//        return -1;
//    }

    //get the rootpath as default
    std::string rootpath;
    rootpath = "/media/xiran_zhang/TOSHIBA EXT/MICCAI_SCARED/dataset2";
    std::array<std::string, 3> keyframe= {"/keyframe_2", "/keyframe_3", "/keyframe_4"};

    for(auto k : keyframe){

        std::string data = "/data";
        std::string left_path = rootpath + k + data + "/left_finalpass";
        std::string right_path = rootpath + k + data + "/right_finalpass";
        std::string scene_path = rootpath + k + data + "/scene_points_sgm";
        std::string repro_mat_path = rootpath + k + data +"/reprojection_data";

        DisparityCalculator DispC;

        CloudSaver CloudS;


        int fileIndex = 0;
        while (true){

            char image_file[30];
            char scene_file[30];
            char repro_file[30];
            sprintf(image_file, "/frame_data%06d.png", fileIndex);
            sprintf(scene_file, "/scene_points%06d.ext", fileIndex);
            sprintf(repro_file, "/frame_data%06d.json",fileIndex);
            std::string left = left_path + image_file;
            std::string right = right_path + image_file;
            std::string scene = scene_path + scene_file;
            std::string repro_mat = repro_mat_path + repro_file;


            std::cout << left << std::endl;
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
            cv::Mat Q = getQMat(repro_mat);
            DepthCalculator DepthC(Q);

            DispC.computeDisparity(d_left_img, d_right_img, d_disp);
            DepthC.computeDepth(d_disp, d_depth);
            d_disp.download(disp);
            d_depth.download(depth);

            std::cout << "disp type: " << disp.type() << ",disp size: " << disp.size << std::endl;
            //disp.convertTo(disp, CV_8U);
            //cv::imshow("disp", disp);
            //cv::waitKey();
            std::cout << scene << std::endl;
            CloudS.saveCloudToEXT(depth, scene, "cloud");

            std::cout << "scene " << fileIndex << " finished" << std::endl;
            fileIndex ++;

        }

    }





    return 0;
}
