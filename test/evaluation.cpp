#include <common.h>
#include <config.h>
#include <stereoCamera.h>
#include <rectifier.h>
#include <disparityCalculator.h>
#include <depthCalculator.h>
#include <CloudSaver.h>
#include <Evaluator.h>


#if GPU_ON
int main(int argc, char *argv[])
{
    cv::Mat Q = (cv::Mat_<double>(4,4) << 1, 0, 0, -546.33655,
                                          0, 1, 0, -415.22531,
                                          0, 0, 0, 1057.7982,
                                          0, 0, 177.86472, 0);
    double scale = 1;
    cv::Mat S = (cv::Mat_<double>(4,4) << 1/scale, 0, 0, 80/scale-40,
                                                0, 1/scale, 0, 20/scale,
                                                0, 0, 0.25, 40,
                                                0, 0, 0, 1);

    cv::Mat Q_real = Q * S;
    //std::cout << Q_real << std::endl;
    DepthCalculator Transportor(Q_real);
    CloudSaver rawScene;

    cv::Mat raw = cv::imread("../data/temp2.png", cv::IMREAD_UNCHANGED);
    //std::cout << raw.type() << std::endl;
    cv::cvtColor(raw, raw, CV_BGR2BGRA);
    cv::Mat channels[4];
    cv::split(raw, channels);

    // 1. seperate RGB and disparity map.
    cv::Mat RGB, disp;
    cv::merge(channels,3, RGB);
    RGB.convertTo(RGB,-1,1.2);
    disp = channels[3];
    //std::cout << RGB.size  << " " << RGB.channels() << " " << RGB.type() << std::endl;
    //std::cout << disp.size << " " << disp.channels() << " " << disp.type() << std::endl;
    cv::namedWindow("RGB");
    cv::namedWindow("disp");
    cv::imshow("RGB",RGB);
    cv::imshow("disp",disp);
    cv::waitKey();

    // 2. calculate Depth from RGB and disparity.
    cv::Mat disp_real;
    disp.convertTo(disp_real, CV_32FC1);
    cv::cuda::GpuMat disparity_cuda(disp_real);
    cv::cuda::GpuMat depth;
    Transportor.computeDepth(disparity_cuda, depth);
    cv::Mat dep;
    depth.download(dep);
    //std::cout << dep << std::endl;
    //std::cout << dep.size << " "  << dep.type() << std::endl;

    // 3. build PointCloud for rawScene.
    std::string pcdfile = "../data/rawScene.pcd";
    rawScene.buildPointCloudColor(dep, RGB);
    rawScene.saveCloudToPCD(pcdfile);

    // 4. Judge the pointCloud.
    Evaluator Judge(rawScene.cloudTC);
    Judge.seperateCloud();
    std::cout << "done!" << std::endl;
    std::string courtdir = "../data/";
    Judge.saveAllPC(courtdir);
    Judge.fitCloudX();
    Judge.distToW();
    Judge.distToB();
    Judge.distToG();


    return 0;
}

#else
int main(){
    return 0;
}

#endif
