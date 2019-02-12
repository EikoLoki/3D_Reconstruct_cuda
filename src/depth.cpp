#include <iostream>
#include <string>

//opencv lib
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/core.hpp"

// include dir
#include <depth_processing.h>

using namespace std;
using namespace cv;


int main(int argc, char** argv)
{
    
    String dir = argv[1];
    int num = atoi(argv[2]);

    for(int i = 1; i <= num; i++){
        char disparity_name[20];
        char depth_name[20];
        sprintf(disparity_name, "disparity%02d.ext",i);
        sprintf(depth_name,"depth%02d.ext",i);
        String disparity_path = dir + "/disparity/" + disparity_name;
        String depth_path = dir + "/depth/" + depth_name;

        Mat disp;

        FileStorage fs_disp(disparity_path, FileStorage::READ);
        fs_disp["disparity"] >> disp;
        fs_disp.release();

        Mat depth(disp.size(),CV_32FC1);
        cout << "i: " << i << endl;
        cout << disp.size() << endl;
        cout << disp.type() << endl;
        disp2Depth(disp, depth);
        //insertDepth_integral(depth);
        //insertDepth_interpolate(depth,75,15);

        FileStorage fs_depth(depth_path, FileStorage::WRITE);
        fs_depth << "depth" << depth;
        fs_depth.release();

    }
    return 0;

}
