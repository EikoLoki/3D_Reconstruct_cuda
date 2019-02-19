#include <common.h>
#include <config.h>
#include <stereoCamera.h>
#include <rectify.h>
#include <disparity.h>
#include <depth.h>
#include <pcd_saver.h>

using namespace std;
using namespace cv;

int main(int argc, char** argv){
    if (argc != 3){
            cout << "please provide config file and camera file!" << endl;
    }

    Config::getParameterFile(argv[1]);


    // open stereo camera
    stereoCamera endo;
    int number = atoi(argv[3]);
    endo.getImage(argv[2], number);

    // start rectify
    Rectify rec;
    rec.rectify(endo);
    Mat rec_l = rec.left_rec;
    Mat rec_r = rec.right_rec;
    imshow("rec",rec_l/2 + rec_r/2);
    waitKey();

    // build disparity map
    Disparity disp;
    disp.computeDisparity(rec);
    Mat disp_map = disp.disparity;
    Mat disp_vis;
    disp_map.convertTo(disp_vis, CV_8U);
    FileStorage disp_fs("../data/disparity.ext",FileStorage::WRITE);
    disp_fs << "disparity" << disp_map;
	disp_fs.release();
    imshow("disparity", disp_vis);
    waitKey();

    //build 3 channels depth map
    Depth dep;
    dep.computeDepth(disp,rec);
    Mat Q = dep.getProjectionMatrix();
    cout << Q << endl;
    FileStorage depth_fs("../data/depth.ext",FileStorage::WRITE);
    depth_fs << "depth" << dep.depth;
	depth_fs.release();

    //build point cloud
    PCD_SAVER pcl;
    pcl.buildPointCloud(dep);
    pcl.savePointCloud("../data/PointCloud.pcd");

    return 0;
}
