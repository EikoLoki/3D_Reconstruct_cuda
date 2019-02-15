#ifndef DEPTH_H
#define DEPTH_H

#include <common.h>
#include <disparity.h>
#include <rectify.h>

class Depth{
public:
    cv::Mat depth; // 3 channels point cloud
    cv::Mat rgb;
private:
    cv::Mat Q;
public:
    bool computeDepth(Disparity disparity, Rectify rectify); // use Q to compute depth from disparity
    cv::Mat getProjectionMatrix();
private:
    void setProjectionMatrix(Rectify rectify);
};
#endif
