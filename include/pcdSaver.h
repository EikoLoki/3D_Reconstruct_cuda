#ifndef PCD_SAVER_H
#define PCD_SAVER_H

#include <common.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <depthCalculator.h>
/*
 * this file is only used to test to visualize the result in point cloud
 * in real rendering please just use depth
 */
typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloud;

class PCDSaver{
public:
    PointCloud::Ptr cloud;
public:
    void buildPointCloud(cv::Mat& depth, cv::Mat& rgb);
    void savePointCloud(const std::string& file);
    ~PCDSaver(){
        cloud->points.clear();
    }
};

#endif
