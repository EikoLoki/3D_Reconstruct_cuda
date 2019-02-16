#ifndef PCD_SAVER_H
#define PCD_SAVER_H

#include <common.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <depth.h>
/*
 * this file is only used to test to visualize the result in point cloud
 * in real rendering please just use depth
 */
typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloud;

class PCD_SAVER{
public:
    PointCloud::Ptr cloud;
public:
    void buildPointCloud(Depth dep);
    void savePointCloud(const std::string& file);
    ~PCD_SAVER(){
        cloud->points.clear();
    }
};

#endif
