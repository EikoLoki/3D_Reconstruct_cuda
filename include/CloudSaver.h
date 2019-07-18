#ifndef PCD_SAVER_H
#define PCD_SAVER_H

#include <common.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>


/*
 * modify this to a useful Point Cloud saver class
 */




class CloudSaver{
    // define points with color as PointT with Color
    typedef pcl::PointXYZRGB PointTC;
    typedef pcl::PointCloud<PointTC> PointCloudTC;
    // define points without color as PointT
    typedef pcl::PointXYZ PointT;
    typedef pcl::PointCloud<PointT> PointCloudT;
public:
    PointCloudTC::Ptr cloudTC;
public:
    void buildPointCloudColor(cv::Mat& depth, cv::Mat& rgb);
    void saveCloudToPCD(const std::string& file);
    void saveCloudToEXT(const cv::Mat depth, const std::string& file, const std::string& keyword);


//    ~CloudSaver(){
//        cloudTC->points.clear();
//    }
};

#endif
