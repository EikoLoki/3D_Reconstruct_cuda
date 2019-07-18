#ifndef EVALUATOR_H
#define EVALUATOR_H

#include <common.h>
#include <vector>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <Eigen/Dense>



class Evaluator{
    typedef pcl::PointXYZRGB PointT;
    typedef pcl::PointCloud<PointT> PointCloud;
public:
    PointCloud::Ptr cloud_raw;
    PointCloud::Ptr cloud_R;
    PointCloud::Ptr cloud_G;
    PointCloud::Ptr cloud_B;
    PointCloud::Ptr cloud_W;

    float R_dist;
    float G_dist;
    float B_dist;
    float W_dist;

private:
    Eigen::Vector3f planeR_para;
    Eigen::Vector3f planeG_para;
    Eigen::Vector3f planeB_para;
    Eigen::Vector3f planeW_para;

    Eigen::Vector3f R_center;
    Eigen::Vector3f G_center;
    Eigen::Vector3f B_center;
    Eigen::Vector3f W_center;

    double R_variance;
    double G_variance;
    double B_variance;
    double W_variance;

public:
    Evaluator(PointCloud::Ptr inputCloud){
        cloud_raw = inputCloud;
    }
    ~Evaluator(){
        cloud_raw->points.clear();
        cloud_R->points.clear();
        cloud_G->points.clear();
        cloud_B->points.clear();
        cloud_W->points.clear();
    }

    /*
     * seperate raw cloud into RGBW 4 channels
     */
    void seperateCloud();

    void saveAllPC(const std::string& dir);

    void fitCloudX();

    void distToW();
    void distToB();
    void distToG();

private:
    /*
     * fit plane to RGBW 4 potential planes
     */
    Eigen::Vector3f planeFitting(PointCloud::Ptr &cloudToFit, Eigen::Vector3f &Center, double &Variance);

};

#endif
