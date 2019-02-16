#include <pcd_saver.h>

void PCD_SAVER::buildPointCloud(Depth dep){
    cv::Mat depth(dep.depth);
    cv::Mat rgb(dep.rgb);
    float scale = 1000;
    PointCloud::Ptr PC(new PointCloud);
    for (int r = 0; r < depth.rows; r++){
        for (int c = 0; c < depth.cols; c++){
            float z = depth.ptr<float>(r)[3*c+2];
            if (z < 0 || z > 0.25 * scale){
                continue;
            }
            PointT p;
            p.z = z / scale;
            p.x = depth.ptr<float>(r)[3*c] / scale;
            p.y = depth.ptr<float>(r)[3*c + 1] / scale;

            p.b = rgb.ptr<uchar>(r)[3*c];
            p.g = rgb.ptr<uchar>(r)[3*c + 1];
            p.r = rgb.ptr<uchar>(r)[3*c + 2];

            PC->points.push_back(p);
        }
    }
    
    PC->height = 1;
    PC->width = PC->points.size();
    PC->is_dense = false;
    cloud = PC;

}

void PCD_SAVER::savePointCloud(const std::string& file){
    std::cout<<"point cloud size = "<< cloud->points.size() << std::endl;
    pcl::io::savePCDFile(file, *cloud);
    std::cout<<"Point cloud saved."<< std::endl;
}
