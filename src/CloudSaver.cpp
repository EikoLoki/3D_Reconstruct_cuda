#include <CloudSaver.h>

void CloudSaver::buildPointCloudColor(cv::Mat& depth, cv::Mat& rgb){
    float scale = 1;
    PointCloudTC::Ptr PC(new PointCloudTC);
    for (int r = 0; r < depth.rows; r++){
        for (int c = 0; c < depth.cols; c++){
            float z = depth.ptr<float>(r)[3*c+2];
            if (z < 0 || z > 0.25 * scale){
                continue;
            }
            PointTC p;
            p.z = z / scale;
            p.x = depth.ptr<float>(r)[3*c] / scale;
            p.y = depth.ptr<float>(r)[3*c + 1] / scale;

            p.b = rgb.ptr<uchar>(r)[3*c];
            p.g = rgb.ptr<uchar>(r)[3*c + 1];
            p.r = rgb.ptr<uchar>(r)[3*c + 2];

            PC->points.push_back(p);
        }
    }
    
    PC->height = depth.rows;
    PC->width = depth.cols;
    PC->is_dense = true;
    cloudTC = PC;

}

void CloudSaver::saveCloudToPCD(const std::string& file){
    std::cout<<"point cloud size = "<< cloudTC->points.size() << std::endl;
    pcl::io::savePCDFile(file, *cloudTC);
    std::cout<<"Point cloud saved."<< std::endl;
}


void CloudSaver::saveCloudToEXT(const cv::Mat depth, const std::string &file, const std::string& keyword){
    std::cout << "--Save Point Cloud to EXT--" << std::endl;
    cv::FileStorage tiff_fs(file, cv::FileStorage::WRITE);
    tiff_fs << keyword << depth;
    tiff_fs.release();
    std::cout << "EXT keyword: " << keyword << std::endl;
    std::cout << "EXT type: " << depth.type() << std::endl;
    std::cout << "EXT size: " << depth.size << std::endl;
    std::cout << "EXT channels: " << depth.channels() << std::endl;

}
