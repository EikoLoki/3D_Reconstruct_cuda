#include <Evaluator.h>

void Evaluator::seperateCloud(){
    PointCloud::Ptr R(new PointCloud);
    PointCloud::Ptr G(new PointCloud);
    PointCloud::Ptr B(new PointCloud);
    PointCloud::Ptr W(new PointCloud);

    int flag_w = 0;
    int flag_r = 0;
    int flag_g = 0;
    int flag_b = 0;

    for (int i = 0; i < cloud_raw->points.size(); i++){
        PointT p;
        p = cloud_raw->at(i);
        float brightness = p.r/3.0 + p.g/3.0 + p.b/3.0;

        if (p.z > 0.14)
            continue;
        if(brightness / p.r > 0.95 && brightness / p.r <1.05 &&
                brightness / p.g > 0.95 && brightness / p.g < 1.05 &&
                brightness / p.b > 0.95 && brightness / p.b < 1.05 &&
                brightness > 150 && p.z > 0.115){
            //if (p.z < 0.13 && p.z > 0.11){
                W->points.push_back(p);
                flag_w = 1;
            //}
        }else if (brightness/p.r > 0.35 && (float)p.r/p.g >2 && (float)p.r/p.b >2 && brightness > 50) {
            //if (p.z < 0.010 && p.z > 0.087){
                R->points.push_back(p);
                flag_r = 1;
            //}
        }else if (brightness/p.g > 0.35 && (float)p.g/p.r >1 && (float)p.g/p.b >1.5 && brightness > 50) {
            //if (p.z < 0.115 && p.z > 0.102){
                G->points.push_back(p);
                flag_g = 1;
            //}
        }else if (brightness/p.b > 0.35 && (float)p.b/p.r>1.5 && (float)p.b/p.g >1.5 && brightness > 20) {
            //if (p.z < 0.120 && p.z > 0.116){
                B->points.push_back(p);
                flag_b = 1;
            //}
        }
    }
    if(flag_w == 1)
        std::cout << "W" << std::endl;
    if(flag_r == 1)
        std::cout << "R" << std::endl;
    if(flag_g == 1)
        std::cout << "G" << std::endl;
    if(flag_b == 1)
        std::cout << "B" << std::endl;

    std::cout << "End of iteration" << std::endl;
    cloud_R = R;
    cloud_G = G;
    cloud_B = B;
    cloud_W = W;
    cloud_R->height = 1;
    cloud_G->height = 1;
    cloud_B->height = 1;
    cloud_W->height = 1;
    cloud_R->width = cloud_R->points.size();
    cloud_G->width = cloud_G->points.size();
    cloud_B->width = cloud_B->points.size();
    cloud_W->width = cloud_W->points.size();
    cloud_R->is_dense = false;
    cloud_G->is_dense = false;
    cloud_B->is_dense = false;
    cloud_W->is_dense = false;
}


void Evaluator::saveAllPC(const std::string& dir){
    std::string fileR = dir + "/cloud_R.pcd";
    std::string fileG = dir + "/cloud_G.pcd";
    std::string fileB = dir + "/cloud_B.pcd";
    std::string fileW = dir + "/cloud_W.pcd";
    pcl::io::savePCDFile(fileR, *cloud_R);
    pcl::io::savePCDFile(fileG, *cloud_G);
    pcl::io::savePCDFile(fileB, *cloud_B);
    pcl::io::savePCDFile(fileW, *cloud_W);
    std::cout<<"RGB Point cloud saved."<< std::endl;
}

void Evaluator::distToW(){
    std::cout << "dist to W:" << std::endl;
    Eigen::Vector3f R_vec;
    Eigen::Vector3f G_vec;
    Eigen::Vector3f B_vec;
    R_vec = R_center - W_center;
    G_vec = G_center - W_center;
    B_vec = B_center - W_center;
    R_dist = R_vec.dot((planeR_para + planeW_para)/2);
    G_dist = G_vec.dot((planeG_para + planeW_para)/2);
    B_dist = B_vec.dot((planeB_para + planeW_para)/2);
    std::cout << "W center:" << W_center[2] << std::endl;
    std::cout << "R center:" << R_center[2] << std::endl;
    std::cout << "G center:" << G_center[2] << std::endl;
    std::cout << "B center:" << B_center[2] << std::endl;
    std::cout <<"R_dist:" << R_dist << std::endl;
    std::cout <<"G_dist:" << G_dist << std::endl;
    std::cout <<"B_dist:" << B_dist << std::endl;
}

void Evaluator::distToB(){
    std::cout << "dist to B:" << std::endl;
    Eigen::Vector3f R_vec;
    Eigen::Vector3f G_vec;
    R_vec = R_center - B_center;
    G_vec = G_center - B_center;
    R_dist = R_vec.dot((planeR_para + planeB_para)/2);
    G_dist = G_vec.dot((planeG_para + planeB_para)/2);
    std::cout << "R center:" << R_center[2] << std::endl;
    std::cout << "G center:" << G_center[2] << std::endl;
    std::cout << "B center:" << B_center[2] << std::endl;
    std::cout <<"R_dist:" << R_dist << std::endl;
    std::cout <<"G_dist:" << G_dist << std::endl;
}

void Evaluator::distToG(){
    std::cout << "dist to G:" << std::endl;
    Eigen::Vector3f R_vec;
    R_vec = R_center - G_center;
    R_dist = R_vec.dot((planeR_para + planeG_para)/2);
    std::cout << "R center:" << R_center[2] << std::endl;
    std::cout << "G center:" << G_center[2] << std::endl;
    std::cout <<"R_dist:" << R_dist << std::endl;
}
void Evaluator::fitCloudX(){
    planeR_para = planeFitting(cloud_R, R_center, R_variance);
    planeG_para = planeFitting(cloud_G, G_center, G_variance);
    planeB_para = planeFitting(cloud_B, B_center, B_variance);
    planeW_para = planeFitting(cloud_W, W_center, W_variance);

    std::cout << "R_variance:" << R_variance << std::endl;
    std::cout << "G_variance:" << G_variance << std::endl;
    std::cout << "B_variance:" << B_variance << std::endl;
    std::cout << "W_variance:" << W_variance << std::endl;


    //std::cout <<"R_para: " << planeR_para << "\nR_center: " << R_center << std::endl;
    //std::cout <<"G_para: " << planeG_para << "\nG_center: " << G_center << std::endl;
    //std::cout <<"B_para: " << planeB_para << "\nB_center: " << B_center << std::endl;
    //std::cout <<"W_para: " << planeW_para << "\nW_center: " << W_center << std::endl;
}

Eigen::Vector3f Evaluator::planeFitting(PointCloud::Ptr& cloudToFit, Eigen::Vector3f& Center, double &Variance){
    int size = cloudToFit->points.size();
    Eigen::MatrixXf coordinates(size,3);
    Eigen::Vector3f center(0,0,0);

    for (int i = 0; i < size; i++){
        PointT p;
        p = cloudToFit->points.at(i);
        coordinates(i,0) = p.x;
        coordinates(i,1) = p.y;
        coordinates(i,2) = p.z;
        center(0) += p.x;
        center(1) += p.y;
        center(2) += p.z;
    }

    center /= size;
    //std::cout << "center: " << center << std::endl;
    // x y z normalized

    Eigen::MatrixXf coordinates_norm = coordinates.rowwise() - center.transpose();

    // 5 multipliers
    Eigen::VectorXf X(size,1);
    Eigen::VectorXf Y(size,1);
    Eigen::VectorXf Z(size,1);
    X = coordinates_norm.col(0);
    Y = coordinates_norm.col(1);
    Z = coordinates_norm.col(2);

    float XX = X.transpose() * X;
    float XY = X.transpose() * Y;
    float YY = Y.transpose() * Y;
    float XZ = X.transpose() * Z;
    float YZ = Y.transpose() * Z;

    float D = XX * YY - XY * XY;
    float a = YZ * XY - XZ * YY;
    float b = XY * XZ - XX * YZ;
    Eigen::Vector3f param(a/D, b/D, 1);

    Eigen::VectorXf Z_prim = coordinates_norm * param;
    Eigen::VectorXf Z_diff = Z + Z_prim;


    // outlier filter
    double threshold = 0.004;
    for(int i = 0; i < 10; i++){
        int new_size = 0;
        for (int r = 0; r < Z_diff.rows(); r++){
            if ( Z_diff(r) < threshold && Z_diff(r) > -threshold){
                new_size ++;
            }
        }

        Eigen::MatrixXf coordinates_new(new_size,3);
        int count = 0;
        for (int r = 0; r < Z_diff.rows(); r++){
            if (Z_diff(r) < threshold && Z_diff(r) > -threshold){
                coordinates_new(count,0) = coordinates(r,0);
                coordinates_new(count,1) = coordinates(r,1);
                coordinates_new(count,2) = coordinates(r,2);
                count ++;
            }
        }

        center = coordinates_new.colwise().mean();
        Eigen::MatrixXf coordinates_new_norm = coordinates_new.rowwise() - center.transpose();
        Eigen::VectorXf X = coordinates_new_norm.col(0);
        Eigen::VectorXf Y = coordinates_new_norm.col(1);
        Eigen::VectorXf Z = coordinates_new_norm.col(2);


        float XX = X.transpose() * X;
        float XY = X.transpose() * Y;
        float YY = Y.transpose() * Y;
        float XZ = X.transpose() * Z;
        float YZ = Y.transpose() * Z;


        float D = XX * YY - XY * XY;
        float a = YZ * XY - XZ * YY;
        float b = XY * XZ - XX * YZ;
        param(0) = a/D;
        param(1) = b/D;

        Z_prim = coordinates_new_norm * param;
        Z_diff = Z + Z_prim;
        std::cout << "variance: "<<Z_diff.dot(Z_diff) << "dim: " << Z_diff.rows() << std::endl;
    }

    Center = center;
    Variance = Z_diff.dot(Z_diff) / Z_diff.rows();

    return param;

}
