#include <depth.h>

bool Depth::computeDepth(Disparity disparity, Rectify rectify){
    cv::Mat disp(disparity.disparity);
    setProjectionMatrix(rectify);

    if (disp.empty()){
        std::cerr << "miss disparity map!\n" << std::endl;
        return false;
    }
    else if (Q.empty()){
        std::cerr << "miss re-projection matrix!\n" << std::endl;
    }
    else{
#if GPU_ON
        cv::cuda::reprojectImageTo3D(disp,depth,Q,dst_cn = 3);
#else
        cv::reprojectImageTo3D(disp,depth,Q);
#endif
    }
    rgb = rectify.left_rec;
}

cv::Mat Depth::getProjectionMatrix(){
    return Q;
}


void Depth::setProjectionMatrix(Rectify rectify){
    Q = rectify.getQMatrix();
}
