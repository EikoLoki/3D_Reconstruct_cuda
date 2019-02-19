#include <depth.h>

bool Depth::computeDepth(Disparity disparity, Rectify rectify){
#if GPU_ON
	cv::cuda::GpuMat disp(disparity.disparity_cuda);
#else
	cv::Mat disp(disparity.disparity);
#endif
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
	cv::cuda::GpuMat d_depth;
	Q.convertTo(Q, CV_32F);  // cuda sb need Q to be CV_32F
    cv::cuda::reprojectImageTo3D(disp, d_depth, Q, 3);
	d_depth.download(depth);
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
