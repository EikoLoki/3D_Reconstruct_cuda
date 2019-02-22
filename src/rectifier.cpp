#include <rectifier.h>

bool Rectifier::rectifyPrepare(){
	
    cv::Size img_size(camera.width, camera.height);

    // all matrix in this function should have the data type of CV_64F
    cv::stereoRectify(camera.camera_intrinsic_left, camera.camera_distCoeff_left,
						camera.camera_intrinsic_right, camera.camera_distCoeff_right,
						img_size, camera.R, camera.T, 
						Rl, Rr, Pl, Pr, Q,
						cv::CALIB_ZERO_DISPARITY, -1, img_size);

    cv::initUndistortRectifyMap(camera.camera_intrinsic_left, camera.camera_distCoeff_left,
								Rl, Pl, img_size, CV_32FC1, mapLx, mapLy);
    cv::initUndistortRectifyMap(camera.camera_intrinsic_right, camera.camera_distCoeff_right,
								Rr, Pr, img_size, CV_32FC1, mapRx, mapRy);
		
#if GPU_ON
    d_mapLx.upload(mapLx);
    d_mapLy.upload(mapLy);
    d_mapRx.upload(mapRx);
    d_mapRy.upload(mapRy);
#endif
}


#if GPU_ON
void Rectifier::rectify(cv::cuda::GpuMat& d_left_raw, cv::cuda::GpuMat& d_right_raw, 
						cv::cuda::GpuMat& d_left_rec, cv::cuda::GpuMat& d_right_rec) {

    cv::cuda::remap(d_left_raw, d_left_rec, d_mapLx, d_mapLy, cv::INTER_LINEAR);
    cv::cuda::remap(d_right_raw,d_right_rec, d_mapRx, d_mapRy, cv::INTER_LINEAR);
}
#else
void Rectifier::rectify(cv::Mat& left_raw, cv::Mat& right_raw,
					cv::Mat& left_rec, cv::Mat& right_rec) {

    cv::remap(left_raw, left_rec, mapLx, mapLy, cv::INTER_LINEAR);
    cv::remap(right_raw, right_rec, mapRx, mapRy, cv::INTER_LINEAR);
}
#endif


