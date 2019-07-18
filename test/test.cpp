#include <common.h>
#include <config.h>
#include <stereoCamera.h>
#include <rectifier.h>
#include <disparityCalculator.h>
#include <depthCalculator.h>
#include <CloudSaver.h>

# if GPU_ON
using namespace std;
using namespace cv;

/* get image from directory*/
void getImage(const std::string& filepath, const int number, cv::Mat& left_src, cv::Mat& right_src){
    char left_name[20];
    char right_name[20];
    sprintf(left_name, "left%02d.png",number);
    sprintf(right_name, "right%02d.png",number);

    std::string left_path = filepath + left_name;
    std::string right_path = filepath + right_name;
    std::cout << left_path << std::endl;
    left_src = cv::imread(left_path);
    right_src = cv::imread(right_path);

    if(left_src.empty() || right_src.empty()){
        std::cerr << "file does not exist!" << std::endl;
    } else {
        std::cout << "image is loaded successfully!" << std::endl;
    }
}



int main(int argc, char** argv){
    if (argc != 3){
            cerr << "please provide config file and camera file!" << endl;
            return -1;
    }

    Config::getParameterFile(argv[1]);
	int number = atoi(argv[3]);

    StereoCameraConfig camera;
    double scale = 0.6;
	camera.setScale(scale);
	Rectifier rectifier(camera);
	DisparityCalculator disparityCalculator;
	DepthCalculator depthCalculator(rectifier.Q);
    CloudSaver pcdSaver;
	for (int i = 1; i <= number; i++){


	    // 1. get new image pair
	    cv::Mat right_src;
	    cv::Mat left_src;
    	getImage(argv[2], i, left_src, right_src);

		double start = cv::getTickCount();
	
		// 2. convert src to d_src
		cv::cuda::GpuMat d_left_raw(left_src);
		cv::cuda::GpuMat d_right_raw(right_src);

		// scale
		cv::cuda::GpuMat d_left_scaled, d_right_scaled;
		cv::cuda::resize(d_left_raw, d_left_scaled, Size(), scale, scale, INTER_LINEAR);
		cv::cuda::resize(d_right_raw, d_right_scaled,Size(), scale, scale, INTER_LINEAR);
		
		// 3. rectify in GPU
		cv::cuda::GpuMat d_left_rec, d_right_rec;
		rectifier.rectify(d_left_scaled, d_right_scaled, d_left_rec, d_right_rec);

		// 4. Calculate disparity map
		cv::cuda::GpuMat d_disparity;
    	disparityCalculator.computeDisparity(d_left_rec, d_right_rec, d_disparity);
    	
		// 5. Calculate depth image
		cv::cuda::GpuMat d_depth;
    	depthCalculator.computeDepth(d_disparity, d_depth);

		double end = cv::getTickCount();
		printf("Total time: %lfms\n", (end - start)*1000/cv::getTickFrequency());

        // 6. Optional
        cv::Mat depth, rgb;
        d_depth.download(depth);
        d_left_rec.download(rgb);
        pcdSaver.buildPointCloudColor(depth, rgb);
        char filename[80];
        sprintf(filename, "../data/PointCloud%02d.pcd", i);
        pcdSaver.saveCloudToPCD(filename);
        FileStorage depth_fs("../data/depth.ext",FileStorage::WRITE);
        depth_fs << "depth" << depth;
        depth_fs.release();

	




	}

    return 0;
}
#else
int main(){
    return 0;
}
#endif
