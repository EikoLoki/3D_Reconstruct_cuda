#include <opencv2/opencv.hpp>

int main(int argc , char** argv){
    cv::Mat dwn_disparity;
    dwn_disparity = cv::imread("../data/test_data/dwn_disparity.png");
    cv::imshow("original",dwn_disparity);
    cv::Mat ch1,ch2,ch3;
    std::vector<cv::Mat> channel(3);
    split(dwn_disparity,channel);
    ch1 = channel[0];
    ch2 = channel[1];
    ch3 = channel[2];
    cv::imshow("ch1",ch1);
    cv::imshow("ch2",ch2);
    cv::imshow("ch3",ch3);
    cv::waitKey(0);
    std::cout << ch1.type()<<std::endl;
    ch1.convertTo(ch1,CV_16SC1);
    cv::FileStorage file("../data/test_data/disparity.ext", cv::FileStorage::WRITE);

    // Write to file!
    file << "disp" << ch1;
    file.release();

    //temp
    cv::Mat temp;
    temp = cv::imread("../data/test_data/rectifyL.png");
    cv::resize(temp,temp,cv::Size(),0.5,0.5);
    cv::imwrite("../data/test_data/davinci_rgb.png",temp);
}
