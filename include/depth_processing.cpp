#include <depth_processing.h>
#include <config.h>

using namespace std;
using namespace cv;

void disp2Depth(Mat &dispMap, Mat &depthMap)
{
    int type = dispMap.type();
    float fx = camera_fx_left;
    float bl = camera_baseline;
    if (type == CV_16UC1)
    {
        int height = dispMap.rows;
        int width = dispMap.cols;

        for (int i = 0; i < height; i++)
        {
            for (int j = 0; j < width; j++)
            {
                //threshold is 60 and 200
                //v2 threshold is 30 and 90
                double scale = 16.0;
                if (dispMap.at<ushort>(i, j) < 30*scale || dispMap.at<ushort>(i,j) > 120*scale){
                    depthMap.at<float>(i ,j) = 0;
                }
                else{
                    depthMap.at<float>(i, j) = (scale * 2 * fx * bl) / dispMap.at<ushort>(i, j);
                }
            }
        }
    }
    else
    {
        cout << "please confirm dispImg's type!" << endl;
        waitKey(0);
    }

}

// insert black hole

void insertDepth_integral(Mat& depth)
{
    const int width = depth.cols;
    const int height = depth.rows;
    float* data = (float*)depth.data;
    cv::Mat integralMap = cv::Mat::zeros(height, width, CV_64F);
    cv::Mat ptsMap = cv::Mat::zeros(height, width, CV_32S);
    double* integral = (double*)integralMap.data;
    int* ptsIntegral = (int*)ptsMap.data;
    memset(integral, 0, sizeof(double) * width * height);
    memset(ptsIntegral, 0, sizeof(int) * width * height);
    for (int i = 0; i < height; ++i)
    {
        int id1 = i * width;
        for (int j = 0; j < width; ++j)
        {
            int id2 = id1 + j;
            if (data[id2] > 1e-3)
            {
                integral[id2] = data[id2];
                ptsIntegral[id2] = 1;
            }
        }
    }
    // integral range
    for (int i = 0; i < height; ++i)
    {
        int id1 = i * width;
        for (int j = 1; j < width; ++j)
        {
            int id2 = id1 + j;
            integral[id2] += integral[id2 - 1];
            ptsIntegral[id2] += ptsIntegral[id2 - 1];
        }
    }
    for (int i = 1; i < height; ++i)
    {
        int id1 = i * width;
        for (int j = 0; j < width; ++j)
        {
            int id2 = id1 + j;
            integral[id2] += integral[id2 - width];
            ptsIntegral[id2] += ptsIntegral[id2 - width];
        }
    }
    int wnd;
    double dWnd = 2;
    while (dWnd > 1)
    {
        wnd = int(dWnd);
        dWnd /= 2;
        for (int i = 0; i < height; ++i)
        {
            int id1 = i * width;
            for (int j = 0; j < width; ++j)
            {
                int id2 = id1 + j;
                int left = j - wnd - 1;
                int right = j + wnd;
                int top = i - wnd - 1;
                int bot = i + wnd;
                left = max(0, left);
                right = min(right, width - 1);
                top = max(0, top);
                bot = min(bot, height - 1);
                int dx = right - left;
                int dy = (bot - top) * width;
                int idLeftTop = top * width + left;
                int idRightTop = idLeftTop + dx;
                int idLeftBot = idLeftTop + dy;
                int idRightBot = idLeftBot + dx;
                int ptsCnt = ptsIntegral[idRightBot] + ptsIntegral[idLeftTop] - (ptsIntegral[idLeftBot] + ptsIntegral[idRightTop]);
                double sumGray = integral[idRightBot] + integral[idLeftTop] - (integral[idLeftBot] + integral[idRightTop]);
                if (ptsCnt <= 0)
                {
                    continue;
                }
                data[id2] = float(sumGray / ptsCnt);
            }
        }
        int s = wnd / 2 * 2 + 1;
        if (s > 201)
        {
            s = 201;
        }
    }
}

void insertDepth_interpolate(Mat &depth, int ksize, double threshold)
{
    const int width = depth.cols;
    const int height = depth.rows;
    float* data = (float*)depth.data;
    int side = ksize / 2;

    // same row smooth
    for (int i = 0; i < height; ++i)
    {
        int id1 = i * width;
        for (int j = 0; j < width; ++j)
        {
            int id2 = id1 + j;
            if (data[id2] > 1e-3)
            {
                double left_sum = 0;
                double right_sum = 0;
                double diff = 0;
                //printf("id2:%d\tid1:%d\tside:%d\n",id2,id1,side);
                if(id2 - side >= id1 && id2 + side <= id1 + width){
                    for(int k = 0; k < side; k++){
                        left_sum += data[id2 - k] - data[id2];
                        right_sum += data[id2 + k] - data[id2];
                        //printf("left_sum: %f\tright_sum: \%f\n",left_sum,right_sum);
                    }
                }
                diff = left_sum + right_sum;
                if(diff < threshold*side && diff > -threshold*side){
                     data[id2] += diff/ (2 * double(ksize));
                    //printf("diff: %f\n",diff);
                }
            }
        }
    }

    // same col smooth
    for (int i = 0; i < height; ++i)
    {
        int id1 = i * width;
        for (int j = 0; j < width; ++j)
        {
            int id2 = id1 + j;
            if (data[id2] > 1e-3)
            {
                double top_sum = 0;
                double bot_sum = 0;
                double diff = 0;
                //printf("id2:%d\tid1:%d\tside:%d\n",id2,id1,side);
                if(id1 - side >= 0  && id1 + side <= height){
                    for(int k = 0; k < side; k++){
                        top_sum += data[id2 - k*width] - data[id2];
                        bot_sum += data[id2 + k*width] - data[id2];
                        //printf("left_sum: %f\tright_sum: \%f\n",left_sum,right_sum);
                    }
                }
                diff = top_sum + bot_sum;
                if(diff < threshold*side && diff > -threshold*side){
                     data[id2] += diff/ (2 * double(ksize));
                    //printf("diff: %f\n",diff);
                }
            }
        }
    }
}
