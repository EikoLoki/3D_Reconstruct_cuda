/*检测模糊度 
 返回值为模糊度，值越大越模糊，越小越清晰，范围在0到几十，10以下相对较清晰，一般为5。
 调用时可在外部设定一个阀值，具体阈值根据实际情况决定，返回值超过阀值当作是模糊图片。 
 算法所耗时间在1毫秒内
*/

#include <iostream>
#include <string>

//opencv lib
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/core.hpp"

int VideoBlurDetect(const cv::Mat &srcimg)
{
	cv::Mat img;
	cv::cvtColor(srcimg, img, CV_BGR2GRAY); // 将输入的图片转为灰度图，使用灰度图检测模糊度
 
	//图片每行字节数及高  
	int width = img.cols;
	int height = img.rows;
	ushort* sobelTable = new ushort[width*height];
	memset(sobelTable, 0, width*height*sizeof(ushort));
 
	int i, j, mul;
	//指向图像首地址  
	uchar* udata = img.data;
    for (i = 1, mul = i*width; i < height - 1; i++, mul += width){
        for (j = 1; j < width - 1; j++){
            sobelTable[mul + j] = abs(udata[mul + j - width - 1] + 2 * udata[mul + j - 1] + udata[mul + j - 1 + width] - \
            udata[mul + j + 1 - width] - 2 * udata[mul + j + 1] - udata[mul + j + width + 1]);
        }
    }

 

 
    for (i = 1, mul = i*width; i < height - 1; i++, mul += width){
        for (j = 1; j < width - 1; j++){
            if (sobelTable[mul + j] < 50 || sobelTable[mul + j] <= sobelTable[mul + j - 1] || \
                sobelTable[mul + j] <= sobelTable[mul + j + 1]) sobelTable[mul + j] = 0;
        }
    }

 
	int totLen = 0;
	int totCount = 1;
 
	uchar suddenThre = 50;
	uchar sameThre = 3;
	//遍历图片  
	for (i = 1, mul = i*width; i < height - 1; i++, mul += width)
	{
		for (j = 1; j < width - 1; j++)
		{
			if (sobelTable[mul + j])
			{
				int   count = 0;
				uchar tmpThre = 5;
				uchar max = udata[mul + j] > udata[mul + j - 1] ? 0 : 1;
 
				for (int t = j; t > 0; t--)
				{
					count++;
					if (abs(udata[mul + t] - udata[mul + t - 1]) > suddenThre)
						break;
 
					if (max && udata[mul + t] > udata[mul + t - 1])
						break;
 
					if (!max && udata[mul + t] < udata[mul + t - 1])
						break;
 
					int tmp = 0;
					for (int s = t; s > 0; s--)
					{
						if (abs(udata[mul + t] - udata[mul + s]) < sameThre)
						{
							tmp++;
							if (tmp > tmpThre) break;
						}
						else break;
					}
 
					if (tmp > tmpThre) break;
				}
 
				max = udata[mul + j] > udata[mul + j + 1] ? 0 : 1;
 
				for (int t = j; t < width; t++)
				{
					count++;
					if (abs(udata[mul + t] - udata[mul + t + 1]) > suddenThre)
						break;
 
					if (max && udata[mul + t] > udata[mul + t + 1])
						break;
 
					if (!max && udata[mul + t] < udata[mul + t + 1])
						break;
 
					int tmp = 0;
					for (int s = t; s < width; s++)
					{
						if (abs(udata[mul + t] - udata[mul + s]) < sameThre)
						{
							tmp++;
							if (tmp > tmpThre) break;
						}
						else break;
					}
 
					if (tmp > tmpThre) break;
				}
				count--;
 
				totCount++;
				totLen += count;
			}
		}
	}
	//模糊度
	float result = (float)totLen / totCount;
	delete[] sobelTable;
	sobelTable = NULL;
 
	return result;
}

int main(int argc, char** argv)
{

    cv::Mat img = cv::imread(argv[1],-1);
    int blurness = VideoBlurDetect(img);
    std::cout << blurness << std::endl;

}
