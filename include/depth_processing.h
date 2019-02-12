#ifndef DEPTHPROCESSING_H
#define DEPTHPROCESSING_H

#include <common.h>

/*
 * usage: change disparity map into depth map (configuration included in config.h)
 * input: disparity map CV_16UC1
 * output: depth map CV_32FC1
 */
void disp2Depth(cv::Mat &dispMap, cv::Mat &depthMap);


/*
 * usage: fill the hollow part in depth map with integral method
 * input: depth (with hollow) CV_32FC1
 * output: depth (fill the hollow) CV_32FC1
 * destructive function
 */
void insertDepth_integral(cv::Mat &depth);

/*
 * usage: fill the hollow part in depth map with interpolate method
 * input: depth (with hollow) CV_32FC1
 *        kszie: interpolate window size
 *        threshold: max depth difference in window;
 * output: depth (fill the hollow) CV_32FC1
 * detructive function
 * result is as solid and smooth as integral method
 * not very recommand
 */

void insertDepth_interpolate(cv::Mat &depth, int ksize=75, double threshold=15);

#endif
