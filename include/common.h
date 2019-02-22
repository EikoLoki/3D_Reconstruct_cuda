#ifndef COMMON_H
#define COMMON_H

#define GPU_ON 1

#include <iostream>
#include <memory>
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/ximgproc.hpp>
#if GPU_ON
#include <opencv2/core/cuda.hpp>
#include <opencv2/cudastereo.hpp>
#include <opencv2/cudaimgproc.hpp>
#include <cuda_runtime.h>
#include <cuda.h>
#endif

#endif
