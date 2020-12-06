# 3D Reconstruct with CUDA

A simple 3D reconstruct from stereo images project.

## Introduction

This project is a implementation of 3D reconstruction by using [Semi-Global Matching](https://core.ac.uk/download/pdf/11134866.pdf)
algorithm. Thanks to [Fixstars](https://github.com/fixstars) for the open-source [libSGM](https://github.com/fixstars/libSGM) as CUDA
implementaion. The whole project can run as CUDA on/off.

### workflow

camera parameters -> config
stereo images -> rectify -> disparity -> depth

### dependency

libSGM: CUDA supported SGM algorithm (included)
OpenCV: with CUDA support compiled
PLC: PointCloud visualization (optional)

## Camera Parameters

Camera parameters are stored in ./config as config.yaml file. You can calibrate your own stereo camera by using matlab toolbox
or opencv pakage (whatever you like). You can override the content in this file. You must provide:
* image size
* Camera intrinsics
* Camera distortion coefficient
* left right camera rotation
* left right camera translation
* Essential matrix
* Fundamental matrix

## Stereo images

Same sized left right images are required, load the image from OpenCV api. If put GPU on, please call cv::cuda::GpuMat to tranfer the
image stored in CPU to GPU.

## Rectify

By using camera parameters we can remap the original image to a new aligned plane.

## Disparity

Apply SGM algorithm to compute the disparity between left and right images. This must happen after the rectification,
so that the left and right image have the aligned scan line.

## Depth

Convert the disparity to depth by some camera parameters (base line, focal length).
After getting the depth, you can save the point cloud file as .ext or render it by using PCL and then save as .pcd

## Others

the demo program is stored in /test. it shows the whole working logic.
