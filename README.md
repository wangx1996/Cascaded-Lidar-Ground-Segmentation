# Cascaded-Lidar-Ground-Segmentation

[![build passing](https://img.shields.io/badge/build-passing-brightgreen.svg)](https://github.com/wangx1996/Cascaded-Lidar-Ground-Segmentation) [![velodyne64 compliant](https://img.shields.io/badge/velodyne64-compliant-red.svg)](https://github.com/wangx1996/Cascaded-Lidar-Ground-Segmentation)

A C++ version for [P. Narksri, E. Takeuchi, Y. Ninomiya, Y. Morales, N. Akai and N. Kawaguchi, "A Slope-robust Cascaded Ground Segmentation in 3D Point Cloud for Autonomous Vehicles," 2018 21st International Conference on Intelligent Transportation Systems (ITSC)](https://ieeexplore.ieee.org/document/8569534)

## Introduction

This is an c++ version implementation on the paper "A Slope-robust Cascaded Ground Segmentation in 3D Point Cloud for Autonomous Vehicles".

The origin code is in [Python](https://bitbucket.org/n-patiphon/slope_robust_ground_seg), which is vesry time consuming for runnning one frame.

So I swtich the code to c++ version, but it still very time consuming that cannnot use in real time.

The code still have a lot of parts to improve, if you want to use it, you can change the code to speed up.

## How to use

### Test platform

#### laptop:

    the 7th i5
    Ubuntu 16.04
    pcl 1.8
    OpenCV 3.3
    CMake
    
#### build

    mkdir build
    cd build
    cmake ..
    make
    ./test1 [filename]
    
#### dataset
 
 The data file is also from the paper https://bitbucket.org/n-patiphon/slope_robust_ground_seg
 
## Some Result

### 1. transform the cloud to depth image

![Image text](https://github.com/wangx1996/Cascaded-Lidar-Ground-Segmentation/blob/main/image/range.png)

### 2. sections for second segmeantion

![Image text](https://github.com/wangx1996/Cascaded-Lidar-Ground-Segmentation/blob/main/image/section.png)

### 3. segmentation result


![Image text](https://github.com/wangx1996/Cascaded-Lidar-Ground-Segmentation/blob/main/image/ground1.png)

![Image text](https://github.com/wangx1996/Cascaded-Lidar-Ground-Segmentation/blob/main/image/groun2.png)

