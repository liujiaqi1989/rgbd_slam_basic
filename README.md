# rgbd_slam_basic

**Author**: Jiaqi Liu

**Current version**: 1.0

This is a self-orginized project. The goal is to realize the basic function of RGB-D SLAM.

## 1.Prerequisities
This project is developed in Ubuntu 16.04.

### C++11 or C++0x Compiler
Until now I just used the normal C++ fuctionalities. No special things from C++11.
### OpenCV
I use OpenCV to deal with images and feature points. Required at least 2.4.3.
### Eigen3
Required by PCL and g2o. Required at least 3.1.0.
### PCL
To generate and visualize the 3D point clouds.

## 2.Dataset
I use the TUM Dataset (Freiburg 1) as my test dataset.
Download the sequences from http://vision.in.tum.de/data/datasets/rgbd-dataset/download and uncompress it.
And the association files can be found in the folder "associations".


