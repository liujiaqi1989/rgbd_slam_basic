#ifndef SLAM_H
#define SLAM_H
#include<iostream>
#include<algorithm>
#include<fstream>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/voxel_grid.h>
#include "frame.h"

using namespace std;
using namespace cv;



class Slam
{
private:
    string settingsPath;
    string resultPath;
    InstrinsicParameters Parameter;
    Frame CurrentFrame;
    vector<Frame> RefFrames;
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr Result;

public:
    Slam(const string &settingsPath, const string &resultPath);
    bool readFromTextFile();
    void Tracking(Mat &imRGB, Mat &imDepth);
};

#endif // SLAM_H
