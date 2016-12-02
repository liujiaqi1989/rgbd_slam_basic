#ifndef TRANSFORM_H
#define TRANSFORM_H
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/eigen.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/cloud_viewer.h>

#include "frame.h"

using namespace std;
using namespace cv;


class Transform
{
private:
    Frame frame;
    InstrinsicParameters Parameter;
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr tempCloud;
    Eigen::Isometry3d T;
public:
    Transform(Frame &frame);
    Frame PnP();
    Frame RGB2PointCloud();
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr jointCloud(Frame &refFrame);

};

#endif // TRANSFORM_H
