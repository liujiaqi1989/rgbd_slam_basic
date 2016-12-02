#ifndef FRAME_H
#define FRAME_H
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/nonfree/nonfree.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/cloud_viewer.h>





using namespace std;
using namespace cv;

struct InstrinsicParameters{
    double cx,cy,fx,fy,scale;
    string detectortype, matchingtype;
};



class Frame
{
public:
    Frame();
    Frame(const Frame &frame);
    Frame(const Mat &imRGB, const Mat &imDepth, const InstrinsicParameters &parameter, vector<Frame> &RefFrames);
    void FindMatching(const Frame &lastRefFrame, const string &matchingtype);

public:
    Mat imRGB;
    Mat imDepth;
    InstrinsicParameters Parameter;
    // Frame ID
    int id=0;

    // Parameter for detection
    vector<KeyPoint> keys;
    int N;
    Mat Descriptor;
    // Parameter for matching
    vector<DMatch> matches;
    vector<DMatch> goodMatches;
    vector<Point2f> matchedP;
    vector<Point3f>lastRefFrameMatchedP3d;
    // Camera matrix
    Mat cameraMatrix;
    // PnP result
    Mat rvec;
    Mat tvec;
    Mat inliers;
    // Point cloud
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud;

};

#endif // FRAME_H
