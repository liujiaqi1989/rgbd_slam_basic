#include "frame.h"

long unsigned int nId=0;
Frame::Frame()
{}

Frame::Frame(const Frame &frame)
    :imRGB(frame.imRGB),imDepth(frame.imDepth),keys(frame.keys),N(frame.N),
      Descriptor(frame.Descriptor),cloud(frame.cloud),id(frame.id), matches(frame.matches),
      goodMatches(frame.goodMatches),cameraMatrix(frame.cameraMatrix),matchedP(frame.matchedP),
      lastRefFrameMatchedP3d(frame.lastRefFrameMatchedP3d), Parameter(frame.Parameter), rvec(frame.rvec),
      tvec(frame.tvec),inliers(frame.inliers)
{

}

Frame::Frame(const Mat &imRGB, const Mat &imDepth, const InstrinsicParameters &parameter, vector<Frame> &RefFrames)
    :Parameter(parameter)
{
    this->imRGB=imRGB.clone();
    this->imDepth=imDepth.clone();
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr Cloud (new pcl::PointCloud<pcl::PointXYZRGBA>);
    cloud=Cloud;
    initModule_nonfree();
    Ptr<FeatureDetector> detector=FeatureDetector::create(parameter.detectortype);
    Ptr<DescriptorExtractor> descriptor=DescriptorExtractor::create(parameter.detectortype);
    detector->detect(imRGB,keys);
    N=keys.size();
    descriptor->compute(imRGB, keys, Descriptor);
    id=nId++;

    double camera_matrix[3][3]={
        {parameter.fx, 0, parameter.cx},
        {0, parameter.fy, parameter.cy},
        {0,0,1}
    };
    Mat camera_Matrix(3, 3, CV_64F, camera_matrix);
    cameraMatrix=camera_Matrix.clone();
    if(id!=0)
    {
      FindMatching(RefFrames.back(),parameter.matchingtype);
    }
}

// Find matched feature points.
void Frame::FindMatching(const Frame &lastRefFrame, const string &matchingtype)
{
    Ptr<DescriptorMatcher> matcher=DescriptorMatcher::create(matchingtype);
    matcher->match(lastRefFrame.Descriptor,Descriptor,matches);

    // Find the good matches (distance bigger than 4 times minDis)
    double minDis=DBL_MAX;

    for(size_t i=0;i<matches.size();i++)
    {
        if(matches[i].distance<minDis)
        {
            minDis=matches[i].distance;
        }
    }
    for(size_t j=0;j<matches.size();j++)
    {
        if(matches[j].distance<4*minDis)
        {
            goodMatches.push_back(matches[j]);
        }
    }

    for(size_t l=0;l<goodMatches.size();l++)
    {
        // Store the image points of matched points.
        matchedP.push_back(keys[goodMatches[l].trainIdx].pt);

        Point2f lastRefFrameMatchedP=lastRefFrame.keys[goodMatches[l].queryIdx].pt;
        ushort d=lastRefFrame.imDepth.ptr<ushort>(int(lastRefFrameMatchedP.y))[int(lastRefFrameMatchedP.x)];
        if(d==0)
           continue;

        // Store the 3D points of last reference frame.
        Point3f P;
        P.z=double(d)/Parameter.scale;
        P.x=(lastRefFrameMatchedP.x-Parameter.cx)*P.z/Parameter.fx;
        P.y=(lastRefFrameMatchedP.y-Parameter.cy)*P.z/Parameter.fy;
        lastRefFrameMatchedP3d.push_back(P);
    }

}

