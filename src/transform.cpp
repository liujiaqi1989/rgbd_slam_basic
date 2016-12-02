#include "transform.h"


Transform::Transform(Frame &frame):frame(frame)
{
     pcl::PointCloud<pcl::PointXYZRGBA>::Ptr Cloud (new pcl::PointCloud<pcl::PointXYZRGBA>);
     tempCloud=Cloud;
}


Frame Transform::PnP()
{
    cout<<frame.cameraMatrix<<endl;
    solvePnPRansac(frame.lastRefFrameMatchedP3d,frame.matchedP,frame.cameraMatrix, Mat(), frame.rvec, frame.tvec,false,
                   1000,1.0,100, frame.inliers);
    Mat R;
    Rodrigues(frame.rvec,R);
    Eigen::Matrix3d r;
    cv2eigen(R,r);

    T=Eigen::Isometry3d::Identity();
    Eigen::AngleAxisd angle(r);
    T=angle;
    T(0,3)=frame.tvec.at<double>(0,0);
    T(1,3)=frame.tvec.at<double>(0,1);
    T(2,3)=frame.tvec.at<double>(0,2);
    cout<<T.matrix()<<endl;
    return frame;
}

Frame Transform::RGB2PointCloud()
{
    for(int i=0;i<frame.imDepth.rows;i++){
        for(int j=0;j<frame.imDepth.cols;j++){
            ushort d=frame.imDepth.ptr<ushort>(i)[j];
            if (d!=0){
                pcl::PointXYZRGBA p;
                p.z = double(d) / frame.Parameter.scale;
                p.x = (j - frame.Parameter.cx) * p.z / frame.Parameter.fx;
                p.y = (i - frame.Parameter.cy) * p.z / frame.Parameter.fy;
                p.b = frame.imRGB.ptr<uchar>(i)[j*3];
                p.g = frame.imRGB.ptr<uchar>(i)[j*3+1];
                p.r = frame.imRGB.ptr<uchar>(i)[j*3+2];
                frame.cloud->points.push_back(p);
            }
        }
    }
    frame.cloud->height=1;
    frame.cloud->width=frame.cloud->points.size();
    frame.cloud->is_dense=false;
    return frame;
}

pcl::PointCloud<pcl::PointXYZRGBA>::Ptr Transform::jointCloud(Frame &refFrame)
{

    pcl::transformPointCloud(*refFrame.cloud, *tempCloud, T.matrix());
    *tempCloud+=*frame.cloud;
    return tempCloud;
}
