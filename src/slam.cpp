#include "slam.h"
#include "transform.h"
#include "frame.h"

Slam::Slam(const string &settingsPath, const string &resultPath)
{

    this->settingsPath=settingsPath;
    this->resultPath=resultPath;
    if(!readFromTextFile()){
        cerr<<"Failed to open settings file at: "<<this->settingsPath<<endl;
    }

}

bool Slam::readFromTextFile(){

    ifstream fparameter;
    fparameter.open(settingsPath.c_str());
    if(!fparameter.is_open()){
       return false;
    }
    map<string, string> Par;
    while(!fparameter.eof())
    {
        string str;
        getline(fparameter,str);
        if(!str.empty()){
            stringstream ss;
            ss<<str;
            string key, content;
            ss>>key;
            ss>>content;
            Par.insert(make_pair(key,content));
        }
    }


    Parameter.scale=atof(Par.find("camera_factor:")->second.c_str());
    Parameter.cx=atof(Par.find("camera_cx:")->second.c_str());
    Parameter.cy=atof(Par.find("camera_cy:")->second.c_str());
    Parameter.fx=atof(Par.find("camera_fx:")->second.c_str());
    Parameter.fy=atof(Par.find("camera_fy:")->second.c_str());
    Parameter.detectortype=Par.find("Detectortype:")->second;
    Parameter.matchingtype=Par.find("Matchingtype:")->second;

    return true;
}


void Slam::Tracking(Mat &imRGB, Mat &imDepth){

    pcl::VoxelGrid<pcl::PointXYZRGBA> voxel;
    voxel.setLeafSize(0.02f,0.02f,0.02f);
    CurrentFrame=Frame(imRGB, imDepth, Parameter, RefFrames);
    Transform transform(CurrentFrame);
    if(CurrentFrame.id==0)
    {
        CurrentFrame=transform.RGB2PointCloud();
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr result(new pcl::PointCloud<pcl::PointXYZRGBA>);
        voxel.setInputCloud( CurrentFrame.cloud );
        voxel.filter(*result);
        pcl::visualization::CloudViewer viewer( "viewer" );
        viewer.showCloud(result);

        RefFrames.push_back(CurrentFrame);
    }
    else
    {
        CurrentFrame=transform.PnP();
        CurrentFrame=transform.RGB2PointCloud();
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr result=transform.jointCloud(RefFrames.back());
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr result2(new pcl::PointCloud<pcl::PointXYZRGBA>);
        voxel.setInputCloud( result );
        voxel.filter(*result2);
        pcl::visualization::CloudViewer viewer( "viewer" );
        viewer.showCloud(result2);
        while( !viewer.wasStopped() )
        {

        }
        RefFrames.push_back(CurrentFrame);
    }

}
