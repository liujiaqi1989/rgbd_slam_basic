
#include <iostream>
#include <fstream>
#include <string>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/cloud_viewer.h>



#include "slam.h"


using namespace std;
using namespace cv;

void LoadImagePath(const string &associationFilename, vector<string> &rgbFilename,
                vector<string> &depthFilename);

int main( int argc, char** argv )
{
  if(argc!=5){
      cerr<<"Usage:./main dataset_path association_path parameter_path result_path"<<endl;
  return 1;
  }

  // load image path
  vector<string> rgbFilename, depthFilename;
  LoadImagePath(argv[2],rgbFilename,depthFilename);

  Slam slam(argv[3],argv[4]);

  // Main loop
  Mat imRGB, imDepth;
  for(size_t i=0;i<rgbFilename.size();i++)
  {
      imRGB=imread(string(argv[1])+"/"+rgbFilename[i],1);
      imDepth=imread(string(argv[1])+"/"+depthFilename[i],-1);
      if(imRGB.empty())
      {
          cerr<<"Failed to load image at: "<<string(argv[1])+"/"+rgbFilename[i]<<endl;
          return 1;
      }
      if(imDepth.empty())
      {
          cerr<<"Failed to load image at: "<<string(argv[1])+"/"+depthFilename[i]<<endl;
          return 1;
      }

      slam.Tracking(imRGB, imDepth);

  }


  return 0;
}

void LoadImagePath(const string &associationFilename, vector<string> &rgbFilename,
                vector<string> &depthFilename)
{
  ifstream fassociation;
  fassociation.open(associationFilename.c_str());
  if(!fassociation.is_open()){
     cerr<<"Failed to open association file at: "<<associationFilename<<endl;
  }

  while(!fassociation.eof())
  {
      string str;
      getline(fassociation,str);
      if(!str.empty())
      {
          stringstream ss;
          ss<<str;
          double t;
          string rgb, depth;
          ss>>t;
          ss>>rgb;
          rgbFilename.push_back(rgb);
          ss>>t;
          ss>>depth;
          depthFilename.push_back(depth);
      }
  }

}
//  Transform pointTransform = Transform(rgb,depth,parameter);

//  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBA>);
//  pointTransform.RGB2PointCloud(cloud);

//  pcl::io::savePCDFile( resultPath+"/pointcloud1.pcd", *cloud );

//  cout<<"Point cloud saved."<<endl;
//  pcl::visualization::CloudViewer viewer( "viewer" );
//  viewer.showCloud(cloud);
//  while( !viewer.wasStopped() )
//  {

//  }
