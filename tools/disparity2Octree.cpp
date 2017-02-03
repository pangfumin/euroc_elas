#include <iostream>
#include <fstream>
using namespace std;
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <Eigen/Geometry> 
#include <boost/format.hpp>  // for formating strings
#include <pcl/point_types.h> 
#include <pcl/io/pcd_io.h> 
#include <pcl/visualization/pcl_visualizer.h>
#include <octomap/octomap.h>
#include "../../octomap/octomap/include/octomap/Pointcloud.h"

using namespace cv;
using namespace std;

int  main(int argc, char** argv)
{
  

   
    double fx,fy,cx,cy;
    double bf;
    fx = 435.2046959714599;
    fy = 435.2046959714599;
    cx = 367.4517211914062;
    cy = 252.2008514404297;
    
    bf = 47.90639384423901;
    
    cv::Mat dispImage = cv::imread(argv[1],CV_LOAD_IMAGE_GRAYSCALE);
    //std::cout<< dispImage.type()<<std::endl;
    //cv::namedWindow("disparity");
    
    std::string treeFilenameOT =   "stereo_tree.ot";
    octomap::OcTree* tree = new octomap::OcTree(0.2);
    octomap::Pointcloud pc;
    int rows = dispImage.rows;
    int cols = dispImage.cols;
    for ( int v=0; v<rows;  v += 3 )
	for ( int u=0; u<cols; u += 3 )
	{
	    unsigned char disp = dispImage.ptr<unsigned char> ( v )[u]; // 深度值
	    
	    if ( disp == 0 ) continue; 
	   
	    float z = 18*bf/disp; 
	    float x = (u-cx)*z/fx;
	    float y = (v-cy)*z/fy; 
	    pc.push_back(x,y,z); 
	}
 
   
    //cv::imshow("disparity",dispImage);
    //cv::waitKey(1000);
    
    octomap::pose6d pose;
    
    tree->insertPointCloud(pc, pose.trans());
    tree->write(treeFilenameOT);
    std::cout<<"Write Octree"<<std::endl;
   
   
    delete tree;
 

   
    return 0;
}
