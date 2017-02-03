#include <iostream>
#include <fstream>
using namespace std;
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <Eigen/Geometry> 
#include "robustoctree.h"
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
    
    std::string treeFilenameOT =   "occmapping_stereo_tree.ot";
    occmapping::RobustOcTreeParameters rTreeParam;
    occmapping::RobustOcTree * pRTree = new occmapping::RobustOcTree(rTreeParam);
    
    octomap::Pointcloud pc;
    int rows = dispImage.rows;
    int cols = dispImage.cols;
    for ( int v=0; v<rows;  v += 1 )
	for ( int u=0; u<cols; u += 1 )
	{
	    unsigned char disp = dispImage.ptr<unsigned char> ( v )[u]; // 深度值
	    
	    if ( disp == 0 ) continue; 
	   
	    float z = 18*bf/disp; 
	    float x = (u-cx)*z/fx;
	    float y = (v-cy)*z/fy; 
	    pc.push_back(x,y,z); 
	}
 
    std::cout<<"pointCloud Size： "<<pc.size()<<std::endl;
    //cv::imshow("disparity",dispImage);
    //cv::waitKey(1000);
    
     octomap::pose6d pose;
     octomap::point3d vec(1,0,0);
    
    pRTree->insertPointCloud(pc, pose.trans(),vec);
    pRTree->write(treeFilenameOT);
    std::cout<<"Write Octree"<<std::endl;
   
   
    delete pRTree;
 

   
    return 0;
}
