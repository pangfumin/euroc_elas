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


using namespace cv;
using namespace std;

int  main()
{
  std::string configFile = "../config/ParamSetting.yml";
  std::string EurocCfg = "../config/EuRoC.yaml";
  std::string pathTime = "../config/MH01.txt";
  
 

    // Read rectification parameters
    cv::FileStorage fsSettings(EurocCfg, cv::FileStorage::READ);
    if(!fsSettings.isOpened())
    {
        cerr << "ERROR: Wrong path to settings" << endl;
        return -1;
    }

   
    double fx,fy,cx,cy;
    double bf;
    fsSettings["Camera.fx"] >> fx;
    fsSettings["Camera.fy"] >> fy;
    fsSettings["Camera.cx"] >> cx;
    fsSettings["Camera.cy"] >> cy;
    
    fsSettings["Camera.bf"] >> bf;
    
    cv::Mat dispImage = cv::imread("../data/1403636750813555456_disp.pgm",CV_LOAD_IMAGE_GRAYSCALE);
    std::cout<< dispImage.type()<<std::endl;
    cv::namedWindow("disparity");
    
    
    // 定义点云使用的格式：这里用的是XYZRGB
    typedef pcl::PointXYZ PointT; 
    typedef pcl::PointCloud<PointT> PointCloud;
    
    // 新建一个点云
    PointCloud::Ptr pointCloud( new PointCloud ); 
    
    int rows = dispImage.rows;
    int cols = dispImage.cols;
    for ( int v=0; v<rows; v++ )
	for ( int u=0; u<cols; u++ )
	{
	    unsigned char disp = dispImage.ptr<unsigned char> ( v )[u]; // 深度值
	    std::cout<< (int)disp<<std::endl;
	    if ( disp == 0 ) continue; 
	    
	    Eigen::Vector3d point; 
	    point[2] = bf/disp; 
	    point[0] = (u-cx)*point[2]/fx;
	    point[1] = (v-cy)*point[2]/fy; 
	  
	    
	    PointT p ;
	    p.x = point[0];
	    p.y = point[1];
	    p.z = point[2];
	 
	    pointCloud->points.push_back( p );
	}
    pointCloud->is_dense = false;
    cout<<"点云共有"<<pointCloud->size()<<"个点."<<endl;
    pcl::io::savePCDFileBinary("../data/depthmap.pcd", *pointCloud );
  
    
    
    
    
   
    cv::imshow("disparity",dispImage);
    cv::waitKey();
    
  
   
   
    
 

   
    return 0;
}
