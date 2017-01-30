#include <iostream>
#include <fstream>
#include <boost/iterator/iterator_concepts.hpp>
#include "Struct_definition.h"
#include <opencv2/highgui/highgui.hpp>
#include <assert.h>

#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "StereoEfficientLargeScale.h"
#include "image.h"
#include<string.h>


using namespace cv;
using namespace std;

void LoadImages(const string &strPathLeft, const string &strPathRight, const string &strPathTimes,
                vector<string> &vstrImageLeft, vector<string> &vstrImageRight, vector<double> &vTimeStamps);



int  main()
{
  std::string configFile = "../config/ParamSetting.yml";
  std::string EurocCfg = "../config/EuRoC.yaml";
  std::string pathTime = "../config/MH01.txt";
  
  
  cv::FileStorage fs(configFile,cv::FileStorage::READ);
  std::string datasetBase;
  fs["datasetBase"]>>datasetBase;
  
  std::string pathLeftImages = datasetBase + "cam0/data";
  std::string pathRightImages = datasetBase + "cam1/data";


    // Retrieve paths to images
    vector<string> vstrImageLeft;
    vector<string> vstrImageRight;
    vector<double> vTimeStamp;
    LoadImages(pathLeftImages,pathRightImages, pathTime, vstrImageLeft, vstrImageRight, vTimeStamp);

    if(vstrImageLeft.empty() || vstrImageRight.empty())
    {
        cerr << "ERROR: No images in provided path." << endl;
        return 1;
    }

    if(vstrImageLeft.size()!=vstrImageRight.size())
    {
        cerr << "ERROR: Different number of left and right images." << endl;
        return 1;
    }

    // Read rectification parameters
    cv::FileStorage fsSettings(EurocCfg, cv::FileStorage::READ);
    if(!fsSettings.isOpened())
    {
        cerr << "ERROR: Wrong path to settings" << endl;
        return -1;
    }

    cv::Mat K_l, K_r, P_l, P_r, R_l, R_r, D_l, D_r;
    fsSettings["LEFT.K"] >> K_l;
    fsSettings["RIGHT.K"] >> K_r;

    fsSettings["LEFT.P"] >> P_l;
    fsSettings["RIGHT.P"] >> P_r;

    fsSettings["LEFT.R"] >> R_l;
    fsSettings["RIGHT.R"] >> R_r;

    fsSettings["LEFT.D"] >> D_l;
    fsSettings["RIGHT.D"] >> D_r;

    int rows_l = fsSettings["LEFT.height"];
    int cols_l = fsSettings["LEFT.width"];
    int rows_r = fsSettings["RIGHT.height"];
    int cols_r = fsSettings["RIGHT.width"];

    if(K_l.empty() || K_r.empty() || P_l.empty() || P_r.empty() || R_l.empty() || R_r.empty() || D_l.empty() || D_r.empty() ||
            rows_l==0 || rows_r==0 || cols_l==0 || cols_r==0)
    {
        cerr << "ERROR: Calibration parameters to rectify stereo are missing!" << endl;
        return -1;
    }

    cv::Mat M1l,M2l,M1r,M2r;
    cv::initUndistortRectifyMap(K_l,D_l,R_l,P_l.rowRange(0,3).colRange(0,3),cv::Size(cols_l,rows_l),CV_32F,M1l,M2l);
    cv::initUndistortRectifyMap(K_r,D_r,R_r,P_r.rowRange(0,3).colRange(0,3),cv::Size(cols_r,rows_r),CV_32F,M1r,M2r);


    const int nImages = vstrImageLeft.size();

   
    cv::namedWindow("leftRectify");
    cv::namedWindow("rightRectify");
    
    Mat dest;
    StereoEfficientLargeScale elas(0,128);

    // we can set various parameter
	    //elas.elas.param.ipol_gap_width=;
	    //elas.elas.param.speckle_size=getParameter("speckle_size");
	    //elas.elas.param.speckle_sim_threshold=getParameter("speckle_sim");

	
	

    // Main loop
    cv::Mat imLeft, imRight, imLeftRect, imRightRect;
    for(int ni=0; ni<nImages; ni++)
    {
        // Read left and right images from file
        imLeft = cv::imread(vstrImageLeft[ni],CV_LOAD_IMAGE_GRAYSCALE);
        imRight = cv::imread(vstrImageRight[ni],CV_LOAD_IMAGE_GRAYSCALE);

        if(imLeft.empty())
        {
            cerr << endl << "Failed to load image at: "
                 << string(vstrImageLeft[ni]) << endl;
            return 1;
        }

        if(imRight.empty())
        {
            cerr << endl << "Failed to load image at: "
                 << string(vstrImageRight[ni]) << endl;
            return 1;
        }

        cv::remap(imLeft,imLeftRect,M1l,M2l,cv::INTER_LINEAR);
        cv::remap(imRight,imRightRect,M1r,M2r,cv::INTER_LINEAR);
	
	//process (imLeftRect,imRightRect);
	elas(imLeftRect,imRightRect,dest,100);

	
	

	Mat show;
	dest.convertTo(show,CV_8U,1.0/8);
	imshow("disp",show);
     
	cv::imshow("leftRectify",imLeftRect);
	cv::imshow("rightRectify",imRightRect);
	cv::waitKey(3);

    }

    
  
    return 0;
}

void LoadImages(const string &strPathLeft, const string &strPathRight, const string &strPathTimes,
                vector<string> &vstrImageLeft, vector<string> &vstrImageRight, vector<double> &vTimeStamps)
{
    ifstream fTimes;
    fTimes.open(strPathTimes.c_str());
    vTimeStamps.reserve(5000);
    vstrImageLeft.reserve(5000);
    vstrImageRight.reserve(5000);
    while(!fTimes.eof())
    {
        string s;
        getline(fTimes,s);
        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            vstrImageLeft.push_back(strPathLeft + "/" + ss.str() + ".png");
            vstrImageRight.push_back(strPathRight + "/" + ss.str() + ".png");
            double t;
            ss >> t;
            vTimeStamps.push_back(t/1e9);

        }
    }
}
