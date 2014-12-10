//----------------------------------------------------------------------------
//
// License:  See top level LICENSE.txt file.
//
// File: openCVtestclass.h
//
// Author:  Martina Di Rita
//
// Description: Class provides OpenCV functions for DSM extraction
//
//----------------------------------------------------------------------------
#ifndef openCVtestclass_HEADER
#define openCVtestclass_HEADER 1

#include <ossim/base/ossimObject.h>
#include <ossim/base/ossimDpt.h>
#include <ossim/base/ossimString.h>

#include <opencv/cv.h>

#include <ctime>
#include <vector>
#include <iostream>

class openCVtestclass
{
public:
	openCVtestclass();
	openCVtestclass(ossimRefPtr<ossimImageData> master, ossimRefPtr<ossimImageData> slave); 
	bool execute();
	bool writeDisparity(double mean_conversionF);
	bool computeDSM(double mean_conversionF, ossimElevManager* elev, ossimImageGeometry* master_geom);
	cv::Mat wallis(cv::Mat raw_image);
	//void addArguments(ossimArgumentParser& ap);
   
	cv::Mat master_mat, slave_mat;
	cv::vector<cv::KeyPoint> keypoints1, keypoints2;
	vector<cv::DMatch > good_matches;
	cv::Mat out_disp; 
};

#endif /* #ifndef openCVtestclass_HEADER */
             
