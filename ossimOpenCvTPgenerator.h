//----------------------------------------------------------------------------
//
// License:  See top level LICENSE.txt file.
//
// File: ossimOpenCvTPgenerator.h
//
// Author:  Martina Di Rita
//
// Description: Class provides a TPs generator
//
//----------------------------------------------------------------------------
#ifndef ossimOpenCvTPgenerator_HEADER
#define ossimOpenCvTPgenerator_HEADER 1

#include <opencv/cv.h>

class ossimOpenCvTPgenerator
{
public:
	ossimOpenCvTPgenerator();
	ossimOpenCvTPgenerator(cv::Mat master, cv::Mat slave); 
	void run();
	void TPgen();
	void TPdraw();
	cv::Mat estRT(std::vector<cv::Point2f> master, std::vector<cv::Point2f> slave);
	cv::Mat warp(cv::Mat slave_16bit);  
	 
	cv::Mat master_mat, slave_mat;
	cv::vector<cv::KeyPoint> keypoints1, keypoints2;
	vector<cv::DMatch > good_matches;
	double slave_x, slave_y, master_x, master_y;
};

#endif /* #ifndef ossimOpenCvTPgenerator_HEADER */
