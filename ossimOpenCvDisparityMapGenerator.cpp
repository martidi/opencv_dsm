//----------------------------------------------------------------------------
//
// License:  See top level LICENSE.txt file.
//
// File: ossimOpenCvDisparityMapGenerator.cpp
//
// Author:  Martina Di Rita
//
// Description: Class provides Disparity Map extraction
//
//----------------------------------------------------------------------------

#include <ossim/base/ossimString.h>
#include <ossim/base/ossimNotify.h>
#include <ossim/base/ossimTrace.h>
#include <ossim/base/ossimIrect.h>
#include <ossim/base/ossimRefPtr.h>
#include <ossim/base/ossimConstants.h>
#include <ossim/elevation/ossimElevManager.h>
#include <ossim/imaging/ossimImageData.h>
#include <ossim/imaging/ossimImageSource.h>

#include "ossimOpenCvDisparityMapGenerator.h"

#include <opencv/highgui.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/flann/flann.hpp>
#include <opencv2/legacy/legacy.hpp>
// Note: These are purposely commented out to indicate non-use.
// #include <opencv2/nonfree/nonfree.hpp>
// #include <opencv2/nonfree/features2d.hpp>
// Note: These are purposely commented out to indicate non-use.

#include <vector>
#include <iostream>

ossimOpenCvDisparityMapGenerator::ossimOpenCvDisparityMapGenerator()
{
	
}

cv::Mat ossimOpenCvDisparityMapGenerator::execute(cv::Mat master_mat, cv::Mat slave_mat)
{
	cout << "DISPARITY MAP GENERATION..." << endl;
	// Disparity Map generation
	int ndisparities = 16; //Maximum disparity minus minimum disparity //con fattore di conversione 1 metti 16*2*2
	int SADWindowSize = 11;   //Matched block size

	cv::StereoSGBM sgbm;

	sgbm.preFilterCap = 63;
	sgbm.SADWindowSize = SADWindowSize > 0 ? SADWindowSize : 3;

	int cn = master_mat.channels();

	sgbm.P1 = 8*cn*sgbm.SADWindowSize*sgbm.SADWindowSize;
	sgbm.P2 = 40*cn*sgbm.SADWindowSize*sgbm.SADWindowSize;
	sgbm.minDisparity = -8; // Minimum possible disparity value  //con fattore di conversione 1 metti -16*2
	sgbm.numberOfDisparities = ndisparities;
	sgbm.uniquenessRatio = 5;
	sgbm.speckleWindowSize = 100;
	sgbm.speckleRange = 1;
	sgbm.disp12MaxDiff = 1; // Maximum allowed difference (in integer pixel units) in the left-right disparity check
	//sgbm.fullDP = true;
	
	double minVal, maxVal;
	cv::Mat array_disp;
	cv::Mat array_disp_8U;   
	sgbm(master_mat, slave_mat, array_disp);
	minMaxLoc( array_disp, &minVal, &maxVal );
	array_disp.convertTo( array_disp_8U, CV_8UC1, 255/(maxVal - minVal), -minVal*255/(maxVal - minVal));   
    cout << "min\t" << minVal << "max\t" << maxVal << endl;
	cv::namedWindow( "SGM Disparity", CV_WINDOW_NORMAL );
	cv::imshow( "SGM Disparity", array_disp_8U);
	cv::imwrite( "SGM Disparity.tif", array_disp_8U);
	
	cv::waitKey(0);


	//Create and write the log file
	ofstream disparity;
	disparity.open ("DSM_parameters_disparity.txt");
	disparity <<"DISPARITY RANGE:" << " " << ndisparities << endl;
	disparity <<"SAD WINDOW SIZE:" << " " << SADWindowSize<< endl;
	disparity << "MINIMUM DISPARITY VALUE:"<< sgbm.minDisparity << endl;
	disparity.close();	


	
	return array_disp;
}



