//----------------------------------------------------------------------------
//
// License:  See top level LICENSE.txt file.
//
// File: openCVtestclass.h
//
// Author:  Martina Di Rita
//
// Description: Class providing OpenCV functions for DSM extraction
//
//----------------------------------------------------------------------------
#ifndef openCVtestclass_HEADER
#define openCVtestclass_HEADER 1

#include <opencv/cv.h>
#include <ossim/imaging/ossimImageDataFactory.h>

class openCVtestclass
{
public:
	openCVtestclass();
    openCVtestclass(ossimRefPtr<ossimImageData> master, ossimRefPtr<ossimImageData> slave);

    bool execute(double mean_conversionF);
	bool writeDisparity(double mean_conversionF);
	
    //bool computeDSM(vector<double> mean_conversionF, ossimElevManager* elev, ossimImageGeometry* master_geom);
    ossimRefPtr<ossimImageData> computeDSM( ossimElevManager* elev, ossimImageGeometry* master_geom);
	cv::Mat wallis(cv::Mat raw_image);
   
    cv::Mat master_mat, slave_mat;
    cv::vector<cv::KeyPoint> keypoints1, keypoints2;
    vector<cv::DMatch > good_matches;
    cv::Mat out_disp;
    vector<cv::Mat> fusedDisp_array;

    double null_disp_threshold;
};

#endif /* #ifndef openCVtestclass_HEADER */
             
