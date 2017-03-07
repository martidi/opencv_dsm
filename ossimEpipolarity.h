//----------------------------------------------------------------------------
//
// License:  See top level LICENSE.txt file.
//
// File: ossimEpipolarity.h
//
// Author:  Martina Di Rita
//
// Description: Class providing OpenCV functions for DSM extraction
//
//----------------------------------------------------------------------------
#ifndef ossimEpipolarity_HEADER
#define ossimEpipolarity_HEADER 1

#include <opencv/cv.h>
#include <ossim/imaging/ossimImageDataFactory.h>

class ossimEpipolarity
{
public:
	ossimEpipolarity();
    //ossimEpipolarity(ossimRefPtr<ossimImageData> master, ossimRefPtr<ossimImageData> slave);
    bool epipolarDirection(ossimString masterName, ossimString slaveName);
    double getMeanRotationAngle();
    double getConversionFactor();
    
    double deltaH;
    double mean_rotation_angle;
    double mean_conversion_factor;


    
    /*cv::Mat master_mat, slave_mat;
    cv::vector<cv::KeyPoint> keypoints1, keypoints2;
    vector<cv::DMatch > good_matches;
    cv::Mat out_disp;
    vector<cv::Mat> fusedDisp_array;

    double null_disp_threshold;*/
};

#endif /* #ifndef openCVtestclass_HEADER */
             
