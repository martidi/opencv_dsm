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

class openCVtestclass
{
public:
	openCVtestclass();
    openCVtestclass(ossimRefPtr<ossimImageData> master, ossimRefPtr<ossimImageData> slave);
    openCVtestclass(ossimRefPtr<ossimImageData> forward, ossimRefPtr<ossimImageData> nadir, ossimRefPtr<ossimImageData> backward);
	
	bool execute();
	
	bool writeDisparity(double mean_conversionF);
	
    bool computeDSM(vector<double> mean_conversionF, ossimElevManager* elev, ossimImageGeometry* master_geom);

	cv::Mat wallis(cv::Mat raw_image);
   
    //DOVRANNO ESSERE RIMOSSE
    cv::Mat backward_mat, nadir_mat, forward_mat;

	vector<cv::Mat> images;
	
    vector<cv::Mat> disparity_maps;

    double null_disp_threshold;
};

#endif /* #ifndef openCVtestclass_HEADER */
             
