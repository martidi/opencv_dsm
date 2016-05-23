//----------------------------------------------------------------------------
//
// License:  See top level LICENSE.txt file.
//
// File: ossimOpenCvDisparityMapGenerator.h
//
// Author:  Martina Di Rita
//
// Description: Class provides Disparity Map extraction
//
//----------------------------------------------------------------------------
#ifndef ossimOpenCvDisparityMapGenerator_HEADER
#define ossimOpenCvDisparityMapGenerator_HEADER 1

#include <opencv/cv.h>
#include "ossimStereoPair.h"

class ossimOpenCvDisparityMapGenerator
{
public:
	ossimOpenCvDisparityMapGenerator();
    void execute(cv::Mat master_mat, cv::Mat slave_mat, ossimStereoPair StereoPair);
    cv::Mat getDisp();

    cv::Mat array_disp;
    cv::Mat mergedDisp_array;
    int ndisparities; //Maximum disparity minus minimum disparity
    int SADWindowSize; //Matched block size
    int minimumDisp;
};

#endif /* #ifndef ossimOpenCvDisparityMapGenerator_HEADER */               
