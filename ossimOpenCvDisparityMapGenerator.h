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
#include "ossim/imaging/ossimImageHandler.h"

class ossimOpenCvDisparityMapGenerator
{
public:
	ossimOpenCvDisparityMapGenerator();
    //void execute(cv::Mat master_mat, cv::Mat slave_mat, ossimStereoPair StereoPair, int rows, int cols, double currentRes);
    void execute(cv::Mat master_mat, cv::Mat slave_mat, ossimStereoPair StereoPair, int rows, int cols, double currentRes, ossimImageHandler *master_handler);
    cv::Mat getDisp();

    ossimRefPtr<ossimImageData> finalDisparity;
    cv::Mat array_disp;
    cv::Mat mergedDisp_array;
    int ndisparities; //Maximum disparity minus minimum disparity
    int SADWindowSize; //Matched block size
    int minimumDisp;
};

#endif /* #ifndef ossimOpenCvDisparityMapGenerator_HEADER */               
