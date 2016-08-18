//----------------------------------------------------------------------------
//
// License:  See top level LICENSE.txt file.
//
// File: openCVtestclass.h
//
// Author:  Martina Di Rita
//
// Description: Class for disparity map extraction and merging
//
//----------------------------------------------------------------------------
#ifndef ossimImagePreprocess_HEADER
#define ossimImagePreprocess_HEADER 1

#include "ossimOpenCvTPgenerator.h"
#include "ossimDispMerging.h"
#include "ossimOpenCvDisparityMapGenerator.h"

#include <opencv2/highgui/highgui.hpp>

class ossimImagePreprocess
{
public:
	ossimImagePreprocess();
    cv::Mat wallis(cv::Mat image);
};

#endif /* #ifndef ossimImagePreprocess_HEADER */
