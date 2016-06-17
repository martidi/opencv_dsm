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
#ifndef ossimDispMerging_HEADER
#define ossimDispMerging_HEADER 1

#include <opencv/cv.h>
#include <ossim/imaging/ossimImageDataFactory.h>
#include "ossim/imaging/ossimImageHandler.h"
#include <ossim/elevation/ossimElevManager.h>
#include <ossim/base/ossimArgumentParser.h>

#include "ossimStereoPair.h"
#include "ossimOpenCvTPgenerator.h"
#include "ossimOpenCvDisparityMapGenerator.h"

class ossimDispMerging
{
public:
        ossimDispMerging();
        bool execute(vector<ossimStereoPair> StereoPairList);
        cv::Mat getMergedDisparity();
        bool computeDsm(vector<ossimStereoPair> StereoPairList, ossimElevManager *elev, int b, ossimArgumentParser ap);
        ossimRefPtr<ossimImageData> getDsm();

private:

        bool imgConversionToMat(); //apre le img ortho e diventano opencv mat
        bool imgPreProcessing(); //vedi wallis
        bool imgGetHisto();
        bool imgConversionTo8bit();
        bool disparityFusion();

        //chiamata TPgenerator -> slave_warped

        //chiamata dense matching

        vector<cv::Mat> disp_array;
        //cv::Mat disparityMap;
        ossimRefPtr<ossimImageData> finalDSM;
        cv::Mat merged_disp;
        cv::Mat master_mat, slave_mat;
        cv::Mat master_mat_8U,slave_mat_8U;
        ossimImageHandler* master_handler;
        ossimImageHandler* slave_handler;
        double null_disp_threshold;

};

#endif /* #ifndef ossimDispMerging_HEADER */
