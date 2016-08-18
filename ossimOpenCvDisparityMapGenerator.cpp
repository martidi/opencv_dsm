//----------------------------------------------------------------------------
//
// License:  See top level LICENSE.txt file.
//
// File: ossimOpenCvDisparityMapGenerator.cpp
//
// Author:  Martina Di Rita
//
// Description: Class providing Disparity Map generation
//
//----------------------------------------------------------------------------

#include <ossim/imaging/ossimImageSource.h>
#include "ossimOpenCvDisparityMapGenerator.h"

#include <opencv2/highgui/highgui.hpp>
// Note: These are purposely commented out to indicate non-use.
// #include <opencv2/nonfree/nonfree.hpp>
// #include <opencv2/nonfree/features2d.hpp>
// Note: These are purposely commented out to indicate non-use.
#include <iostream>

ossimOpenCvDisparityMapGenerator::ossimOpenCvDisparityMapGenerator()
{
	
}

void ossimOpenCvDisparityMapGenerator::execute(cv::Mat master_mat, cv::Mat slave_mat, ossimStereoPair StereoPair)
{
	cout << "DISPARITY MAP GENERATION \t in progress..." << endl;
		
	//******************************************************
	// Abilitate for computing disparity on different scales 
	//******************************************************
	/*
	double fscale = 1.0/2.0;
	cv::resize(master_mat, master_mat, cv::Size(), fscale, fscale, cv::INTER_AREA );
	cv::resize(slave_mat, slave_mat, cv::Size(), fscale, fscale, cv::INTER_AREA );	
	cv::namedWindow( "Scaled master", CV_WINDOW_NORMAL );
	cv::imshow( "Scaled master", master_mat);
	cv::namedWindow( "Scaled slave", CV_WINDOW_NORMAL );
	cv::imshow( "Scaled slave", slave_mat);
	*/	
					
    ndisparities = 16; //Maximum disparity minus minimum disparity - prove a 128
    minimumDisp = -8; //prova a -32
    SADWindowSize = 5; //Matched block size

    // Disparity Map generation
    int cn = master_mat.channels();
    cv::StereoSGBM sgbm;

    sgbm.preFilterCap = 63;
    sgbm.SADWindowSize = SADWindowSize > 0 ? SADWindowSize : 3;
    sgbm.P1 = 8*cn*sgbm.SADWindowSize*sgbm.SADWindowSize;
    sgbm.P2 = 40*cn*sgbm.SADWindowSize*sgbm.SADWindowSize;
    sgbm.minDisparity = minimumDisp; // Minimum possible disparity value  //con fattore di conversione 1 metti -16*2
    sgbm.numberOfDisparities = ndisparities;
    sgbm.uniquenessRatio = 5;
    sgbm.speckleWindowSize = 100;
    sgbm.speckleRange = 1;
    sgbm.disp12MaxDiff = 1; // Maximum allowed difference (in integer pixel units) in the left-right disparity check
    //sgbm.fullDP = true; //activate for consider 8 directions (Hirschmuller algorithm) instead of 5;

    double minVal, maxVal;

    cv::Mat array_disp_8U;
    sgbm(master_mat, slave_mat, array_disp);
    cv::imwrite( "float_Disparity_0.tif", array_disp);

    cv::FileStorage fs("test.yml", cv::FileStorage::WRITE);
    fs << "cameraMatrix" << array_disp;
    fs.release();

    minMaxLoc( array_disp, &minVal, &maxVal );
    array_disp.convertTo( array_disp_8U, CV_8UC1, 255/(maxVal - minVal), -minVal*255/(maxVal - minVal));
    cout << "min\t" << minVal << " " << "max\t" << maxVal << endl;
    cv::namedWindow( "SGM Disparity", CV_WINDOW_NORMAL );
    cv::imshow( "SGM Disparity", array_disp_8U);
    cv::imwrite( "SGM Disparity.tif", array_disp_8U);
    cv::waitKey(0);


	//******************************************************
	// Abilitate for computing disparity on different scales 
	//****************************************************** 	
	//array_disp = array_disp/fscale; // to consider the scale factor also in the disparity values (i singoli valori sono alterati)
	//cv::resize(array_disp, array_disp, cv::Size(), 1.0/fscale, 1.0/fscale, cv::INTER_AREA ); // to resize the disparity map as the initial image

	//Create and write the log file
	ofstream disparity;
	disparity.open ("DSM_parameters_disparity.txt");
	disparity <<"DISPARITY RANGE:" << " " << ndisparities << endl;
	disparity <<"SAD WINDOW SIZE:" << " " << SADWindowSize<< endl;
	disparity << "MINIMUM DISPARITY VALUE:"<< sgbm.minDisparity << endl;
    disparity.close();

    // ogni disp map deve essere ruotata, convertita a CV_64F, divisa per 16 bit, resa metrica tramite il conv fact

    // Rotation for along-track OPTICAL images
    //********* To be commented for SAR images *********
    //cv::transpose(array_disp, array_disp);
    //cv::flip(array_disp, array_disp, 0);
    //********* To be commented for SAR images *********



   array_disp.convertTo(array_disp, CV_64F);
   array_disp = ((array_disp/16.0)) / StereoPair.getConversionFactor(); //quando divido per il fattore di conversione le rendo metriche

    for(int i=0; i< array_disp.rows; i++)
    {
        for(int j=0; j< array_disp.cols; j++)
        {
            if(array_disp.at<double>(i,j) < (minimumDisp + 0.5 - 1 )/ StereoPair.getConversionFactor())
            {
                array_disp.at<double>(i,j) = -9999.0;
            }
        }
    }

    cv::FileStorage valori("valori.yml", cv::FileStorage::WRITE);
    valori << "cameraMatrix" << array_disp;
    valori.release();


}



cv::Mat ossimOpenCvDisparityMapGenerator::getDisp()
{

    return array_disp;

}

