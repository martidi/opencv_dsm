//----------------------------------------------------------------------------
//
// License:  See top level LICENSE.txt file.
//
// File: openCVtestclass.cpp
//
// Author:  Martina Di Rita
//
// Description: Class provides OpenCV functions for DSM extraction
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

#include "openCVtestclass.h"
#include "ossimOpenCvTPgenerator.h"
#include "ossimOpenCvDisparityMapGenerator.h"

#include <ossim/base/ossimArgumentParser.h>
#include <ossim/base/ossimApplicationUsage.h>

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

openCVtestclass::openCVtestclass()
{
	
}

openCVtestclass::openCVtestclass(ossimRefPtr<ossimImageData> master, ossimRefPtr<ossimImageData> slave)
{
	// Create the OpenCV images
	master_mat.create(cv::Size(master->getWidth(), master->getHeight()), CV_16UC1);
	slave_mat.create(cv::Size(slave->getWidth(), slave->getHeight()), CV_16UC1);
	
	memcpy(master_mat.ptr(), (void*) master->getUshortBuf(), 2*master->getWidth()*master->getHeight());
	memcpy(slave_mat.ptr(), (void*) slave->getUshortBuf(), 2*slave->getWidth()*slave->getHeight());

	cout << "OSSIM->OpenCV image conversion done" << endl;
	
	// Rotation for along-track images
	cv::transpose(master_mat, master_mat);
	cv::flip(master_mat, master_mat, 1);
	
	cv::transpose(slave_mat, slave_mat);
	cv::flip(slave_mat, slave_mat, 1);	
}


bool openCVtestclass::execute()
{			
	//openCVtestclass::wallis *filter = openCVtestclass::wallis(master_mat);
	//test->wallis(img_slave);
	
	// Wallis filter				
   	//master_mat = wallis(master_mat);	
   	//slave_mat = wallis(slave_mat);		  	
   	
	double minVal_master, maxVal_master, minVal_slave, maxVal_slave;
	cv::Mat master_mat_8U;
	cv::Mat slave_mat_8U;  
      
   	minMaxLoc( master_mat, &minVal_master, &maxVal_master );
   	minMaxLoc( slave_mat, &minVal_slave, &maxVal_slave );
	master_mat.convertTo( master_mat_8U, CV_8UC1, 255.0/(maxVal_master - minVal_master), -minVal_master*255.0/(maxVal_master - minVal_master));
	slave_mat.convertTo( slave_mat_8U, CV_8UC1, 255.0/(maxVal_slave - minVal_slave), -minVal_slave*255.0/(maxVal_slave - minVal_slave)); 
	
	ossimOpenCvTPgenerator* TPfinder = new ossimOpenCvTPgenerator(master_mat_8U, slave_mat_8U);
	TPfinder->run();
	
	cv::Mat slave_mat_warp = TPfinder->warp(slave_mat);
	
	//cv::Ptr<cv::CLAHE> filtro = cv::createCLAHE(3.0);
	//filtro->apply(master_mat_8U, master_mat_8U); 
	//filtro->apply(slave_mat_warp, slave_mat_warp);
    
	//cv::imwrite("Master_8bit_bSGM.tif",  master_mat_8U);
	//cv::imwrite("Slave_8bit_bSGM.tif",  slave_mat_warp);
    	
	ossimOpenCvDisparityMapGenerator* dense_matcher = new ossimOpenCvDisparityMapGenerator();
	
	//***
	// Abilitate for computing disparity on different scales 
	/*
	double fscale = 1.0/2.0;
	cv::resize(master_mat_8U, master_mat_8U, cv::Size(), fscale, fscale, cv::INTER_AREA );
	cv::resize(slave_mat_warp, slave_mat_warp, cv::Size(), fscale, fscale, cv::INTER_AREA );	
	cv::namedWindow( "Scaled master", CV_WINDOW_NORMAL );
	cv::imshow( "Scaled master", master_mat_8U);
	cv::namedWindow( "Scaled slave", CV_WINDOW_NORMAL );
	cv::imshow( "Scaled slave", slave_mat_warp);
	*/
	//***
	
	//master_mat.convertTo(master_mat, CV_16U);
	
	//slave_mat_warp.convertTo(slave_mat_warp, CV_16U);
	
	out_disp = dense_matcher->execute(master_mat_8U, slave_mat_warp); 
	
	// Abilitate for computing disparity on different scales 	
	//out_disp = out_disp/fscale; // to consider the scale factor also in the disparity values
	//cv::resize(out_disp, out_disp, cv::Size(), 1.0/fscale, 1.0/fscale, cv::INTER_AREA );
	
	return true;
}

bool openCVtestclass::computeDSM(double mean_conversionF, ossimElevManager* elev, ossimImageGeometry* master_geom)
{
	cv::transpose(out_disp, out_disp);
	cv::flip(out_disp, out_disp, 0);
    
	cv::Mat out_16bit_disp = cv::Mat::zeros (out_disp.size(),CV_64F);
	out_disp.convertTo(out_disp, CV_64F);
	out_16bit_disp = (out_disp/16.0) / mean_conversionF;
	
	cout<< "DSM GENERATION \t wait few minutes ..." << endl;

	for(int i=0; i< out_16bit_disp.rows; i++)
	{
		for(int j=0; j< out_16bit_disp.cols; j++)
		{
			ossimDpt image_pt(j,i);
			ossimGpt world_pt;     
			master_geom->localToWorld(image_pt, world_pt);
			ossim_float64 hgtAboveMSL =  elev->getHeightAboveMSL(world_pt);
			if(out_16bit_disp.at<double>(i,j) <= -7.5/mean_conversionF)
			{ 
				out_16bit_disp.at<double>(i,j) = 0.0;
			}
			out_16bit_disp.at<double>(i,j) += hgtAboveMSL;
		}
	}
/*
	// Conversion from OpenCV to OSSIM images   
	
	//ossimRefPtr<ossimImageData> disp_ossim = disp_ossim_handler->getSize();
	cout << "OpenCV->OSSIM image conversion done_1" << endl;	
	
	//ossimImageHandler* disp_ossim_handler = ossimImageHandlerRegistry::instance() ->open(ossimFilename("../../../../img_data/ZY_3/ZY3_NAD_E11.5_N46.5_20120909_L1A0000657936/ZY3_TLC_E11_5_N46_5_20120909_L1A0000657936_NAD.TIF"));
	//ossimIrect bounds_disp = disp_ossim_handler->getBoundingRect(0); 			
	ossimRefPtr<ossimImageData> disp_ossim = new ossimImageData(NULL, OSSIM_DOUBLE, 1, out_16bit_disp.cols, out_16bit_disp.rows );
	//disp_ossim->setWidthHeight(out_16bit_disp.cols,out_16bit_disp.rows);    
		
	//ossimImageHandler* master_handler = ossimImageHandlerRegistry::instance()->open(ossimFilename(argv[3]));  	
	//ossimIrect bounds_master = master_handler->getBoundingRect(0); 		
	//ossimRefPtr<ossimImageData> img_master = master_handler->getTile(bounds_master, 0);     
	//ossimRefPtr<ossimImageGeometry> raw_slave_geom = raw_slave_handler->getImageGeometry(); 	
	cout << "OpenCV->OSSIM image conversion done_2" << endl;		
	
	//master_mat.create(cv::Size(master->getWidth(), master->getHeight()), CV_16UC1);
	//slave_mat.create(cv::Size(slave->getWidth(), slave->getHeight()), CV_16UC1);

	memcpy((void*)disp_ossim->getDoubleBuf(), (void*)out_16bit_disp.ptr(), out_16bit_disp.cols*out_16bit_disp.rows);
	//memcpy(slave_mat.ptr(), (void*) slave->getUshortBuf(), 2*slave->getWidth()*slave->getHeight());
	
	//disp_ossim = disp_ossim->getDoubleBuf();
	
	cout << "OpenCV->OSSIM image conversion done_3" << endl;
*/ 	
	cv::Mat intDSM; 
	// Conversion from float to integer to write and show
	out_16bit_disp.convertTo(intDSM, CV_16U);
	
	cv::imwrite("Temp_DSM.tif", intDSM);
		
	double minVal, maxVal;
	minMaxLoc(intDSM, &minVal, &maxVal);
	intDSM.convertTo(intDSM, CV_8UC1, 255/(maxVal - minVal), -minVal*255/(maxVal - minVal));   
	
	cv::namedWindow("Temp_DSM", CV_WINDOW_NORMAL);
	cv::imshow("Temp_DSM", intDSM);
	cv::waitKey(0);	
	
	return true;
}

bool openCVtestclass::writeDisparity(double conv_factor)
{
	cv::transpose(out_disp, out_disp);
	cv::flip(out_disp, out_disp, 0);
    
	out_disp = (out_disp/16.0) * conv_factor;
	cv::imwrite("mDisparity.jpg", out_disp);
	
	return true;
}


cv::Mat openCVtestclass::wallis(cv::Mat image)
{
	cout <<"Filtering images..."<< endl;
	//cv::Mat image =  imread(raw_image, CV_LOAD_IMAGE_UNCHANGED );
	// Check for invalid input	
    /*if( image.empty() )                      
    {
        cout <<  "Could not open or find the image" << endl ;
        return -1;
    }*/
    
	int n = image.cols;	
	int m = image.rows;
	
	// Block dimension in i and j
	int dim_n = 40, dim_m = 40;

	int N_Block = n/dim_n;
	cout<<"N_Block\t"<<N_Block<<endl;
	int M_Block = m/dim_m;
	cout<<"M_Block\t"<<M_Block<<endl;
	int resto_n = n%dim_n;
	int resto_m = m%dim_m;
	
	//alloco due array lunghi N_Block e M_Block
	int dimension_x[N_Block];
	int dimension_y[M_Block];
	
	dim_n = dim_n + resto_n/N_Block;
	dim_m = dim_m + resto_m/M_Block;
	resto_n = n%dim_n;
	resto_m = m%dim_m;
	
	int i;
	for (i=0; i < N_Block; i++)
	{	
		if (resto_n>0)
		{
		dimension_x[i] = dim_n+1;
		resto_n--;
		}
		else
		{
		dimension_x[i] = dim_n;	
		}		
		//printf("%d\n", dimension_x[i]);
	}
	
	
	for (i=0; i < M_Block; i++)
	{	
		if (resto_m>0)
		{
		dimension_y[i] = dim_m+1;
		resto_m--;
		}
		else
		{
		dimension_y[i] = dim_m;	
		}		
		//printf("%d\n", dimension_y[i]);
	}
	
	// c is the CONTRAST expansion constant [0.7-1.0]
	// to reduce the enhancement of noise c should be reduced
	// it has a much stronger influence on noise than sf
	// lower values produce very little contrast and detail
	// values closer to 1 produce a highly contrasted image with greater detail 
	double c = 0.8; 
	
	// sf is the target value of the LOCAL STANDARD DEVIATION in a i,j window [50.0-80.0]
	// the value of sf should decrease with decreasing window dimensions(i,j) to avoid increase of noise
	// it decides the contrast of image; higher values result in a greater contrast stretch
	// producing higher local contrast and greater detail 
	double sf =  80.0;
	
	// b is the BRIGHTNESS forcing constant [0.5-1.0]
	// to keep primary image gray mean b has to be small
	// 0 will keep the original pixel values
	// 1 will generate an output image equal to the wallis filter specified
	double b = 0.9;
	
	// mf is the target value of the LOCAL MEAN in a i,j window [127.0-140.0]
	// an higher value wil brighten the image
	double mf = 127.0;
	
	int px = 0, py = 0;
	
	//ricorda che devi invertire M_Block e N_Block perchè l'immagine è ruotata
	cv::Mat Coeff_R0 = cv::Mat::zeros(M_Block, N_Block, CV_64F);
	//(N_Block, M_Block, CV_64F);
	cv::Mat Coeff_R1 = cv::Mat::zeros(M_Block, N_Block, CV_64F);
	cout <<"Coeff_R0"<<	Coeff_R0.size() <<endl;	
	cout <<"Coeff_R1"<<	Coeff_R1.size() <<endl;					
	// computing mean and standard deviation in every (dim_n*dim_m) window
	for(int i=0; i<N_Block; i++)
	{
		py = 0;	
		for(int j=0; j<M_Block; j++)
		{	
			cv::Mat block = image(cv::Rect(px,py,dimension_x[i],dimension_y[j]));
			cv::Scalar mean, stDev;	
			cv::meanStdDev(block, mean, stDev);	
			
			py += dimension_y[j];	
			
			double r1 = c*sf/(c*stDev.val[0] + (1-c)*sf);				//Baltsavias
			//double r1 = c*stDev.val[0]/(c*stDev.val[0] + (sf/c));		//Fraser	
			//double r1 = c*sf/(c*stDev.val[0] + (sf/c));				//Xiao				
			double r0 = b*mf + (1 - b - r1)*mean.val[0];	
							
			Coeff_R1.at<double>(j,i) = r1;			
			Coeff_R0.at<double>(j,i) = r0;
		}
		px += dimension_x[i];
	}	
	
	cv::resize(Coeff_R1, Coeff_R1, image.size(), cv::INTER_LINEAR );		
	cv::resize(Coeff_R0, Coeff_R0, image.size(), cv::INTER_LINEAR );	
	
	cout <<"Checking image resize..."<<endl;	
	cout <<"Coeff_R0"<<	Coeff_R0.size() <<endl;	
	cout <<"Coeff_R1"<<	Coeff_R1.size()  <<endl;	
	cout <<"Image type\t"<<	image.type() <<endl;
	cout <<"Image depth\t"<<	image.depth() <<endl;		
	cout <<"Image size\t"<<	image.size() <<endl;
			
	image.convertTo(image, CV_64F);
					 
	cv::Mat Filtered_image = cv::Mat::zeros(cv::Size(N_Block, M_Block), CV_64F);
	cv::multiply(image, Coeff_R1, Filtered_image ); 	
	cv::add(Filtered_image, Coeff_R0, Filtered_image);	
				
	double minVal, maxVal;
	minMaxLoc( Filtered_image, &minVal, &maxVal );
	Filtered_image.convertTo( Filtered_image, CV_8UC1, 255/(maxVal - minVal), -minVal*255/(maxVal - minVal));   
	minMaxLoc( image, &minVal, &maxVal );
	image.convertTo( image, CV_8UC1, 255/(maxVal - minVal), -minVal*255/(maxVal - minVal));  	
	
	// showing the result
	/*cv::namedWindow("Normal_image", CV_WINDOW_NORMAL);
	cv::imshow("Normal_image", image );	
		
	cv::namedWindow("Filtered_image", CV_WINDOW_NORMAL);
	cv::imshow("Filtered_image", Filtered_image );
	
	cv::waitKey(0);*/	
	return Filtered_image;				
}


/*void openCVtestclass::addArguments(ossimArgumentParser& ap)
{
   ossimString usageString = ap.getApplicationName();
   usageString += " [option]... [input-option]... <input-file(s)> <output-file>\nNote at least one input is required either from one of the input options, e.g. --input-dem <my-dem.hgt> or adding to command line in front of the output file in which case the code will try to ascertain what type of input it is.\n\nAvailable traces:\n-T \"ossimChipperUtil:debug\"   - General debug trace to standard out.\n-T \"ossimChipperUtil:log\"     - Writes a log file to output-file.log.\n-T \"ossimChipperUtil:options\" - Writes the options to output-file-options.kwl.";

   ossimApplicationUsage* au = ap.getApplicationUsage();
   
   au->setCommandLineUsage(usageString);
    
   au->setDescription(ap.getApplicationName()+" Utility application for generating elevation products from dem data.");
   
   au->addCommandLineOption("--azimuth", "<azimuth>\nhillshade option - Light source azimuth angle for bump shade.\nRange: 0 to 360, Default = 180.0");

   au->addCommandLineOption( "-b or --bands <n,n...>", "Use the specified bands in given order: e.g. \"3,2,1\" will select bands 3, 2 and 1 of the input image.\nNote: it is 1 based" );

   au->addCommandLineOption("--central-meridian","<central_meridian_in_decimal_degrees>\nNote if set this will be used for the central meridian of the projection.  This can be used to lock the utm zone.");
}*/
