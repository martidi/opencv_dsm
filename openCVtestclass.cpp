//----------------------------------------------------------------------------
//
// License:  See top level LICENSE.txt file.
//
// File: openCVtestclass.cpp
//
// Author:  Martina Di Rita
//
// Description: Class providing OpenCV functions for DSM extraction
//
//----------------------------------------------------------------------------

#include <ossim/elevation/ossimElevManager.h>
#include <ossim/imaging/ossimImageSource.h>
#include <ossim/imaging/ossimTiffWriter.h>
#include <ossim/imaging/ossimImageDataFactory.h>

#include "openCVtestclass.h"
#include "ossimOpenCvTPgenerator.h"
#include "ossimOpenCvDisparityMapGenerator.h"

#include <ossim/base/ossimArgumentParser.h>
#include <ossim/base/ossimApplicationUsage.h>
#include <ossim/base/ossimStringProperty.h>
#include <ossim/base/ossimIrect.h>

#include <ossim/point_cloud/ossimPointCloudImageHandler.h>
#include <ossim/point_cloud/ossimGenericPointCloudHandler.h>

#include <opencv2/highgui/highgui.hpp>
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
	// ****************************
	// Activate for Wallis filter	
	// ****************************			
    //master_mat = wallis(master_mat);
    //slave_mat = wallis(slave_mat);
	//		  	
   	// ****************************

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

	ossimOpenCvDisparityMapGenerator* dense_matcher = new ossimOpenCvDisparityMapGenerator();				
	out_disp = dense_matcher->execute(master_mat_8U, slave_mat_warp); 
	
	null_disp_threshold = (dense_matcher->minimumDisp)+0.5;	

	return true;
}

ossimRefPtr<ossimImageData> openCVtestclass::computeDSM(double mean_conversionF, ossimElevManager* elev, ossimImageGeometry* master_geom)
{
    // for along-track images
    cv::transpose(out_disp, out_disp);
    cv::flip(out_disp, out_disp, 0);

    // Creation of an OSSIM tiff
    //vector<ossimGpt> image_points;  // Need to fill this vector array

    out_disp.convertTo(out_disp, CV_64F);
    out_disp = ((out_disp/16.0)) / mean_conversionF;

    cout<< " " << endl << "DSM GENERATION \t wait few minutes..." << endl;
    cout << "null_disp_threshold"<< null_disp_threshold<< endl;

    for(int i=0; i< out_disp.rows; i++)
    {
        for(int j=0; j< out_disp.cols; j++)
        {
            ossimDpt image_pt(j,i);
            ossimGpt world_pt;

            master_geom->localToWorld(image_pt, world_pt);

            ossim_float64 hgtAboveMSL = elev->getHeightAboveMSL(world_pt);

            //ossim_float64 hgtAboveMSL =  elev->getHeightAboveEllipsoid(world_pt); //Augusta site
            if(out_disp.at<double>(i,j) >= null_disp_threshold/abs(mean_conversionF))
            {
                out_disp.at<double>(i,j) += hgtAboveMSL;


                //hgtAboveMSL += out_disp.at<double>(i,j);

                //world_pt.height(hgtAboveMSL);

                // image_points.push_back(world_pt);
                // cout <<"punti"<<image_points[i]<<endl;
            }
            //To fill holes with DSM coarse
            /*else
            {
                out_disp.at<double>(i,j) = hgtAboveMSL;
            }*/
        }
    }

    // Set the destination image size:
    ossimIpt image_size (out_disp.cols , out_disp.rows);
    ossimRefPtr<ossimImageData> outImage = ossimImageDataFactory::instance()->create(0, OSSIM_FLOAT32, 1, image_size.x, image_size.y);

    if(outImage.valid())
       outImage->initialize();
   // else
     //  return -1;

    for (int i=0; i< out_disp.cols; i++) // for every column
    {
        for(int j=0; j< out_disp.rows; j++) // for every row
        {
            outImage->setValue(i,j,out_disp.at<double>(j,i));
        }
    }


/*
    cv::Mat out_16bit_disp = cv::Mat::zeros (out_disp.size(),CV_64F);
    out_disp.convertTo(out_disp, CV_64F);
    //cout <<  "fattore di conv" << mean_conversionF << endl;
    out_16bit_disp = ((out_disp/16.0)) / mean_conversionF;
    //out_16bit_disp = (out_disp/16.0) * mean_conversionF;
    cout<< " " << endl << "DSM GENERATION \t wait few minutes..." << endl;
    cout << "null_disp_threshold"<< null_disp_threshold<< endl;
    for(int i=0; i< out_16bit_disp.rows; i++)
    {
        for(int j=0; j< out_16bit_disp.cols; j++)
        {
            ossimDpt image_pt(j,i);
            ossimGpt world_pt;
            master_geom->localToWorld(image_pt, world_pt);
            ossim_float64 hgtAboveMSL =  elev->getHeightAboveMSL(world_pt);

            if(out_16bit_disp.at<double>(i,j) <= null_disp_threshold/abs(mean_conversionF))
            //if(out_16bit_disp.at<double>(i,j) <= null_disp_threshold*abs(mean_conversionF))
            {
                out_16bit_disp.at<double>(i,j) = 0.0;
            }
            out_16bit_disp.at<double>(i,j) += hgtAboveMSL;
        }
    }
*/




/*
	cv::Mat intDSM; 
	// Conversion from float to integer to write and show
    out_disp.convertTo(intDSM, CV_16U);
	
    //cv::imwrite("Temp_DSM.tif", intDSM);
		
	double minVal, maxVal;
	minMaxLoc(intDSM, &minVal, &maxVal);
	intDSM.convertTo(intDSM, CV_8UC1, 255/(maxVal - minVal), -minVal*255/(maxVal - minVal));   
	
	cv::namedWindow("Temp_DSM", CV_WINDOW_NORMAL);
	cv::imshow("Temp_DSM", intDSM);
	cv::waitKey(0);	
	
    return true;*/
    return outImage;
}

bool openCVtestclass::writeDisparity(double conv_factor)
{
	// Rotation for along-track images
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
    double b = 1;
	
	// mf is the target value of the LOCAL MEAN in a i,j window [127.0-140.0]
	// an higher value will brighten the image
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

