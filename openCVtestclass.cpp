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
/*	// Create the OpenCV images
	master_mat.create(cv::Size(master->getWidth(), master->getHeight()), CV_16UC1);
	slave_mat.create(cv::Size(slave->getWidth(), slave->getHeight()), CV_16UC1);
	
	memcpy(master_mat.ptr(), (void*) master->getUshortBuf(), 2*master->getWidth()*master->getHeight());
	memcpy(slave_mat.ptr(), (void*) slave->getUshortBuf(), 2*slave->getWidth()*slave->getHeight());

	cout << "OSSIM->OpenCV image conversion done" << endl;
	
	// Rotation for along-track images
    cv::transpose(master_mat, master_mat);
	cv::flip(master_mat, master_mat, 1);
	
	cv::transpose(slave_mat, slave_mat);
    cv::flip(slave_mat, slave_mat, 1);*/
}


openCVtestclass::openCVtestclass(ossimRefPtr<ossimImageData> forward, ossimRefPtr<ossimImageData> nadir, ossimRefPtr<ossimImageData> backward)
{
	// Create the OpenCV images

    forward_mat.create(cv::Size(forward->getWidth(), forward->getHeight()), CV_16UC1);
	nadir_mat.create(cv::Size(nadir->getWidth(), nadir->getHeight()), CV_16UC1);
	backward_mat.create(cv::Size(backward->getWidth(), backward->getHeight()), CV_16UC1);


    images.push_back(nadir_mat);
    images.push_back(forward_mat);
    images.push_back(backward_mat);


    memcpy(images[0].ptr(), (void*) nadir->getUshortBuf(), 2*nadir->getWidth()*nadir->getHeight());
    memcpy(images[1].ptr(), (void*) forward->getUshortBuf(), 2*forward->getWidth()*forward->getHeight());
    memcpy(images[2].ptr(), (void*) backward->getUshortBuf(), 2*backward->getWidth()*backward->getHeight());

	
	cout << "OSSIM->OpenCV image conversion done" << endl;
	
	// Rotation for along-track images
    for(size_t i = 0; i < images.size(); i++)
        {
        cv::transpose(images[i], images[i]);
        cv::flip(images[i], images[i], 1);
        }
}


bool openCVtestclass::execute()
{			
	// ****************************
	// Activate for Wallis filter	
	// ****************************			
    //for(size_t i = 0; i < images.size(); i++)
    //{
      //  images[i] = wallis(images[i]);
    //}


    vector<cv::Mat> images_8U;

    for(size_t i = 0; i < images.size(); i++)
    {
        //cv::Mat pointer;
        double minVal_images= 0, maxVal_images= 0;

        images_8U.push_back(images[i]);
        minMaxLoc( images[i], &minVal_images, &maxVal_images );
        images[i].convertTo( images_8U[i], CV_8UC1, 255.0/(maxVal_images - minVal_images), -minVal_images*255.0/(maxVal_images - minVal_images));

        cv::namedWindow("Input image", CV_WINDOW_NORMAL);
        cv::imshow("Input image", images_8U[i]);
        cv::waitKey(0);

    }

    for(size_t i = 1; i < images.size(); i++)
    {

        ossimOpenCvTPgenerator* TPfinder = new ossimOpenCvTPgenerator(images_8U[0], images_8U[i]);
        TPfinder->run();

        cv::Mat slave_mat_warp = TPfinder->warp(images[i]);

        ossimOpenCvDisparityMapGenerator* dense_matcher = new ossimOpenCvDisparityMapGenerator();

        // to check the time necessary for Disp Map gen
        ossimNotify(ossimNotifyLevel_NOTICE) << "elapsed time in seconds: " << std::setiosflags(ios::fixed) << std::setprecision(3)
        << ossimTimer::instance()->time_s() << endl << endl;

        disparity_maps.push_back(dense_matcher->execute(images_8U[0], slave_mat_warp));

        ossimNotify(ossimNotifyLevel_NOTICE) << "elapsed time in seconds: " << std::setiosflags(ios::fixed) << std::setprecision(3)
        << ossimTimer::instance()->time_s() << endl << endl;

        null_disp_threshold = (dense_matcher->minimumDisp)+0.5;
    }

  //  writeDisparity(1.0);

	return true;
}


bool openCVtestclass::computeDSM(double mean_conversionF, ossimElevManager* elev, ossimImageGeometry* master_geom)
{
    vector<cv::Mat> disparity_maps_16bit;

    disparity_maps_16bit.resize(disparity_maps.size());

    cout << disparity_maps.size() << endl;
    for (unsigned int i = 0; i < disparity_maps.size(); ++i)
    {
        cv::transpose(disparity_maps[i], disparity_maps[i]);
        cv::flip(disparity_maps[i], disparity_maps[i], 0);

        disparity_maps[i].convertTo(disparity_maps[i], CV_64F);
        disparity_maps_16bit[i] = (disparity_maps[i]/16.0) / mean_conversionF;

        // A questo punto ho 2 mappe di disparità "metriche"
        // Devo fonderle
    }

    cout << disparity_maps_16bit[0].size() << endl;
    cout << disparity_maps_16bit[1].size() << endl;

    //cv::Mat temp = disparity_maps_16bit[0];

    cv::Mat fusedDisp = cv::Mat::zeros(disparity_maps_16bit[0].rows, disparity_maps_16bit[0].cols, CV_64F);

    cout<< " " << endl << "DSM GENERATION \t wait few minutes..." << endl;
    cout << "null_disp_threshold"<< null_disp_threshold<< endl;

    cout << fusedDisp.rows << endl;
    cout << fusedDisp.cols << endl;

    for (int i=0; i< disparity_maps_16bit[0].rows; i++) // per tutte le righe e le colonne della mappa di disparità
    {
        for(int j=0; j< disparity_maps_16bit[0].cols; j++)
        {
            int num=0.0;

            for (unsigned int k = 0; k < disparity_maps_16bit.size(); k++)
            {
                if(disparity_maps_16bit[k].at<double>(i,j) > null_disp_threshold/abs(mean_conversionF))
                {
                    fusedDisp.at<double>(i,j) += disparity_maps_16bit[k].at<double>(i,j);
                    num++;
                }
            }

            fusedDisp.at<double>(i,j)  = fusedDisp.at<double>(i,j) /num;

            // sommo la disparità metrica al dsm coarse
            ossimDpt image_pt(j,i);
            ossimGpt world_pt;
            master_geom->localToWorld(image_pt, world_pt);
            ossim_float64 hgtAboveMSL =  elev->getHeightAboveMSL(world_pt);
            //ossim_float64 hgtAboveMSL =  elev->getHeightAboveEllipsoid(world_pt); //Augusta site
            fusedDisp.at<double>(i,j) += hgtAboveMSL;
        }
   }

/*
            double minVal, maxVal;
            minMaxLoc( fusedDisp, &minVal, &maxVal );
            fusedDisp.convertTo( fusedDisp, CV_8UC1, 255/(maxVal - minVal), -minVal*255/(maxVal - minVal));
            cv::namedWindow( "Fused Disparity", CV_WINDOW_NORMAL );
            cv::imshow( "Fused Disparity", fusedDisp);

    for (int i=0; i< disparity_maps_16bit[0].rows; i++) // per tutte le righe e le colonne della mappa di disparità
    {
        for(int j=0; j< disparity_maps_16bit[0].cols; j++)
        {
            // sommo la disparità metrica al dsm coarse
            ossimDpt image_pt(j,i);
            ossimGpt world_pt;
            master_geom->localToWorld(image_pt, world_pt);
            ossim_float64 hgtAboveMSL =  elev->getHeightAboveMSL(world_pt);
            //ossim_float64 hgtAboveMSL =  elev->getHeightAboveEllipsoid(world_pt); //Augusta site
            fusedDisp.at<double>(i,j) += hgtAboveMSL;
         }
    }
*/






/*	// Conversion from OpenCV to OSSIM images
	
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
    fusedDisp.convertTo(intDSM, CV_16U);
	cv::imwrite("Temp_DSM.tif", intDSM);
		
    double minVal, maxVal;
	minMaxLoc(intDSM, &minVal, &maxVal);
	intDSM.convertTo(intDSM, CV_8UC1, 255/(maxVal - minVal), -minVal*255/(maxVal - minVal));   
	
	cv::namedWindow("Temp_DSM", CV_WINDOW_NORMAL);
	cv::imshow("Temp_DSM", intDSM);
	cv::waitKey(0);	
	
	return true;
}


/*
bool openCVtestclass::writeDisparity(double conv_factor)
{
    for(int i = 0; i < disparity_maps.size(); i++)
    {
        cv::transpose(disparity_maps[i], disparity_maps[i]);
        cv::flip(disparity_maps[i], disparity_maps[i], 0);

       // disparity_maps[i] = (disparity_maps[i]/16.0) * conv_factor;
        disparity_maps[i] = (disparity_maps[i]/16.0) * 1;


        ossimString name = "SGM Disparity_";
       name.append(i);
        name.append(".tif");

        cv::imwrite(name.string(), disparity_maps[i]);


    }
	return true;
}*/


cv::Mat openCVtestclass::wallis(cv::Mat image)
{
	cout <<"Filtering images..."<< endl;
   
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
    double mf = 140.0; //era 127.0
	
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
	cv::namedWindow("Normal_image", CV_WINDOW_NORMAL);
	cv::imshow("Normal_image", image );	
		
	cv::namedWindow("Filtered_image", CV_WINDOW_NORMAL);
	cv::imshow("Filtered_image", Filtered_image );
	
	cv::waitKey(0);	
    return Filtered_image;
}


