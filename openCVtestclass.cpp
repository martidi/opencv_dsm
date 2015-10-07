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
#include <ossim/imaging/ossimMemoryImageSource.h>
#include <ossim/point_cloud/ossimGenericPointCloudHandler.h>
#include <ossim/point_cloud/ossimPointCloudImageHandler.h>
#include <ossim/base/ossimStringProperty.h>

#include "openCVtestclass.h"
#include "ossimOpenCvTPgenerator.h"
#include "ossimOpenCvDisparityMapGenerator.h"
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv/cv.h"
#include <opencv2/highgui/highgui.hpp>
// Note: These are purposely commented out to indicate non-use.
// #include <opencv2/nonfree/nonfree.hpp>
// #include <opencv2/nonfree/features2d.hpp>
// Note: These are purposely commented out to indicate non-use.
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
        disparity_maps.push_back(dense_matcher->execute(images_8U[0], slave_mat_warp));

        null_disp_threshold = (dense_matcher->minimumDisp)+0.5;
    }

  //  writeDisparity(1.0);
	return true;
}


ossimRefPtr<ossimImageData> openCVtestclass::computeDSM(vector<double> mean_conversionF, ossimElevManager* elev, ossimImageGeometry* master_geom)
{
    //vector<cv::Mat> disparity_maps_16bit;
    vector<cv::Mat> disparity_maps_8bit;
    //vector<ossimGpt> image_points;
    double minVal, maxVal;

    //disparity_maps_16bit.resize(disparity_maps.size());
    disparity_maps_8bit.resize(disparity_maps.size());

    cout << disparity_maps.size() << endl;
    for (unsigned int k = 0; k < disparity_maps.size(); ++k)
    {
        cv::transpose(disparity_maps[k], disparity_maps[k]);
        cv::flip(disparity_maps[k], disparity_maps[k], 0);

        disparity_maps[k].convertTo(disparity_maps[k], CV_64F);
        disparity_maps[k] = (disparity_maps[k]/16.0); // / mean_conversionF[k];

        // A questo punto ho 2 mappe di disparità ruotate e corrette in pix (/16.0) per opencv
        // Devo fonderle e renderle "metriche"

        //To show disparity maps
        minMaxLoc( disparity_maps[k], &minVal, &maxVal );
        disparity_maps[k].convertTo(disparity_maps_8bit[k], CV_8UC1, 255/(maxVal - minVal), -minVal*255/(maxVal - minVal));
        cout << "min\t" << minVal << " " << "max\t" << maxVal << endl;

        ossimString win_name = "Disparity_Map_";
        cv::namedWindow( win_name+ossimString(k), CV_WINDOW_NORMAL );
        cv::imshow( win_name+ossimString(k), disparity_maps_8bit[k]);
    }

    cout << disparity_maps[0].size() << endl;
    cout << disparity_maps[1].size() << endl;

    cv::Mat error_disp = (disparity_maps[0]/mean_conversionF[0]) - (disparity_maps[1]/mean_conversionF[1]); //quando divido per il fattore di conversione diventa metrico
    minMaxLoc(error_disp, &minVal, &maxVal);
    error_disp.convertTo(error_disp, CV_8UC1, 255/(maxVal - minVal), -minVal*255/(maxVal - minVal));
    cv::namedWindow("Error disp", CV_WINDOW_NORMAL);
    cv::imshow("Error disp", error_disp);
    cv::waitKey(0);

    cv::Mat fusedDisp = cv::Mat::zeros(disparity_maps[0].rows, disparity_maps[0].cols, CV_64F);


    cout<< " " << endl << "DSM GENERATION \t wait few minutes..." << endl;
    cout << "null_disp_threshold\t"<< null_disp_threshold<< endl;

    //cout << "n° rows\t" << fusedDisp.rows << endl;
    //cout << "n° columns\t" << fusedDisp.cols << endl;

    for (int i=0; i< disparity_maps[0].rows; i++) // for every row
    {
        for(int j=0; j< disparity_maps[0].cols; j++) // for every column
        {
            int num=0.0;

            if(fabs(error_disp.at<double>(i,j)) < 5)
            {
                for (unsigned int k = 0; k < disparity_maps.size(); k++)  // for every disparity map
                {
                    if(disparity_maps[k].at<double>(i,j) > null_disp_threshold) // sto togliendo i valori minori della threshold
                    {

                        fusedDisp.at<double>(i,j) += disparity_maps[k].at<double>(i,j)/mean_conversionF[k]; // "metric" disparity
                        //outImage.operator =(  outImage->setValue(i,j,disparity_maps[k].at<double>(i,j)/mean_conversionF[k]));
                        num++;
                    }
                }
                fusedDisp.at<double>(i,j)  = fusedDisp.at<double>(i,j) /num;
               // outImage->setValue(i,j,outImage/num);
            }
            else
            {
               // outImage->setValue(i,j,disparity_maps[1].at<double>(i,j)/mean_conversionF[1]);
                fusedDisp.at<double>(i,j) = disparity_maps[1].at<double>(i,j)/mean_conversionF[1];
            }

            // sum between "metric" disparity and coarse dsm
            ossimDpt image_pt(j,i);
            ossimGpt world_pt;
            master_geom->localToWorld(image_pt, world_pt);
            ossim_float64 hgtAboveMSL =  elev->getHeightAboveMSL(world_pt);
            //ossim_float64 hgtAboveMSL =  elev->getHeightAboveEllipsoid(world_pt); //Augusta site
            fusedDisp.at<double>(i,j) += hgtAboveMSL; // to work with millimeters
            //hgtAboveMSL += fusedDisp.at<double>(i,j);
            //world_pt.height(hgtAboveMSL);
            //image_points.push_back(world_pt);
        }
    }

    // Set the destination image size:
    ossimIpt image_size (fusedDisp.cols , fusedDisp.rows);
    ossimRefPtr<ossimImageData> outImage = ossimImageDataFactory::instance()->create(0, OSSIM_FLOAT64, 1, image_size.x, image_size.y);

    if(outImage.valid())
       outImage->initialize();
   // else
     //  return -1;

    for (int i=0; i< fusedDisp.cols; i++) // for every row
    {
        for(int j=0; j< fusedDisp.rows; j++) // for every column
        {
            outImage->setValue(i,j,fusedDisp.at<double>(j,i));
        }
    }


/*
    // come faccio a prendere (o inserire) il valore nel punto i,j?
    outImage = outImage + outImage->setValue(i,j,disparity_maps[k].at<double>(i,j)/mean_conversionF[k]);

    outImage->setValue(i,j,disparity_maps[1].at<double>(i,j)/mean_conversionF[1]);

    outImage->setValue(i,j,disparity_maps[k].at<double>(i,j)/mean_conversionF[k]);
    outImage->loadTile();
*/


    /*

    //Conversion from m to cm
    fusedDisp = fusedDisp*100.0;

    // Conversion from float to integer to show and write
    cv::Mat intDSM;
    fusedDisp.convertTo(intDSM, CV_16U);
    cv::imwrite("DSM_float.tif", intDSM);

	minMaxLoc(intDSM, &minVal, &maxVal);
	intDSM.convertTo(intDSM, CV_8UC1, 255/(maxVal - minVal), -minVal*255/(maxVal - minVal));   
	
    cv::namedWindow("Temp_DSM", CV_WINDOW_NORMAL);
	cv::imshow("Temp_DSM", intDSM);
    cv::waitKey(0);	*/
    return outImage;
    //return true;
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


