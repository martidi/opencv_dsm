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
#include "ossimImagePreprocess.h"
#include "ossimOpenCvTPgenerator.h"
#include "ossimDispMerging.h"
#include "ossimOpenCvDisparityMapGenerator.h"

#include <ossim/imaging/ossimMemoryImageSource.h>
#include "ossim/imaging/ossimImageHandlerRegistry.h"
#include "ossim/imaging/ossimImageHandler.h"
#include "ossim/imaging/ossimImageFileWriter.h"
#include "ossim/imaging/ossimImageWriterFactoryRegistry.h"

#include <opencv2/highgui/highgui.hpp>

ossimImagePreprocess::ossimImagePreprocess()
{
	
}


cv::Mat ossimImagePreprocess::wallis(cv::Mat image)
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
        cv::namedWindow("Non_filtered_image", CV_WINDOW_NORMAL);
        cv::imshow("Non_filtered_image", image );
		
	cv::namedWindow("Filtered_image", CV_WINDOW_NORMAL);
	cv::imshow("Filtered_image", Filtered_image );
	
	cv::waitKey(0);	

        return Filtered_image;
}


