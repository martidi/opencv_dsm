//----------------------------------------------------------------------------
//
// License:  See top level LICENSE.txt file.
//
// File: ossimOpenCvTPgenerator.cpp
//
// Author:  Martina Di Rita
//
// Description: Class provides a TPs generator
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

#include "ossimOpenCvTPgenerator.h"

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

ossimOpenCvTPgenerator::ossimOpenCvTPgenerator()
{

}

ossimOpenCvTPgenerator::ossimOpenCvTPgenerator(cv::Mat master, cv::Mat slave)
{
	master_mat = master;
	slave_mat = slave;
}

cv::Mat ossimOpenCvTPgenerator::estRT(std::vector<cv::Point2f> master, std::vector<cv::Point2f> slave)
{
	size_t m = master.size();
	
	if ( master.size() != slave.size() ) 
	{
		throw 0;
	}
    
	// Computing barycentric coordinates
	
	cv::Scalar mean_master_x , mean_master_y, mean_slave_x, mean_slave_y;
	cv::Scalar stDev_master_x, stDev_master_y, stDev_slave_x, stDev_slave_y;
	
	cv::Mat components_matrix = cv::Mat::zeros(m,4, CV_64F);
	for(size_t i = 0; i < m; i++)
	{
		components_matrix.at<double>(i,0) = master[i].x;
		components_matrix.at<double>(i,1) = master[i].y;
		components_matrix.at<double>(i,2) = slave[i].x;
		components_matrix.at<double>(i,3) = slave[i].y;	
	}
	cv::meanStdDev(components_matrix.col(0), mean_master_x, stDev_master_x);				
	cv::meanStdDev(components_matrix.col(1), mean_master_y, stDev_master_y); 
	cv::meanStdDev(components_matrix.col(2), mean_slave_x, stDev_slave_x); 
	cv::meanStdDev(components_matrix.col(3), mean_slave_y, stDev_slave_y); 
		
	master_x = mean_master_x.val[0];
	master_y = mean_master_y.val[0];
	slave_x = mean_slave_x.val[0];
	slave_y	= mean_slave_y.val[0];		
	
	
	cout << "mean_x_master = " << master_x << endl
		 << "mean_y_master = " << master_y << endl
		 << "mean_x_slave = "  << slave_x  << endl
		 << "mean_y_slave = "  << slave_y  << endl << endl; 

			
	std::vector<cv::Point2f> bar_master, bar_slave;

	for (size_t i = 0; i < m; i++)
	{
		cv::Point2f pt1;
		cv::Point2f pt2;

		pt1.x = master[i].x - master_x;
		pt1.y = master[i].y - master_y;

		pt2.x = slave[i].x - slave_x;
		pt2.y = slave[i].y - slave_y;
        
        //cout << pt1.x << "\t" << pt1.y << "\t" << pt2.x << "\t" << pt2.y<< "\t" << pt1.y-pt2.y<< endl;

        bar_master.push_back(pt1);
        bar_slave.push_back(pt2);
	}
	
	/// ***rigorous model start***
	cv::Mat x_approx = cv::Mat::zeros (2+m,1,6);
	cv::Mat result = cv::Mat::zeros (2+m, 1, 6);
	cv::Mat A = cv::Mat::zeros(2*m,2+m,6);
	cv::Mat B = cv::Mat::zeros(2*m,1,6);
    
	cv::Mat trX;
	double disp_median = master_x-slave_x;
	
	for (int j= 0; j <1; j++)
	{
		for (size_t i=0; i < m ; i++)
		{
			A.at<double>(2*i,0) = bar_slave[i].y;
			A.at<double>(2*i,1) = 0.0;
			A.at<double>(2*i,2+i) = 1.0;
			
			A.at<double>(2*i+1,0) = -bar_slave[i].x- disp_median;
			A.at<double>(2*i+1,1) = 1.0;
			A.at<double>(2*i+1,2+i) = 0.0;
			
			B.at<double>(2*i,0)   = bar_master[i].x - cos(x_approx.at<double>(0,0))*(bar_slave[i].x  +disp_median+ x_approx.at<double>(2+i,0)) 
													 - sin(x_approx.at<double>(0,0))*bar_slave[i].y;
 			B.at<double>(2*i+1,0) = bar_master[i].y + sin(x_approx.at<double>(0,0))*(bar_slave[i].x  +disp_median+ x_approx.at<double>(2+i,0)) 
													 - cos(x_approx.at<double>(0,0))*bar_slave[i].y - x_approx.at<double>(1,0);
		}
	
		cv::solve(A, B, result, cv::DECOMP_SVD);
		x_approx = x_approx+result;
		
		cv::Mat trX;
		cv::transpose(result, trX);

		//cout << "Result matrix "<< endl;
		//cout << trX << endl << endl;
	
		cv::transpose(x_approx, trX);

		//cout << "X approx matrix iteration " << j << endl;
		//cout << trX << endl << endl;
	}
		
	//cout << "Difference " << endl;	
	//cout << A*x_approx-B << endl;
	
	//cout << A << endl;
	//cout << B << endl;
	
	trX = A*x_approx-B;
	
	/*for(size_t i=0; i < m ; i++)
	{
		cout << master[i].y <<"\t" << trX.at<double>(2*i+1,0) <<endl;
	}*/
	
	// rotation is applied in the TPs barycenter	
	cv::Point2f pt(slave_x , slave_y);
	cv::Mat r = cv::getRotationMatrix2D(pt, -x_approx.at<double>(0,0)*180.0/3.141516, 1.0);
 	r.at<double>(1,2) += x_approx.at<double>(1,0) - master_y + slave_y;  
 	    
	/// ***rigorous model end*** 	
 		
	/// ***linear model start***
	/*cv::Mat result = cv::Mat::zeros (2, 1, 6);
	cv::Mat A = cv::Mat::zeros(m,2,6);
	cv::Mat B = cv::Mat::zeros(m,1,6);
    
    for (size_t i=0; i<m ; i++)
		{
			A.at<double>(i,0) = bar_slave[i].y;
			A.at<double>(i,1) = 1.0;
			
			B.at<double>(i,0) = bar_master[i].y;
		}	
	
	cv::solve(A, B, result, cv::DECOMP_SVD);
	cv::Mat trX;
	cv::transpose(result, trX);

	cout << "Result matrix "<< endl;
	cout << trX << endl << endl;

	cout << "Difference " << endl;	
	cout << A*result-B << endl;
	
	cv::Mat r = cv::Mat::zeros (2, 3, 6);
	r.at<double>(0,0) = 1.0;
	r.at<double>(0,1) = 0.0;
	r.at<double>(0,2) = 0.0;
	r.at<double>(1,0) = 0.0;
	r.at<double>(1,1) = result.at<double>(0,0);
	r.at<double>(1,2) = result.at<double>(1,0) - master_y + slave_y;*/
 	     
	/// ***linear model end***	
		
	return r;
}

void ossimOpenCvTPgenerator::run()
{
	cv::namedWindow( "master_img", CV_WINDOW_NORMAL );
	cv::imshow("master_img", master_mat);
   
	cv::namedWindow( "slave_img", CV_WINDOW_NORMAL );
	cv::imshow("slave_img", slave_mat);
   
	cv::waitKey(0);
   
	TPgen();  
	TPdraw();
}

void ossimOpenCvTPgenerator::TPdraw()
{
	cv::Mat filt_master, filt_slave;
	cv::Ptr<cv::CLAHE> filtro = cv::createCLAHE(3.0);
	filtro->apply(master_mat, filt_master); 
	filtro->apply(slave_mat, filt_slave);
	
	// Drawing the results
	cv::Mat img_matches;
	cv::drawMatches(filt_master, keypoints1, filt_slave, keypoints2,
               good_matches, img_matches, cv::Scalar::all(-1), cv::Scalar::all(-1),
               vector<char>(), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS );
	
	cv::resize(img_matches, img_matches, cv::Size(), 1.0/8.0, 1.0/8.0, cv::INTER_AREA);

	cv::namedWindow("TP matched", CV_WINDOW_NORMAL );
	cv::imshow("TP matched", img_matches );	
   
	cv::waitKey(0);
}

void ossimOpenCvTPgenerator::TPgen()
{
   	// Computing detector
	cv::OrbFeatureDetector detector(10000);
	detector.detect(master_mat, keypoints1);
	detector.detect(slave_mat, keypoints2);
	
	// Computing descriptors
	cv::BriefDescriptorExtractor extractor;
	cv::Mat descriptors1, descriptors2;
	extractor.compute(master_mat, keypoints1, descriptors1);
	extractor.compute(slave_mat, keypoints2, descriptors2);

	// Matching descriptors
	cv::BFMatcher matcher(cv::NORM_L2);
	vector<cv::DMatch> matches;
	matcher.match(descriptors1, descriptors2, matches);	

	// Calculation of max and min distances between keypoints 
	double max_dist = matches[0].distance; double min_dist = matches[0].distance;

	for( int i = 1; i < descriptors1.rows; i++ )
	{ 
		double dist = matches[i].distance;
		if( dist < min_dist ) min_dist = dist;
		if( dist > max_dist ) max_dist = dist;
	}
	
	//cout << "Min dist between keypoints = " << min_dist << endl;
	//cout << "Max dist between keypoints = " << max_dist << endl;
		
	// Selection of the better 1% descriptors 
	int N_TOT = descriptors1.rows;
	int N_GOOD = 0, N_ITER = 0;

	double good_dist = (max_dist+min_dist)/2.0;
	double per = 100;
	 
	while (fabs(per-0.01) > 0.001 && N_ITER <= 200)
	{		
		for( int i = 0; i < descriptors1.rows; i++ )
		{
			if(matches[i].distance <= good_dist) N_GOOD++;
		}	
		
		per = (double)N_GOOD/(double)N_TOT;
		
		if(per >= 0.01)
		{
			max_dist = good_dist;
		}
		else
		{
		    min_dist = good_dist;
		}
		
		good_dist = (max_dist+min_dist)/2.0;
		
		//cout<< per << " " << min_dist << " " << max_dist << " "<< good_dist <<endl;
		
		N_ITER++;
		N_GOOD = 0;
	} 
	
	// Error computation
	//boost::accumulators::accumulator_set<double, boost::accumulators::stats<boost::accumulators::tag::mean, boost::accumulators::tag::median, boost::accumulators::tag::variance> > acc_x;
	//boost::accumulators::accumulator_set<double, boost::accumulators::stats<boost::accumulators::tag::mean, boost::accumulators::tag::median, boost::accumulators::tag::variance> > acc_y;
	
	for( int i = 0; i < descriptors1.rows; i++ )
	{
		if(matches[i].distance <= good_dist)
		{ 
			// Parallax computation
			double px = keypoints1[i].pt.x - keypoints2[matches[i].trainIdx].pt.x;
			double py = keypoints1[i].pt.y - keypoints2[matches[i].trainIdx].pt.y;	
			
			if(fabs(py) <  500)
			{
				good_matches.push_back(matches[i]);

				//acc_x(px);
				//acc_y(py);
		        
				//cout << i << " " << px << " " << " " << py << " "<<endl;	
			}
		}
	}
	
	//cout << "% points found = " << (double)good_matches.size()/(double)matches.size() << endl;
	cout << endl << "Points found before the 3 sigma test = " << (double)good_matches.size() <<endl << endl; 
	
	// 3 sigma test
	cout << "3 SIGMA TEST " << endl;			 
	int control = 0;
	int num_iter = 0;
	
	do
	{
		num_iter ++;
		cout << "Iteration n = " << num_iter << endl;
		control = 0;
 
 
		cv::Mat parallax = cv::Mat::zeros(good_matches.size(), 1, CV_64F);
		for(size_t i = 0; i < good_matches.size(); i++)
		{
			parallax.at<double>(i,0) = keypoints1[good_matches[i].queryIdx].pt.y - keypoints2[good_matches[i].trainIdx].pt.y; 	
		}		
		cv::Scalar mean_parallax, stDev_parallax;
		cv::meanStdDev(parallax, mean_parallax, stDev_parallax);
		
		double dev_y = stDev_parallax.val[0]; 	
		double mean_diff_y = mean_parallax.val[0]; 
		
		cout << "dev_y = " << dev_y << endl
	         << "mean_diff_y = " << mean_diff_y << endl;
    		
    		
		vector<cv::DMatch > good_matches_corr;
		
		// Get the keypoints from the good_matches
		for (size_t i = 0; i < good_matches.size(); i++)
		{	
        	double py = keypoints1[good_matches[i].queryIdx].pt.y - keypoints2[good_matches[i].trainIdx].pt.y;
			
			if (py< 3*dev_y+mean_diff_y && py > -3*dev_y+mean_diff_y)        
			{
				good_matches_corr.push_back(good_matches[i]);
			}
			else //find outlier 
			{
				control = 10;
			}
		}	
		good_matches = good_matches_corr;
	}
	while(control !=0);
	cout << endl << "Good points found after the 3 sigma test = " << (double)good_matches.size() <<endl << endl; 
}

cv::Mat ossimOpenCvTPgenerator::warp(cv::Mat slave_16bit)
{
	std::vector<cv::Point2f> aff_match1, aff_match2;
	// Get the keypoints from the good_matches
	for (unsigned int i = 0; i < good_matches.size(); ++i)
	{	
		cv::Point2f pt1 = keypoints1[good_matches[i].queryIdx].pt;
		cv::Point2f pt2 = keypoints2[good_matches[i].trainIdx].pt;
		aff_match1.push_back(pt1);
		aff_match2.push_back(pt2);
		//printf("%3d pt1: (%.2f, %.2f) pt2: (%.2f, %.2f)\n", i, pt1.x, pt1.y, pt2.x, pt2.y);
	}
     
	// Estimate quasi-epipolar transformation model 
	cv::Mat rot_matrix = estRT(aff_match2, aff_match1);
	
	//cout << "Rotation matrix" << endl;
	//cout << rot_matrix << endl;
 
    // Set the destination image the same type and size as source
	cv::Mat warp_dst = cv::Mat::zeros(master_mat.rows, master_mat.cols, master_mat.type());
	cv::Mat warp_dst_16bit = cv::Mat::zeros(slave_16bit.rows, slave_16bit.cols, slave_16bit.type());
	
	cv::warpAffine(slave_mat, warp_dst, rot_matrix, warp_dst.size());
	cv::warpAffine(slave_16bit, warp_dst_16bit, rot_matrix, warp_dst.size());
    
	cv::namedWindow("Master image", CV_WINDOW_NORMAL);
	cv::imshow("Master image", master_mat );
	cv::imwrite("Master_8bit.tif",  master_mat);

	cv::namedWindow("Warped image", CV_WINDOW_NORMAL);
	cv::imshow("Warped image", warp_dst );
	cv::imwrite("Slave_8bit.tif",  warp_dst);
	
	cv::waitKey(0);
    
	return warp_dst;
}	

