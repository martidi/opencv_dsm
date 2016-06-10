//----------------------------------------------------------------------------
//
// License:  See top level LICENSE.txt file.
//
// File: ossimOpenCvTPgenerator.cpp
//
// Author:  Martina Di Rita
//
// Description: Class providing a TPs generator
//
//----------------------------------------------------------------------------

#include <ossim/base/ossimIrect.h>
#include <ossim/imaging/ossimImageSource.h>

#include "ossimOpenCvTPgenerator.h"

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

void ossimOpenCvTPgenerator::run()
{
    /*cv::Ptr<cv::CLAHE> filtro = cv::createCLAHE(8.0); //threshold for contrast limiting
    filtro->apply(master_mat, master_mat);
    filtro->apply(slave_mat, slave_mat);*/

    cv::namedWindow( "master_img", CV_WINDOW_NORMAL );
    cv::imshow("master_img", master_mat);
   
    cv::namedWindow( "slave_img", CV_WINDOW_NORMAL );
    cv::imshow("slave_img", slave_mat);
   
	TPgen();  
	TPdraw();
}

void ossimOpenCvTPgenerator::TPgen()
{
    // Computing detector
    /*
    cv::OrbFeatureDetector detector(30000, 2.0f,8, 151, 0, 2, cv::ORB::HARRIS_SCORE, 151 ); // edgeThreshold = 150, patchSize = 150);
	detector.detect(master_mat, keypoints1);
	detector.detect(slave_mat, keypoints2);
    */
    cv::Ptr<cv::FeatureDetector> m_detector;
    cv::Ptr<cv::OrbFeatureDetector> detector = cv::FeatureDetector::create("ORB");
    m_detector = new cv::GridAdaptedFeatureDetector (detector, 500, 5, 5 );
    m_detector->detect(master_mat, keypoints1);
    m_detector->detect(slave_mat, keypoints2);

    cerr << "Features found = " << keypoints1.size() << " \tmaster " << keypoints2.size() << " \tslave " << endl;

	// Computing descriptors
	cv::BriefDescriptorExtractor extractor;
	cv::Mat descriptors1, descriptors2;
	extractor.compute(master_mat, keypoints1, descriptors1);
	extractor.compute(slave_mat, keypoints2, descriptors2);

	// Matching descriptors
	cv::BFMatcher matcher(cv::NORM_L2);
    vector<cv::DMatch> matches;
	matcher.match(descriptors1, descriptors2, matches);	

    cerr << matches.size() << endl;

/*    //HARRIS corner detector

    int thresh = 200;

    cv::Mat dst, dst_slave, dst_norm, dst_norm_scaled;
    //dst = cv::Mat::zeros( master_mat.size(), CV_32FC1 );
    //dst_slave = cv::Mat::zeros( master_mat.size(), CV_32FC1 );

    // Detector parameters
    int blockSize = 3;  //block dimension
    int apertureSize = 9;
    double k = 0.04;

    // Detecting corners
    cornerHarris( master_mat, dst, blockSize, apertureSize, k, cv::BORDER_DEFAULT );
    cornerHarris( slave_mat, dst_slave, blockSize, apertureSize, k, cv::BORDER_DEFAULT );

    // Normalizing
    normalize( dst, dst_norm, 0, 255, cv::NORM_MINMAX, CV_32FC1, cv::Mat() );
    convertScaleAbs( dst_norm, dst_norm_scaled );

    // Drawing a circle around corners
    for( int j = 0; j < dst_norm.rows ; j++ )
       { for( int i = 0; i < dst_norm.cols; i++ )
            {
              if( (int) dst_norm.at<float>(j,i) > thresh )
                {
                 circle( dst_norm_scaled, cv::Point( i, j ), 5,  cv::Scalar(0), 2, 8, 0 );
                }
            }
       }

    // Showing the result
    cv::namedWindow( "Prova_harris_master", CV_WINDOW_NORMAL );
    cv::imshow( "Prova_harris_master", dst);
    // Showing the result
    cv::namedWindow( "Prova_harris_slave", CV_WINDOW_NORMAL );
    cv::imshow( "Prova_harris_slave", dst_slave);

    cv::resize(dst_slave, dst_slave, dst.size());
    cv::Mat product = dst.mul(dst_slave);

    cv::namedWindow( "Prova_harris_product", CV_WINDOW_NORMAL );
    cv::imshow( "Prova_harris_product", product);

*/


// *****************TEMPLATE MATCHING***************************************************
/*

    // Template window on master image
    int n = master_mat.cols;
    int m = master_mat.rows;

    cout << n << " colonne\t" << m << " righe" << endl;

    int n_rows = 10;
    int n_cols = 10;
    int centerX_master = n/(n_cols+1);  // ottengo quanto devono distare i vari centri della griglia sulla master
    int centerY_master = m/(n_rows+1);
    int centerX_slave = n/(n_rows+1);  // ottengo quanto devono distare i vari centri della griglia sulla slave
    int centerY_slave = m/(n_cols+1);

    int NewcenterX_master = centerX_master;
    int NewcenterY_master = centerY_master;
    int NewcenterX_slave = centerX_slave;
    int NewcenterY_slave = centerY_slave;

    cout << centerX_master << " x del punto\t" <<  centerY_master << " y del punto" << endl;

    cv::Mat templ, research, research_display, result;

    int height_templ = 25;
    int width_templ = 25;
    int height_research = 50;
    int width_research = 50;

    // Localizing the best match with minMaxLoc
    double minVal; double maxVal; cv::Point minLoc; cv::Point maxLoc;

    //Create and write the log file
    ofstream templ_match("Template_matching_cycles.txt");


    cv::Mat corr_values(n_rows*n_cols,5, CV_64F);
    cout <<"Inizio ciclo FOR" << endl;
    for (int i = 0; i < n_rows; i++) // rows
    {
        for (int j = 0; j < n_cols; j++) // cols
        {
            cv::Rect rect_master = cv::Rect(NewcenterX_master - (width_templ/2) , NewcenterY_master - (height_templ/2), width_templ , height_templ); // (left corner.x, left corner.y, width, height)
            templ = master_mat(rect_master); // template window on master image grande rect

            cv::Rect rect_slave = cv::Rect(NewcenterX_slave - (width_research/2) , NewcenterY_slave - (height_research/2), width_research , height_research); // (left corner.x, left corner.y, width, height)
            research = slave_mat(rect_slave); // template window on slave image grande rect

            templ_match << i << " i cycle" << endl
                        << j << " j cycle" << endl
                        << "(" << NewcenterX_master << "," << NewcenterY_master << ")" << " X, Y template window center coordinates " << endl
                        << "(" << NewcenterX_master - (width_templ/2) << "," << NewcenterY_master - (height_templ/2) << ")" << " X, Y template window left corner coordinates " << endl
                        << "(" << NewcenterX_slave << "," << NewcenterY_slave << ")" << " X, Y research window center coordinates " << endl
                        << "(" << NewcenterX_slave - (width_research/2) << "," << NewcenterY_slave - (height_research/2) << ")" << " X, Y research window left corner coordinates " << endl << endl;

            // Source image to display the rectangle
            research.copyTo( research_display ); //Copy of the slave patch to draw a rectangle

            // Conversion from 32 to 8 bit image
            double minVal_research, maxVal_research, minVal_templ, maxVal_templ;

            minMaxLoc( master_mat, &minVal_research, &maxVal_research );
            minMaxLoc( slave_mat, &minVal_templ, &maxVal_templ );

            cv::Mat research_display_8U;
            cv::Mat templ_8U;

            research.convertTo(research_display_8U, CV_8UC1, 255.0/(maxVal_research - minVal_research), -minVal_research*255.0/(maxVal_research - minVal_research));
            templ.convertTo   (templ_8U, CV_8UC1, 255.0/(maxVal_templ - minVal_templ), -minVal_templ*255.0/(maxVal_templ - minVal_templ));

            // Do the Matching and Normalize
            cv::matchTemplate( research_display_8U, templ_8U, result,  CV_TM_CCORR_NORMED );
            //cv::normalize( result, result, 0, 1, cv::NORM_MINMAX, -1, cv::Mat() );

            minMaxLoc( result, &minVal, &maxVal, &minLoc, &maxLoc, cv::Mat() );

            cv::Point corner_window = cv::Point( maxLoc.x + (width_templ/2) , maxLoc.y + (height_templ/2)); // coordinate del punto di massima correlazione rispetto allo spigolo della research window
            cv::Point corner_slave = cv::Point(corner_window.x + NewcenterX_slave - (width_research/2), corner_window.y + NewcenterY_slave - (height_research/2)); // coordinate del punto di massima correlazione rispetto allo spigolo della slave
            cv::Point corner_master = cv::Point(NewcenterX_master, NewcenterY_master ); // coordinate del punto di massima correlazione rispetto allo spigolo della master

            corr_values.at<double>(j+i*n_rows,0) = corner_master.x;
            corr_values.at<double>(j+i*n_rows,1) = corner_master.y;
            corr_values.at<double>(j+i*n_rows,2) = corner_slave.x;
            corr_values.at<double>(j+i*n_rows,3) = corner_slave.y;
            corr_values.at<double>(j+i*n_rows,4) = maxVal;

            cv::namedWindow("Template image", CV_WINDOW_NORMAL);
            cv::imshow("Template image", templ );
            cv::namedWindow("Research image", CV_WINDOW_NORMAL);
            cv::imshow("Research image", research );

            // Show me what you got
            rectangle( research_display, maxLoc, cv::Point( maxLoc.x + templ.cols , maxLoc.y + templ.rows ), cv::Scalar::all(0), 2, 8, 0 );
            rectangle( result, cv::Point( maxLoc.x - 0.5*templ.cols , maxLoc.y - 0.5*templ.rows ), cv::Point( maxLoc.x + 0.5*templ.cols , maxLoc.y + 0.5*templ.rows ), cv::Scalar::all(0), 2, 8, 0 );

            cv::namedWindow("Source image", CV_WINDOW_NORMAL);
            cv::imshow( "Source image", research_display );

            cv::namedWindow("Result window", CV_WINDOW_NORMAL);
            cv::imshow( "Result window", result );

            NewcenterX_master += centerX_master;
            NewcenterX_slave += centerX_slave;
        }


        NewcenterY_master += centerY_master;
        NewcenterX_master = centerX_master;

        NewcenterY_slave += centerY_slave;
        NewcenterX_slave = centerX_slave;
    }

    templ_match << "Stampa matrice intera " <<endl;
    templ_match << "Matrix\n " << corr_values << endl << endl;
    templ_match.close();

    cv::waitKey(0);
*/


	// Calculation of max and min distances between keypoints 
    double max_dist = matches[0].distance; double min_dist = matches[0].distance;
    cout << "max dist" << max_dist << endl;
    cout << "min dist" << min_dist << endl;

	for( int i = 1; i < descriptors1.rows; i++ )
	{ 
        double dist = matches[i].distance;
        //cout << "dist" << dist << endl;
		if( dist < min_dist ) min_dist = dist;
		if( dist > max_dist ) max_dist = dist;
	}
	
	//cout << "Min dist between keypoints = " << min_dist << endl;
	//cout << "Max dist between keypoints = " << max_dist << endl;
		
	// Selection of the better 1% descriptors 
	int N_TOT = descriptors1.rows;
	int N_GOOD = 0, N_ITER = 0;

    double good_dist = (max_dist + min_dist) /2.0;
	double per = 100;
	 
    while (fabs(per-0.98) > 0.001 && N_ITER <= 200)
	{		
		for( int i = 0; i < descriptors1.rows; i++ )
		{
            if(matches[i].distance <= good_dist) N_GOOD++;
		}	
		
		per = (double)N_GOOD/(double)N_TOT;
		
        if(per >= 0.98) //if(per >= 0.01)
		{
			max_dist = good_dist;
		}
		else
		{
		    min_dist = good_dist;
		}
		
        good_dist = (max_dist + min_dist)/2.0;
		
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
			
            if(fabs(py) <  10)
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


void ossimOpenCvTPgenerator::TPdraw()
{
    /*cv::Mat filt_master, filt_slave;
    cv::Ptr<cv::CLAHE> filtro = cv::createCLAHE(8.0); //threshold for contrast limiting
    filtro->apply(master_mat, filt_master);
    filtro->apply(slave_mat, filt_slave);*/

    // Drawing the results
    cv::Mat img_matches;
    cv::drawMatches(master_mat, keypoints1, slave_mat, keypoints2,
               good_matches, img_matches, cv::Scalar::all(-1), cv::Scalar::all(-1),
               vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);

    cv::resize(img_matches, img_matches, cv::Size(), 1.0/1.0, 1.0/1.0, cv::INTER_AREA);

    cv::namedWindow("TP matched", CV_WINDOW_NORMAL );
    cv::imshow("TP matched", img_matches );

    cv::waitKey(0);
}


cv::Mat ossimOpenCvTPgenerator::warp(cv::Mat slave_16bit)
{
    vector<cv::Point2f> aff_match1, aff_match2;
	// Get the keypoints from the good_matches
	for (unsigned int i = 0; i < good_matches.size(); ++i)
	{	
		cv::Point2f pt1 = keypoints1[good_matches[i].queryIdx].pt;
		cv::Point2f pt2 = keypoints2[good_matches[i].trainIdx].pt;
		aff_match1.push_back(pt1);
		aff_match2.push_back(pt2);
		//printf("%3d pt1: (%.2f, %.2f) pt2: (%.2f, %.2f)\n", i, pt1.x, pt1.y, pt2.x, pt2.y);
	}

//************************************* Inserisci punti collimati a mano su master e slave native
/*    vector<cv::Point2f> pippo1, pippo2;

    pippo2.push_back(cv::Point2f(186.0, 109.0)); //slave
    pippo2.push_back(cv::Point2f(217.0, 231.0));
    pippo2.push_back(cv::Point2f(321.0, 274.0));
    pippo2.push_back(cv::Point2f(541.0, 180.0));
    pippo2.push_back(cv::Point2f(551.0, 72.0));
    pippo2.push_back(cv::Point2f(721.0, 327.0));
    pippo2.push_back(cv::Point2f(426.0, 511.0));
    pippo2.push_back(cv::Point2f(226.0, 829.0));
    pippo2.push_back(cv::Point2f(797.0, 297.0));
    pippo2.push_back(cv::Point2f(62.0, 793.0));

    pippo1.push_back(cv::Point2f(165.0, 105.0)); //master
    pippo1.push_back(cv::Point2f(195.0, 226.0));
    pippo1.push_back(cv::Point2f(301.0, 269.0));
    pippo1.push_back(cv::Point2f(524.0, 173.0));
    pippo1.push_back(cv::Point2f(543.0, 70.0));
    pippo1.push_back(cv::Point2f(709.0, 324.0));
    pippo1.push_back(cv::Point2f(425.0, 510.0));
    pippo1.push_back(cv::Point2f(223.0, 829.0));
    pippo1.push_back(cv::Point2f(797.0, 296.0));
    pippo1.push_back(cv::Point2f(39.0, 789.0));
*/
//***************************************  prova per controllare che il modello sia buono

    //cv::Mat rot_matrix = estRT(pippo2, pippo1);


	// Estimate quasi-epipolar transformation model 
    cv::Mat rot_matrix = estRT(aff_match2, aff_match1);
	
	//cout << "Rotation matrix" << endl;
	//cout << rot_matrix << endl;
 
    // Set the destination image the same type and size as source
	cv::Mat warp_dst = cv::Mat::zeros(master_mat.rows, master_mat.cols, master_mat.type());
	cv::Mat warp_dst_16bit = cv::Mat::zeros(slave_16bit.rows, slave_16bit.cols, slave_16bit.type());
	
	cv::warpAffine(slave_mat, warp_dst, rot_matrix, warp_dst.size());
	cv::warpAffine(slave_16bit, warp_dst_16bit, rot_matrix, warp_dst.size());
    
    //cv::namedWindow("Master image", CV_WINDOW_NORMAL);
    //cv::imshow("Master image", master_mat );
	cv::imwrite("Master_8bit.tif",  master_mat);

    //cv::namedWindow("Warped image", CV_WINDOW_NORMAL);
    //cv::imshow("Warped image", warp_dst );
	cv::imwrite("Slave_8bit.tif",  warp_dst);
	
	//cv::waitKey(0);
    
	return warp_dst;
}	

cv::Mat ossimOpenCvTPgenerator::estRT(std::vector<cv::Point2f> master, std::vector<cv::Point2f> slave)
{
    size_t m = master.size();

    if ( master.size() != slave.size() )
    {
        throw 0;
    }

    // Computing barycentric coordinates

    cv::Scalar mean_master_x , mean_master_y, mean_slave_x, mean_slave_y, mean_shift_x, mean_shift_y;
    cv::Scalar stDev_master_x, stDev_master_y, stDev_slave_x, stDev_slave_y, stDev_shift_x, stDev_shift_y;

    cv::Mat components_matrix = cv::Mat::zeros(m,6, CV_64F);
    for(size_t i = 0; i < m; i++)
    {
        components_matrix.at<double>(i,0) = master[i].x;
        components_matrix.at<double>(i,1) = master[i].y;
        components_matrix.at<double>(i,2) = slave[i].x;
        components_matrix.at<double>(i,3) = slave[i].y;
        components_matrix.at<double>(i,4) = master[i].x - slave[i].x;
        components_matrix.at<double>(i,5) = master[i].y - slave[i].y;
    }
    cv::meanStdDev(components_matrix.col(0), mean_master_x, stDev_master_x);
    cv::meanStdDev(components_matrix.col(1), mean_master_y, stDev_master_y);
    cv::meanStdDev(components_matrix.col(2), mean_slave_x, stDev_slave_x);
    cv::meanStdDev(components_matrix.col(3), mean_slave_y, stDev_slave_y);
    cv::meanStdDev(components_matrix.col(4), mean_shift_x, stDev_shift_x);
    cv::meanStdDev(components_matrix.col(5), mean_shift_y, stDev_shift_y);

    master_x = mean_master_x.val[0];
    master_y = mean_master_y.val[0];
    slave_x = mean_slave_x.val[0];
    slave_y	= mean_slave_y.val[0];


    double StDevShiftX = stDev_shift_x.val[0];
    double StDevShiftY = stDev_shift_y.val[0];


    cout << "Mean_x_master = " << master_x << endl
         << "Mean_y_master = " << master_y << endl
         << "Mean_x_slave = "  << slave_x  << endl
         << "Mean_y_slave = "  << slave_y  << endl
         << "Shift in x = "	<< master_x - slave_x << "\tpixel" << endl
         << "Shift in y = "	<< master_y - slave_y << "\tpixel" <<endl << endl
         << "St.dev. shift x = " <<	StDevShiftX << endl
         << "St.dev. shift y = " <<	StDevShiftY << endl << endl;

    std::vector<cv::Point2f> bar_master, bar_slave;

    for (size_t i = 0; i < m; i++)
    {
        cv::Point2f pt1;
        cv::Point2f pt2;

        pt1.x = master[i].x - master_x;
        pt1.y = master[i].y - master_y;

        //pt2.x = slave[i].x - master_x;
        //pt2.y = slave[i].y - master_y;

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

    for (int j= 0; j <3; j++)
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
        //cout << "Matrice risultati\n" << x_approx << endl;
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
    //cv::Point2f pt(master_x , master_y);
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
    //cout << "Matrice r" << r << endl;
    return r;
}
