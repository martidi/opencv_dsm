//----------------------------------------------------------------------------
//
// License:  See top level LICENSE.txt file.
//
// File: ossim-opencv.cpp
//
// Author:  Martina Di Rita
//
// Description: This plugIn is able to extract a geocoded Digital Surface Model
//				using a stereopair.
//
//----------------------------------------------------------------------------

#include <ossim/base/ossimArgumentParser.h>
#include <ossim/base/ossimApplicationUsage.h>
#include <ossim/base/ossimConstants.h> 
#include <ossim/base/ossimException.h>
#include <ossim/base/ossimNotify.h>
#include <ossim/base/ossimRefPtr.h>
#include <ossim/base/ossimTimer.h>
#include <ossim/base/ossimTrace.h>
#include <ossim/base/ossimGpt.h>
#include <ossim/base/ossimDpt.h>
#include <ossim/base/ossimKeywordlist.h>
#include <ossim/base/ossimKeywordNames.h>
#include <ossim/base/ossimStdOutProgress.h>

#include <ossim/elevation/ossimElevManager.h>

#include "ossim/imaging/ossimImageHandlerRegistry.h"
#include "ossim/imaging/ossimImageHandler.h"
#include "ossim/imaging/ossimImageGeometry.h"
#include "ossim/imaging/ossimImageFileWriter.h"
#include "ossim/imaging/ossimImageWriterFactoryRegistry.h"

#include <ossim/init/ossimInit.h>

#include <ossim/util/ossimChipperUtil.h>

#include "ossimOpenCvTPgenerator.h"
#include "openCVtestclass.h"
#include "ossimOpenCvDisparityMapGenerator.h"

#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <opencv/cv.h>



#include <iostream>
#include <sstream>
#include <cstdlib> /* for exit */
#include <iomanip>

#define C_TEXT( text ) ((char*)std::string( text ).c_str())

using namespace std;

static const std::string CUT_MAX_LAT_KW          = "cut_max_lat";
static const std::string CUT_MAX_LON_KW          = "cut_max_lon";
static const std::string CUT_MIN_LAT_KW          = "cut_min_lat";
static const std::string CUT_MIN_LON_KW          = "cut_min_lon";
static const std::string METERS_KW               = "meters";
static const std::string OP_KW                   = "operation";
static const std::string RESAMPLER_FILTER_KW     = "resampler_filter";

bool ortho (ossimKeywordlist kwl)
{
	// Make the generator
	ossimRefPtr<ossimChipperUtil> chipper = new ossimChipperUtil;
	chipper->initialize(kwl);

	try
	{      
		// ossimChipperUtil::execute can throw an exception
		chipper->execute();
		ossimNotify(ossimNotifyLevel_NOTICE)
		<< "elapsed time in seconds: "
		<< std::setiosflags(ios::fixed)
		<< std::setprecision(3)
		<< ossimTimer::instance()->time_s() << endl << endl;
	}
	catch (const ossimException& e)
	{
		ossimNotify(ossimNotifyLevel_WARN) << e.what() << endl;
		exit(1);
	}
	return true;
}

static ossimTrace traceDebug = ossimTrace("ossim-chipper:debug");

int main(int argc,  char* argv[])
{
	// Initialize ossim stuff, factories, plugin, etc.
	ossimTimer::instance()->setStartTick();
   	ossimArgumentParser ap(&argc, argv);
	ossimInit::instance()->initialize(ap);
	try
	{ 
		// PARSER *******************************
		cout << "Arg number " << ap.argc() << endl;
				
		ossimKeywordlist forward_key;
		ossimKeywordlist nadir_key;
		ossimKeywordlist backward_key;		

        forward_key.addPair("image1.file", ap[1]);
        nadir_key.addPair("image1.file", ap[2]);
        backward_key.addPair("image1.file", ap[3]);

		//Default keyword for orthorectification
		forward_key.addPair(OP_KW, "ortho");
		nadir_key.addPair( OP_KW, "ortho");
		backward_key.addPair( OP_KW, "ortho");		
		
		forward_key.addPair(RESAMPLER_FILTER_KW, "box");
		nadir_key.addPair( RESAMPLER_FILTER_KW, "box");
        backward_key.addPair( RESAMPLER_FILTER_KW, "box");

        if(ap.argc() < 6) //ap.argv[0] is the application name
        {
            ap.writeErrorMessages(ossimNotify(ossimNotifyLevel_NOTICE));
            std::string errMsg = "Few arguments...";
            cout << endl << "Usage: ossim-dsm-app <input_fwd_image> <input_nad_image> <input_bwd_image> <output_results_directory> <output_dsm_name> [options] <n° steps for pyramidal>" << endl;
            cout << "Options:" << endl;
            cout << "--cut-bbox-ll <min_lat> <min_lon> <max_lat> <max_lon> \t Specify a bounding box with the minimum"   << endl;
            cout << "\t\t\t\t\t\t\tlatitude/longitude and max latitude/longitude" << endl;
            cout << "\t\t\t\t\t\t\tin decimal degrees." << endl;
            cout << "--meters <meters> \t\t\t\t\t Specify a size (in meters) for a resampling"   << endl<< endl;
            throw ossimException(errMsg);
        }




		
		// Parsing
		std::string tempString1,tempString2,tempString3,tempString4;
		ossimArgumentParser::ossimParameter stringParam1(tempString1);
		ossimArgumentParser::ossimParameter stringParam2(tempString2);
		ossimArgumentParser::ossimParameter stringParam3(tempString3);
		ossimArgumentParser::ossimParameter stringParam4(tempString4);
    
		if(ap.read("--meters", stringParam1) )
		{
			forward_key.addPair(METERS_KW, tempString1 );
			nadir_key.addPair( METERS_KW, tempString1 );
			backward_key.addPair( METERS_KW, tempString1 );			
		}
        double finalRes = atof(tempString1.c_str());
		cout << "Orthoimages resolution = " << tempString1 <<" meters"<< endl << endl;


        int nsteps;
        ossimArgumentParser::ossimParameter iteration(nsteps);
        if(ap.read("--nsteps", iteration))
        {
          //else nsteps = 1;
        }
        cout << "step number for pyramidal\t " << nsteps << endl;

        double lat_min;
        double lon_min;
        double lat_max;
        double lon_max;

        double MinHeight;
        double MaxHeight;
        
		if( ap.read("--cut-bbox-ll", stringParam1, stringParam2, stringParam3, stringParam4) )
		{
			forward_key.addPair( CUT_MIN_LAT_KW, tempString1 );
			forward_key.addPair( CUT_MIN_LON_KW, tempString2 );
			forward_key.addPair( CUT_MAX_LAT_KW, tempString3 );
			forward_key.addPair( CUT_MAX_LON_KW, tempString4 );
			
			nadir_key.addPair( CUT_MIN_LAT_KW, tempString1 );
			nadir_key.addPair( CUT_MIN_LON_KW, tempString2 );
			nadir_key.addPair( CUT_MAX_LAT_KW, tempString3 );
			nadir_key.addPair( CUT_MAX_LON_KW, tempString4 );
		
			backward_key.addPair( CUT_MIN_LAT_KW, tempString1 );
			backward_key.addPair( CUT_MIN_LON_KW, tempString2 );
			backward_key.addPair( CUT_MAX_LAT_KW, tempString3 );
			backward_key.addPair( CUT_MAX_LON_KW, tempString4 );
			
			lat_min = atof(tempString1.c_str());
			lon_min = atof(tempString2.c_str());
			lat_max = atof(tempString3.c_str());
     		lon_max = atof(tempString4.c_str());
			
     						     								
 			cout << "Tile extent:" << "\tLat_min = "<< lat_min << endl   
								<<"\t\tLon_min = " << lon_min << endl
								<<"\t\tLat_max = " << lat_max << endl
								<<"\t\tLon_max = " << lon_max << endl << endl; 

            //**********MIN and MAX HEIGHT COMPUTATION************************
           std::vector<ossim_float64> HeightAboveMSL;
            for(double lat = lat_min; lat < lat_max; lat += 0.001)
            {
                for(double lon = lon_min; lon < lon_max; lon += 0.001)
                {
                    ossimGpt world_point(lat, lon, 0.00);
                    ossim_float64 hgtAboveMsl = ossimElevManager::instance()->getHeightAboveMSL(world_point);
                    HeightAboveMSL.push_back(hgtAboveMsl);
                }
            }

            MinHeight = *min_element(HeightAboveMSL.begin(), HeightAboveMSL.end());
            MaxHeight = *max_element(HeightAboveMSL.begin(), HeightAboveMSL.end());
            cout << "Min height for this tile is " << std::setprecision(6) << MinHeight << " m" << endl;
            cout << "Max height for this tile is " << std::setprecision(6) << MaxHeight << " m" << endl;
		}


		// End of arg parsing
		ap.reportRemainingOptionsAsUnrecognized();
		if (ap.errors())
		{
			ap.writeErrorMessages(ossimNotify(ossimNotifyLevel_NOTICE));
			std::string errMsg = "Unknown option...";
			throw ossimException(errMsg);
		}
		

		//END PARSER****************************
	        
        cout << endl << "FORWARD DIRECTORY:" << " " << ap[1] << endl;
        cout << "NADIR DIRECTORY:"  << " " << ap[2] << endl;	
        cout << "BACKWARD DIRECTORY:"  << " " << ap[3] << endl << endl;        

        // Elevation manager instance
        ossimElevManager* elev = ossimElevManager::instance();

        cout << "numero di elevation    " << elev->getNumberOfElevationDatabases() << endl;

        ossimImageHandler* raw_forward_handler = ossimImageHandlerRegistry::instance()->open(ossimFilename(ap[1]));
        ossimImageHandler* raw_nadir_handler = ossimImageHandlerRegistry::instance()->open(ossimFilename(ap[2]));
        ossimImageHandler* raw_backward_handler = ossimImageHandlerRegistry::instance()->open(ossimFilename(ap[3]));

        ossimRefPtr<ossimImageGeometry> raw_forward_geom = raw_forward_handler->getImageGeometry();
        ossimRefPtr<ossimImageGeometry> raw_nadir_geom = raw_nadir_handler->getImageGeometry();
        ossimRefPtr<ossimImageGeometry> raw_backward_geom = raw_backward_handler->getImageGeometry();

        // CONVERSION FACTOR (from pixels to meters) COMPUTATION *************

        // Conversion factor computed on tile and not over all the image
        double Dlon = (lon_max - lon_min)/2.0;
        double Dlat = (lat_max - lat_min)/2.0;

        // Getting ready the log file
        char * logfile = ap[3];
        string log(logfile);
        log.erase(log.end()-4, log.end()-0 );
        log = log + "_logfile.txt";

        // Creating and writing the log file
        ofstream myfile;
        myfile.open (log.c_str());

        cv::Mat conv_factor_J_NadFwd = cv::Mat::zeros(3,3, CV_64F);
        cv::Mat conv_factor_I_NadFwd = cv::Mat::zeros(3,3, CV_64F);
        cv::Mat conv_factor_J_NadBwd = cv::Mat::zeros(3,3, CV_64F);
        cv::Mat conv_factor_I_NadBwd = cv::Mat::zeros(3,3, CV_64F);

        for (int i=0 ; i<3 ; i++) //LAT
        {
            for (int j=0 ; j<3 ; j++) //LON
            {
                ossimGpt groundPoint(lat_max-i*Dlat,lon_min+j*Dlon,MinHeight -50);
                ossimGpt groundPointUp(lat_max-i*Dlat,lon_min+j*Dlon,MaxHeight + 50);

                ossimDpt imagePoint(0.,0.);
                ossimDpt imagePointUp(0.,0.);

                raw_nadir_geom->worldToLocal(groundPoint,imagePoint);        //con qst trasf ottengo imagePoint della nadir
                raw_nadir_geom->worldToLocal(groundPointUp,imagePointUp);

                myfile << "NADIR IMAGE" << "\t" << "Ground point" << groundPoint << "\t" << "Image point" << imagePoint << "\t" << "Ground point up" << groundPointUp << "\t" << "Image point up" << imagePointUp << "\t";

                double DeltaI_nad = imagePointUp.x - imagePoint.x;
                double DeltaJ_nad = imagePointUp.y - imagePoint.y;

                raw_forward_geom->worldToLocal(groundPoint,imagePoint);
                raw_forward_geom->worldToLocal(groundPointUp,imagePointUp);

                myfile << "FORWARD IMAGE" << "\t" << "Ground point" << groundPoint << "\t" << "Image point" << imagePoint << "\t" << "Ground point up" << groundPointUp << "\t" << "Image point up" << imagePointUp << "\t" << endl;

                double DeltaI_fwd = imagePointUp.x - imagePoint.x;
                double DeltaJ_fwd = imagePointUp.y - imagePoint.y;

                raw_backward_geom->worldToLocal(groundPoint,imagePoint);
                raw_backward_geom->worldToLocal(groundPointUp,imagePointUp);

                myfile << "BACKWARD IMAGE" << "\t" << "Ground point" << groundPoint << "\t" << "Image point" << imagePoint << "\t" << "Ground point up" << groundPointUp << "\t" << "Image point up" << imagePointUp << "\t" << endl;

                double DeltaI_bwd = imagePointUp.x - imagePoint.x;
                double DeltaJ_bwd = imagePointUp.y - imagePoint.y;

                conv_factor_J_NadFwd.at<double>(i,j) = DeltaJ_fwd - DeltaJ_nad; // conv_factor for ACROSS-track imgs
                conv_factor_I_NadFwd.at<double>(i,j) = DeltaI_fwd - DeltaI_nad; // conv_factor for ALONG-track imgs
                conv_factor_J_NadBwd.at<double>(i,j) = DeltaJ_fwd - DeltaJ_bwd;
                conv_factor_I_NadBwd.at<double>(i,j) = DeltaI_fwd - DeltaI_bwd;
            }
        }

        cv::Scalar mean_conv_factor_J_NadFwd, stDev_conv_factor_J_NadFwd;
        cv::meanStdDev(conv_factor_J_NadFwd, mean_conv_factor_J_NadFwd, stDev_conv_factor_J_NadFwd);
        cv::Scalar mean_conv_factor_I_NadFwd, stDev_conv_factor_I_NadFwd;
        cv::meanStdDev(conv_factor_I_NadFwd, mean_conv_factor_I_NadFwd, stDev_conv_factor_I_NadFwd);

        double stDev_conversionF_J_NadFwd = stDev_conv_factor_J_NadFwd.val[0];
        double mean_conversionF_J_NadFwd = mean_conv_factor_J_NadFwd.val[0]/(MaxHeight -MinHeight + 100);
        double stDev_conversionF_I_NadFwd = stDev_conv_factor_I_NadFwd.val[0];
        double mean_conversionF_I_NadFwd = mean_conv_factor_I_NadFwd.val[0]/(MaxHeight -MinHeight + 100);
        double mean_conversionF_NadFwd = (sqrt((mean_conv_factor_J_NadFwd.val[0]*mean_conv_factor_J_NadFwd.val[0]) + (mean_conv_factor_I_NadFwd.val[0]*mean_conv_factor_I_NadFwd.val[0]))/(MaxHeight -MinHeight + 100));

        cv::Scalar mean_conv_factor_J_NadBwd, stDev_conv_factor_J_NadBwd;
        cv::meanStdDev(conv_factor_J_NadBwd, mean_conv_factor_J_NadBwd, stDev_conv_factor_J_NadBwd);
        cv::Scalar mean_conv_factor_I_NadBwd, stDev_conv_factor_I_NadBwd;
        cv::meanStdDev(conv_factor_I_NadBwd, mean_conv_factor_I_NadBwd, stDev_conv_factor_I_NadBwd);

        double stDev_conversionF_J_NadBwd = stDev_conv_factor_J_NadBwd.val[0];
        double mean_conversionF_J_NadBwd = mean_conv_factor_J_NadBwd.val[0]/(MaxHeight -MinHeight + 100);
        double stDev_conversionF_I_NadBwd = stDev_conv_factor_I_NadBwd.val[0];
        double mean_conversionF_I_NadBwd = mean_conv_factor_I_NadBwd.val[0]/(MaxHeight -MinHeight + 100);
        double mean_conversionF_NadBwd = (sqrt((mean_conv_factor_J_NadBwd.val[0]*mean_conv_factor_J_NadBwd.val[0]) + (mean_conv_factor_I_NadBwd.val[0]*mean_conv_factor_I_NadBwd.val[0]))/(MaxHeight -MinHeight + 100));


        cout << "J Conversion Factor from pixels to meters for NAD-FWD\t" << mean_conversionF_J_NadFwd << endl;
        cout << "Standard deviation J Conversion Factor for NAD-FWD\t" << stDev_conversionF_J_NadFwd << endl << endl;
        cout << "I Conversion Factor from pixels to meters for NAD-FWD\t" << mean_conversionF_I_NadFwd << endl;
        cout << "Standard deviation I Conversion Factor for NAD-FWD\t" << stDev_conversionF_I_NadFwd << endl << endl;

        cout << "Total Conversion Factor from pixels to meters for NAD-FWD\t" << mean_conversionF_NadFwd << endl <<endl;

        cout << "J Conversion Factor from pixels to meters NAD-BWD\t" << mean_conversionF_J_NadBwd << endl;
        cout << "Standard deviation J Conversion Factor NAD-BWD\t" << stDev_conversionF_J_NadBwd << endl << endl;
        cout << "I Conversion Factor from pixels to meters NAD-BWD\t" << mean_conversionF_I_NadBwd << endl;
        cout << "Standard deviation I Conversion Factor NAD-BWD\t" << stDev_conversionF_I_NadBwd << endl << endl;

        cout << "Total Conversion Factor from pixels to meters NAD-BWD\t" << mean_conversionF_NadBwd << endl;

        // END CONVERSION FACTOR COMPUTATION ****************************************

        myfile << endl << "Nadir orthorectification parameters" <<endl;
        myfile << nadir_key << endl;
        myfile << "Forward orthorectification parameters" <<endl;
        myfile << forward_key << endl;
        myfile << "Backward orthorectification parameters" <<endl;
        myfile << backward_key << endl;
        myfile <<"Conversion Factor from pixels to meters for NAD-FWD\t" << mean_conversionF_J_NadFwd <<endl;
        myfile <<"Standard deviation Conversion Factor for NAD-FWD\t" << stDev_conversionF_J_NadFwd <<endl;
        myfile <<"Conversion Factor from pixels to meters for NAD-BWD\t" << mean_conversionF_J_NadBwd <<endl;
        myfile <<"Standard deviation Conversion Factor for NAD-BWD\t" << stDev_conversionF_J_NadBwd <<endl;
        myfile.close();

        double iter = (nsteps-1);

        for(int i = (nsteps-1) ; i >= 0  ; i--)
        {
            iter;

            std::ostringstream strs;
            strs << iter;
            std::string nLev = strs.str();

            forward_key.add( ossimKeywordNames::OUTPUT_FILE_KW, ossimFilename(ap[4]) + ossimString("ortho_images/") + ossimFilename(ap[5]) + ossimString("_level") + nLev + ossimString("_orthoForward.TIF"));
            nadir_key.add( ossimKeywordNames::OUTPUT_FILE_KW, ossimFilename(ap[4]) + ossimString("ortho_images/") + ossimFilename(ap[5]) + ossimString("_level") + nLev + ossimString("_orthoNadir.TIF"));
            backward_key.add( ossimKeywordNames::OUTPUT_FILE_KW, ossimFilename(ap[4]) + ossimString("ortho_images/") + ossimFilename(ap[5]) + ossimString("_level") + nLev + ossimString("_orthoBackward.TIF"));


            double orthoRes = finalRes*pow (2, iter);
            cout << finalRes << "risFinale" << endl;
            cout << iter << "iter" << endl;
            cout << orthoRes << endl;
            std::ostringstream strsRes;
            strsRes << orthoRes;
            std::string ResParam = strsRes.str();

            forward_key.addPair(METERS_KW, ResParam);
            nadir_key.addPair( METERS_KW, ResParam);
            backward_key.addPair( METERS_KW, ResParam);

            cout << "Start forward orthorectification" << endl;
            ortho(forward_key);

            cout << "Start nadir orthorectification" << endl;
            ortho(nadir_key);

            cout << "Start backward orthorectification" << endl;
            ortho(backward_key);
				 
            // ImageHandlers & ImageGeometry instance
            ossimImageHandler* forward_handler = ossimImageHandlerRegistry::instance()->open(ossimFilename(ap[4]) + ossimString("ortho_images/") + ossimFilename(ap[5]) + ossimString("_level") + nLev + ossimString("_orthoForward.TIF"));
            ossimImageHandler* nadir_handler = ossimImageHandlerRegistry::instance()->open(ossimFilename(ap[4]) + ossimString("ortho_images/") + ossimFilename(ap[5]) + ossimString("_level") + nLev + ossimString("_orthoNadir.TIF"));
            ossimImageHandler* backward_handler = ossimImageHandlerRegistry::instance()->open(ossimFilename(ap[4]) + ossimString("ortho_images/") + ossimFilename(ap[5]) + ossimString("_level") + nLev + ossimString("_orthoBackward.TIF"));


            if(forward_handler && nadir_handler && backward_handler && raw_forward_handler && raw_nadir_handler && raw_backward_handler) // enter if exist both forward, nadir and backward
            {
                // Load ortho images
                ossimIrect bounds_forward = forward_handler->getBoundingRect(0);
                ossimIrect bounds_nadir = nadir_handler->getBoundingRect(0);
                ossimIrect bounds_backward = backward_handler->getBoundingRect(0);

                ossimRefPtr<ossimImageData> img_forward = forward_handler->getTile(bounds_forward, 0);
                ossimRefPtr<ossimImageData> img_nadir = nadir_handler->getTile(bounds_nadir, 0);
                ossimRefPtr<ossimImageData> img_backward = backward_handler->getTile(bounds_backward, 0);

                // TPs generation
                openCVtestclass *tripletCv = new openCVtestclass(img_forward, img_nadir, img_backward) ;
                tripletCv->execute();

                remove(ossimFilename(ossimFilename(ap[4]) + ossimString("temp_elevation/") + ossimFilename(ap[5])+ossimString(".TIF")));

                // From Disparity to DSM
                ossimImageGeometry* nadir_geom = nadir_handler->getImageGeometry().get();
                nadir_handler->saveImageGeometry();
                tripletCv->computeDSM(mean_conversionF_NadFwd, elev, nadir_geom); //non è corretto mettere mean_conversionF_NadFwd
                                                                                  // dovrei mettere anche mean_conversionF_NadBwd

                // Geocoded DSM generation
                ossimImageHandler *handler_disp = ossimImageHandlerRegistry::instance()->open(ossimFilename("Temp_DSM.tif"));
                handler_disp->setImageGeometry(nadir_geom);
                handler_disp->saveImageGeometry();

                ossimFilename pathDSM;
                if (i == 0)
                    pathDSM = ossimFilename(ap[4]) + ossimString("DSM/") + ossimFilename(ap[5]) + ossimString(".TIF");
                else
                    pathDSM = ossimFilename(ap[4]) + ossimString("temp_elevation/") + ossimFilename(ap[5])+ossimString(".TIF");


                ossimImageFileWriter* writer = ossimImageWriterFactoryRegistry::instance()->createWriter(pathDSM);
                writer->connectMyInputTo(0, handler_disp);

                // Add a listener to get percent complete.
                ossimStdOutProgress prog(0, true);
                writer->addListener(&prog);
                writer->execute();
                writer->removeListener(&prog);


                writer->close();
                writer = 0;
                delete tripletCv;

                cout << endl << "D.A.T.E. Plug-in has successfully generated a Digital Surface Model from your triplet!\n" << endl;
                double a;
                cin >> a;
            }
            iter --;
		}
	}     
	catch (const ossimException& e)
	{
		ossimNotify(ossimNotifyLevel_WARN) << e.what() << endl;
		return 1;
	}
  
	return 0;
}
