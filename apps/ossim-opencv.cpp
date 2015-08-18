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
#include <ossim/base/ossimEcefPoint.h>
#include <ossim/base/ossimKeywordlist.h>
#include <ossim/base/ossimKeywordNames.h>
#include <ossim/base/ossimStringProperty.h>

#include <ossim/elevation/ossimElevManager.h>
#include <ossim/elevation/ossimSrtmHandler.h>

#include "ossim/imaging/ossimImageHandlerRegistry.h"
#include "ossim/imaging/ossimImageHandler.h"
#include "ossim/imaging/ossimImageGeometry.h"
#include "ossim/imaging/ossimImageFileWriter.h"
#include "ossim/imaging/ossimImageWriterFactoryRegistry.h"
#include "ossim/imaging/ossimSrtmTileSource.h"
#include <ossim/imaging/ossimTiffWriter.h>
#include <ossim/imaging/ossimTiffOverviewBuilder.h>
#include <ossim/imaging/ossimEquationCombiner.h>

#include <ossim/init/ossimInit.h>

#include <ossim/point_cloud/ossimPointCloudImageHandler.h>
#include <ossim/point_cloud/ossimGenericPointCloudHandler.h>

#include <ossim/projection/ossimUtmProjection.h>
#include <ossim/projection/ossimUtmpt.h>

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

// PER SAR static const std::string BANDS_KW                = "b";


bool ortho (ossimKeywordlist kwl)
{
	// Make the generator
	ossimRefPtr<ossimChipperUtil> chipper = new ossimChipperUtil;

    try
    {
        chipper->initialize(kwl);
        kwl.write("KwlParameter.kwl"); // to save the kwl in a file
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
		cout << endl << "Arg number " << ap.argc() << endl;
				
		ossimKeywordlist master_key;
		ossimKeywordlist slave_key;

        master_key.addPair("image1.file", ap[1]);
        slave_key.addPair("image1.file", ap[2]);

		//Default keyword for orthorectification
		master_key.addPair(OP_KW, "ortho");
		slave_key.addPair( OP_KW, "ortho");
		
        master_key.addPair(RESAMPLER_FILTER_KW, "box");
        slave_key.addPair(RESAMPLER_FILTER_KW, "box");

        if(ap.argc() < 5) //ap.argv[0] is the application name
        {
            ap.writeErrorMessages(ossimNotify(ossimNotifyLevel_NOTICE));
            std::string errMsg = "Few arguments...";
            cout << endl << "Usage: ossim-dsm-app <input_left_image> <input_right_image> <output_results_directory> <output_dsm_name> [options] <n° steps for pyramidal>" << endl;
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
            master_key.addPair(METERS_KW, tempString1 );
            slave_key.addPair( METERS_KW, tempString1 );
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
			master_key.addPair( CUT_MIN_LAT_KW, tempString1 );
			master_key.addPair( CUT_MIN_LON_KW, tempString2 );
			master_key.addPair( CUT_MAX_LAT_KW, tempString3 );
			master_key.addPair( CUT_MAX_LON_KW, tempString4 );
			
			slave_key.addPair( CUT_MIN_LAT_KW, tempString1 );
			slave_key.addPair( CUT_MIN_LON_KW, tempString2 );
			slave_key.addPair( CUT_MAX_LAT_KW, tempString3 );
			slave_key.addPair( CUT_MAX_LON_KW, tempString4 );
		
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
	        
        cout << endl << "MASTER DIRECTORY:" << " " << ap[1] << endl;
        cout << "SLAVE DIRECTORY:"  << " " << ap[2] << endl << endl;	

        ossimImageHandler* raw_master_handler = ossimImageHandlerRegistry::instance()->open(ossimFilename(ap[1]));
        ossimImageHandler* raw_slave_handler = ossimImageHandlerRegistry::instance()->open(ossimFilename(ap[2]));        

        ossimRefPtr<ossimImageGeometry> raw_master_geom = raw_master_handler->getImageGeometry();
        ossimRefPtr<ossimImageGeometry> raw_slave_geom = raw_slave_handler->getImageGeometry();


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

        cv::Mat conv_factor_J = cv::Mat::zeros(3,3, CV_64F);
        cv::Mat conv_factor_I = cv::Mat::zeros(3,3, CV_64F);

        for (int i=0 ; i<3 ; i++) //LAT
        {
            for (int j=0 ; j<3 ; j++) //LON
            {
                ossimGpt groundPoint(lat_max-i*Dlat,lon_min+j*Dlon,MinHeight -50);
                ossimGpt groundPointUp(lat_max-i*Dlat,lon_min+j*Dlon,MaxHeight + 50);

                ossimDpt imagePoint(0.,0.);
                ossimDpt imagePointUp(0.,0.);

                raw_master_geom->worldToLocal(groundPoint,imagePoint);        // to obtain master imagePoint
                raw_master_geom->worldToLocal(groundPointUp,imagePointUp);

                myfile << "MASTER IMAGE" << "\t" << "Ground point" << groundPoint << "\t" << "Image point" << imagePoint << "\t" << "Ground point up" << groundPointUp << "\t" << "Image point up" << imagePointUp << "\t";

                double DeltaI_Master = imagePointUp.x - imagePoint.x;
                double DeltaJ_Master = imagePointUp.y - imagePoint.y;

                raw_slave_geom->worldToLocal(groundPoint,imagePoint);
                raw_slave_geom->worldToLocal(groundPointUp,imagePointUp);

                myfile << "SLAVE IMAGE" << "\t" << "Ground point" << groundPoint << "\t" << "Image point" << imagePoint << "\t" << "Ground point up" << groundPointUp << "\t" << "Image point up" << imagePointUp << "\t" << endl;

                double DeltaI_Slave = imagePointUp.x - imagePoint.x;
                double DeltaJ_Slave = imagePointUp.y - imagePoint.y;

                conv_factor_J.at<double>(i,j) = DeltaJ_Slave - DeltaJ_Master;
                conv_factor_I.at<double>(i,j) = DeltaI_Slave - DeltaI_Master;
            }
        }

        //cout << conv_factor_J << endl;
        //cout << conv_factor_I << endl;

        cv::Scalar mean_conv_factor_J, stDev_conv_factor_J;
        cv::meanStdDev(conv_factor_J, mean_conv_factor_J, stDev_conv_factor_J);

        cv::Scalar mean_conv_factor_I, stDev_conv_factor_I;
        cv::meanStdDev(conv_factor_I, mean_conv_factor_I, stDev_conv_factor_I);

        double stDev_conversionF_J = stDev_conv_factor_J.val[0];
        double mean_conversionF_J = mean_conv_factor_J.val[0]/(MaxHeight -MinHeight + 100);
        double stDev_conversionF_I = stDev_conv_factor_I.val[0];
        double mean_conversionF_I = mean_conv_factor_I.val[0]/(MaxHeight -MinHeight + 100);
        double mean_conversionF = ((sqrt((mean_conv_factor_J.val[0]*mean_conv_factor_J.val[0]) + (mean_conv_factor_I.val[0]*mean_conv_factor_I.val[0]))/(MaxHeight -MinHeight + 100)))*1.0;

        cout << "J Conversion Factor from pixels to meters\t" << mean_conversionF_J << endl;
        cout << "Standard deviation J Conversion Factor\t" << stDev_conversionF_J << endl << endl;
        cout << "I Conversion Factor from pixels to meters\t" << mean_conversionF_I << endl;
        cout << "Standard deviation I Conversion Factor\t" << stDev_conversionF_I << endl << endl;
        cout << "Total Conversion Factor from pixels to meters\t" << mean_conversionF << endl << endl;

        // END CONVERSION FACTOR COMPUTATION ****************************************

        myfile << endl << "Master orthorectification parameters" <<endl;
        myfile << master_key << endl;
        myfile << "Slave orthorectification parameters" <<endl;
        myfile << slave_key << endl;
        myfile <<"Conversion Factor from pixels to meters\t" << mean_conversionF_J <<endl;
        myfile <<"Standard deviation Conversion Factor\t" << stDev_conversionF_J <<endl;
        myfile.close();


        double iter = (nsteps-1);

        for(int i = (nsteps-1) ; i >= 0  ; i--)
        {
            iter;

            std::ostringstream strs;
            strs << iter;
            std::string nLev = strs.str();

            // Elevation manager instance
            ossimElevManager* elev = ossimElevManager::instance();

            master_key.add( ossimKeywordNames::OUTPUT_FILE_KW, ossimFilename(ap[3]) + ossimString("ortho_images/") + ossimFilename(ap[4]) + ossimString("_level") + nLev + ossimString("_orthoMaster.TIF"));
            slave_key.add( ossimKeywordNames::OUTPUT_FILE_KW, ossimFilename(ap[3]) + ossimString("ortho_images/") + ossimFilename(ap[4]) + ossimString("_level") + nLev + ossimString("_orthoSlave.TIF"));

            double orthoRes = finalRes*pow (2, iter);
            cout << finalRes << "\t final res" << endl;
            cout << iter << "iter" << endl;
            cout << orthoRes << "\t resolution" << endl;
            std::ostringstream strsRes;
            strsRes << orthoRes;
            std::string ResParam = strsRes.str();

            master_key.addPair(METERS_KW, ResParam);
            slave_key.addPair( METERS_KW, ResParam);

            cout << "Start master orthorectification level " << nLev << endl;
            ortho(master_key);

            cout << "Start slave orthorectification level " << nLev << endl;
            ortho(slave_key);

            // ImageHandlers & ImageGeometry instance
            ossimImageHandler* master_handler = ossimImageHandlerRegistry::instance()->open(ossimFilename(ap[3]) + ossimString("ortho_images/") + ossimFilename(ap[4]) + ossimString("_level") + nLev + ossimString("_orthoMaster.TIF"));
            ossimImageHandler* slave_handler = ossimImageHandlerRegistry::instance()->open(ossimFilename(ap[3]) + ossimString("ortho_images/") + ossimFilename(ap[4]) + ossimString("_level") + nLev + ossimString("_orthoSlave.TIF"));


            if(master_handler && slave_handler && raw_master_handler && raw_slave_handler) // enter if exist both master and slave
            {
                // Load ortho images
                ossimIrect bounds_master = master_handler->getBoundingRect(0);
                ossimIrect bounds_slave = slave_handler->getBoundingRect(0);

                ossimRefPtr<ossimImageData> img_master = master_handler->getTile(bounds_master, 0);
                ossimRefPtr<ossimImageData> img_slave = slave_handler->getTile(bounds_slave, 0);

                // TPs and disp map generation
                openCVtestclass *test = new openCVtestclass(img_master, img_slave) ;
                test->execute();

                remove(ossimFilename(ossimFilename(ap[3]) + ossimString("temp_elevation/") + ossimFilename(ap[4])+ossimString(".TIF")));

                // From Disparity to DSM
                ossimImageGeometry* master_geom = master_handler->getImageGeometry().get();
                master_handler->saveImageGeometry();
                test->computeDSM(mean_conversionF/1.0, elev, master_geom);

                // Geocoded DSM generation
                ossimImageHandler *handler_disp = ossimImageHandlerRegistry::instance()->open(ossimFilename("Temp_DSM.tif"));
                handler_disp->setImageGeometry(master_geom); // now I have a "geo" image handler
                cout << "size" << master_geom->getImageSize() << endl;
                handler_disp->saveImageGeometry();

                ossimFilename pathDSM;
                if (i == 0)
                    pathDSM = ossimFilename(ap[3]) + ossimString("DSM/") + ossimFilename(ap[4]) + ossimString(".TIF");
                else
                    pathDSM = ossimFilename(ap[3]) + ossimString("temp_elevation/") + ossimFilename(ap[4])+ossimString(".TIF");

                ossimImageFileWriter* writer = ossimImageWriterFactoryRegistry::instance()->createWriter(pathDSM);
                writer->connectMyInputTo(0, handler_disp);
                writer->execute();
                int a;
                cin >>a;
                writer->close();
                writer = 0;
                handler_disp = 0;
                test= 0;


            }
            iter --;
        }
    cout << endl << "D.A.T.E. Plug-in has successfully generated a Digital Surface Model from your stereo-pair!\n" << endl;
    }

	catch (const ossimException& e)
	{
		ossimNotify(ossimNotifyLevel_WARN) << e.what() << endl;
		return 1;
	}
  
	return 0;
}
