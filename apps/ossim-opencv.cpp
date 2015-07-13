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

#include <ossim/init/ossimInit.h>

#include <ossim/util/ossimChipperUtil.h>

#include "ossim/imaging/ossimImageHandlerRegistry.h"
#include "ossim/imaging/ossimImageHandler.h"
#include "ossim/imaging/ossimImageGeometry.h"
#include "ossim/imaging/ossimImageFileWriter.h"
#include "ossim/imaging/ossimImageWriterFactoryRegistry.h"
#include "ossim/imaging/ossimSrtmTileSource.h"

#include <ossim/elevation/ossimElevManager.h>
#include <ossim/elevation/ossimSrtmHandler.h>
#include <ossim/projection/ossimUtmProjection.h>
#include <ossim/projection/ossimUtmpt.h>
#include <ossim/base/ossimEcefPoint.h>
#include "ossimOpenCvTPgenerator.h"
#include "openCVtestclass.h"
#include "ossimOpenCvDisparityMapGenerator.h"

#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <opencv/cv.h>

#include <ossim/base/ossimKeywordlist.h>
#include <ossim/base/ossimKeywordNames.h>

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

    try
    {
        chipper->initialize(kwl);
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
		
		//Default keyword for orthorectification
		master_key.addPair(OP_KW, "ortho");
		slave_key.addPair( OP_KW, "ortho");
		
		master_key.addPair(RESAMPLER_FILTER_KW, "box");
		slave_key.addPair( RESAMPLER_FILTER_KW, "box");
		
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
		
		cout << "Orthoimages resolution = " << tempString1 <<" meters"<< endl << endl;
        
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
                    //ossim_float64 hgtAboveEllipsoid = ossimElevManager::instance()->getHeightAboveEllipsoid(world_point);
                    //ossim_float64 geoidOffset = ossimGeoidManager::instance()->offsetFromEllipsoid(world_point);
                    //ossim_float64 mslOffset = 0.0;

                    HeightAboveMSL.push_back(hgtAboveMsl);
                }
            }

            MinHeight = *min_element(HeightAboveMSL.begin(), HeightAboveMSL.end());
            MaxHeight = *max_element(HeightAboveMSL.begin(), HeightAboveMSL.end());
            cout << "Min height for this tile is " << std::setprecision(6) << MinHeight << " m" << endl;
            cout << "Max height for this tile is " << std::setprecision(6) << MaxHeight << " m" << endl;


/*
            //Calcolo max e min

			// Elevation manager instance
			ossim_float64 hgtAboveMsl = ossimElevManager::instance()->getHeightAboveMSL(world_point);
			ossim_float64 hgtAboveEllipsoid = ossimElevManager::instance()->getHeightAboveEllipsoid(world_point);
			ossim_float64 geoidOffset = ossimGeoidManager::instance()->offsetFromEllipsoid(world_point);
            ossim_float64 mslOffset = 0.0;


			ossimDrect boundBox = ossimDrect(lon_min, lat_max, lon_max, lat_min); //vuole dei double in input
						
			ossimSrtmTileSource* dsm = new ossimSrtmTileSource();
			std::vector<std::string> cells;	
            ossimElevManager::instance()->getCellsForBounds(lat_min,lon_min,lat_max,lon_max, cells);

   			cout << cells[0] << endl;

            ossimRefPtr<ossimImageData> tile = dsm->getTile(boundBox);


            std::vector<ossim_float64> minvalue (1,0.0);
            std::vector<ossim_float64> maxvalue (1,100.0);

            tile->computeMinMaxPix(minvalue, maxvalue);  //mi dà un void...quindi come faccio a tirar fuori qualcosa?


            dsm->initialize();

            dsm->open();

            cout << "qt POWER " << dsm->getMaxPixelValue() << endl;


			ossimSrtmSupportData sd;

			sd.setFilename(cells[0], true);

			cout << "stronzi " << sd.getMaxPixelValue() << endl;
			cout << "stronzi 2 " << sd.getMinPixelValue() << endl;

			return 0;
*/
			//ossimRefPtr<ossimImageData> geom = dsm->getTile(boundBox);
			
			//std::vector<ossim_float64> minvalue (1,0.0);
			//std::vector<ossim_float64> maxvalue (1,100.0);

			//double pippo = geom->getWidth();

//			cout << pippo << endl;

			//geom->computeMinMaxPix(minvalue, maxvalue);
			
//			cout << minvalue[0] << "bella per noi " << maxvalue[0] << endl; 


			//ossimRefPtr<ossimImageData> geom = 
			//ossimSrtmTileSource::getImageGeometry()
			
			//ossimRefPtr<ossimImageData> geom = ossimElevManager::instance()->ImageGeometry();

			//Prendo il tile con getTile
			//ossimRefPtr<ossimImageData> SrtmTile = ossimSrtmTileSource::getTile(boundBox);


/*
			double SrtmMinHeight = ossimElevManager::instance()->getMinHeightAboveMSL();
			double SrtmMaxHeight = ossimElevManager::instance()->getMaxHeightAboveMSL();
			cout << SrtmMinHeight << "MinHeight" << endl <<  SrtmMaxHeight << "MaxHeight" << endl;
			
*/


/*
			ossimSrtmHandler mySrtm (data/elevation/srtm/v4_1");

			double minH = mySrtm.getMinHeightAboveMSL();
						
			ossimGpt myLocation;
			ossimElevManager::instance()->getHeightAboveMSL(myLocation);
			double maxH = ossimElevManager::instance()->getMaxHeightAboveMSL();
			
			cout << "max height = " << maxH << endl << endl;
					
		ossimElevManager* elev = ossimElevManager::instance();     
		ossim_float64 hgtAboveMSL = elev->getHeightAboveMSL(myLocation);
		double minH = elev->getMinHeightAboveMSL();	
		cout << "min height = " << minH << endl << endl;
		//ossimElevSource* source = ossimElevSource::ossimElevSource();	
		//ossim_float64 hgtAboveMsl_min = source->getMinHeightAboveMSL(); 
		//ossim_float64 hgtAboveMsl_min = ossimElevSource()->getMinHeightAboveMSL(); 


		ossimElevSource::ossimElevSource(const ossimElevSource& src)
			:ossimSource(src),
		theMinHeightAboveMSL(src.theMinHeightAboveMSL),
		theMaxHeightAboveMSL(src.theMaxHeightAboveMSL),
		
		ossim_float64 hgtAboveMsl_min = ossimElevSource::getMinHeightAboveMSL(); 
		ossim_float64 hgtAboveMsl_max = ossimElevSource::getMaxHeightAboveMSL(); 

		cout << "Min Height above MSL" << hgtAboveMsl_min << endl;
		//cout << "Max Height above MSL" << hgtAboveMsl_max << endl;

			if(ossim::isnan(hgtAboveEllipsoid)||ossim::isnan(hgtAboveMsl))
			{
				mslOffset = ossim::nan();
			}
			else
			{
				mslOffset = hgtAboveEllipsoid - hgtAboveMsl;
			}
   
			std::vector<ossimFilename> cellList;
			ossimElevManager::instance()->getOpenCellList(cellList);
   
			if (!cellList.empty())
			{
				cout << "Opened cell:            " << cellList[0] << "\n";
			}
			else
			{
				cout << "Did not find cell for point: " << lat_min << "\n";
			}   

			cout << "MSL to ellipsoid delta for ll point: ";
			if (!ossim::isnan(mslOffset))
			{
				cout << std::setprecision(15) << mslOffset;
			}
			else
			{
				cout << "nan";
			}
   
			cout << "\nHeight above MSL for ll point:       ";
			if (!ossim::isnan(hgtAboveMsl))
			{
				cout << std::setprecision(15) << hgtAboveMsl;
			}
			else
			{
				cout << "nan";
			}
   
			cout << "\nHeight above ellipsoid for ll point: ";
			if (!ossim::isnan(hgtAboveEllipsoid))
			{
				cout << std::setprecision(15) << hgtAboveEllipsoid << "\n";
			}
			else
			{
				cout << "nan" << "\n";
			}
   
			cout << "Geoid value for ll point:            ";
			if (!ossim::isnan(geoidOffset))
			{
				cout << std::setprecision(15) << geoidOffset << std::endl;
			}
			else
			{
				cout << "nan" << std::endl;
            }*/
        }
		// End of arg parsing
		
		ap.reportRemainingOptionsAsUnrecognized();
		if (ap.errors())
		{
			ap.writeErrorMessages(ossimNotify(ossimNotifyLevel_NOTICE));
			std::string errMsg = "Unknown option...";
			throw ossimException(errMsg);
		}
		
		ossimString  key	= "";
		
		if(ap.argc() >= 5) //ap.argv[0] is the application name
		{
			master_key.add( ossimKeywordNames::OUTPUT_FILE_KW, ap[3]);		
			slave_key.add( ossimKeywordNames::OUTPUT_FILE_KW, ap[4]);
						
			master_key.addPair("image1.file", ap[1]);
			slave_key.addPair("image1.file", ap[2]);
		}
		else 
		{
			ap.writeErrorMessages(ossimNotify(ossimNotifyLevel_NOTICE));
			std::string errMsg = "Few arguments...";
			cout << endl << "Usage: ossim-dsm-app <input_left_image> <input_right_image> <output_ortho_left_image> <output_ortho_right_image> [options] <output_DSM>" << endl;
			cout << "Options:" << endl;
			cout << "--cut-bbox-ll <min_lat> <min_lon> <max_lat> <max_lon> \t Specify a bounding box with the minimum"   << endl;   
			cout << "\t\t\t\t\t\t\tlatitude/longitude and max latitude/longitude" << endl; 
			cout << "\t\t\t\t\t\t\tin decimal degrees." << endl; 
			cout << "--meters <meters> \t\t\t\t\t Specify a size (in meters) for a resampling"   << endl<< endl; 
			throw ossimException(errMsg);
		}
		
		//END PARSER****************************
	        
        cout << endl << "MASTER DIRECTORY:" << " " << ap[1] << endl;
        cout << "SLAVE DIRECTORY:"  << " " << ap[2] << endl << endl;	
	
		
	    cout << "Start master orthorectification" << endl;
        //ortho(master_key);
	
		cout << "Start slave orthorectification" << endl;
        //ortho(slave_key);
			
		// Elevation manager instance
		ossimElevManager* elev = ossimElevManager::instance();		
  
		// ImageHandlers & ImageGeometry instance
		ossimImageHandler* master_handler = ossimImageHandlerRegistry::instance()->open(ossimFilename(ap[3]));             
		ossimImageHandler* slave_handler = ossimImageHandlerRegistry::instance()->open(ossimFilename(ap[4]));
  
		ossimImageHandler* raw_master_handler = ossimImageHandlerRegistry::instance()->open(ossimFilename(ap[1]));
		ossimImageHandler* raw_slave_handler = ossimImageHandlerRegistry::instance()->open(ossimFilename(ap[2]));
                      
    				
		if(master_handler && slave_handler && raw_master_handler && raw_slave_handler) // enter if exist both master and slave  
		{
			// Load ortho images
			ossimIrect bounds_master = master_handler->getBoundingRect(0); 			
			ossimIrect bounds_slave = slave_handler->getBoundingRect(0);   
			ossimRefPtr<ossimImageData> img_master = master_handler->getTile(bounds_master, 0);          
			ossimRefPtr<ossimImageData> img_slave = slave_handler->getTile(bounds_slave, 0); 

			// TPs generation 
			openCVtestclass *test = new openCVtestclass(img_master, img_slave) ; 					
   			test->execute();

			// CONVERSION FACTOR (from pixels to meters) COMPUTATION *************
			ossimRefPtr<ossimImageGeometry> raw_master_geom = raw_master_handler->getImageGeometry();    
			ossimRefPtr<ossimImageGeometry> raw_slave_geom = raw_slave_handler->getImageGeometry(); 
          
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

            //for (int i=0 ; i<3 ; i++) //LAT
            //{
            int index = 0;
            std::vector<double> xVectorMaster;
            std::vector<double> yVectorMaster;
            std::vector<double> zVectorMaster;
            std::vector<double> xVectorSlave;
            std::vector<double> yVectorSlave;
            std::vector<double> zVectorSlave;

                // VERSOR COMPUTATION
                for (int j=1 ; j<4 ; j++)
                {                  
                    ossimDpt localPt (10000, j+(j-1)*10000); //Trento GeoEye
                    //ossimDpt localPt (11000, j+(j-1)*8300); // Argenta
                    ossimGpt worldPtMaster (0.,0.);
                    ossimGpt worldPtUpMaster(0., 0.);
                    ossimGpt worldPtSlave(0.,0.);
                    ossimGpt worldPtUpSlave(0., 0.);

                    raw_master_geom->localToWorld(localPt, 0, worldPtMaster);
                    cout << "MASTER\t" << endl << endl;
                    cout << localPt  << endl;   // local coordinates for this cycle
                    cout << worldPtMaster  << endl;   // ground coordinates @ 0 m
                    raw_master_geom->localToWorld(localPt, 1400, worldPtUpMaster);
                    cout << worldPtUpMaster << endl;  // ground coordinates @ 1400 m

                    raw_slave_geom->localToWorld(localPt, 0, worldPtSlave);
                    cout << "SLAVE\t" << endl << endl;
                    cout << localPt  << endl;   // local coordinates for this cycle
                    cout << worldPtSlave  << endl;   // ground coordinates @ 0 m
                    raw_slave_geom->localToWorld(localPt, 1400, worldPtUpSlave);
                    cout << worldPtUpSlave << endl;  // ground coordinates @ 1400 m

                    ossimEcefPoint ECEFgenMaster(worldPtMaster);
                    ossimEcefPoint ECEFgenUPMaster(worldPtUpMaster);
                    ossimEcefPoint ECEFgenSlave(worldPtSlave);
                    ossimEcefPoint ECEFgenUPSlave(worldPtUpSlave);

                    double xDiffsquareMaster = (ECEFgenMaster.x() - ECEFgenUPMaster.x())*(ECEFgenMaster.x() - ECEFgenUPMaster.x());
                    double yDiffsquareMaster = (ECEFgenMaster.y() - ECEFgenUPMaster.y())*(ECEFgenMaster.y() - ECEFgenUPMaster.y());
                    double zDiffsquareMaster = (ECEFgenMaster.z() - ECEFgenUPMaster.z())*(ECEFgenMaster.z() - ECEFgenUPMaster.z());

                    double normMaster = sqrt(xDiffsquareMaster + yDiffsquareMaster + zDiffsquareMaster);

                    double xVersorCompMaster = (ECEFgenMaster.x() - ECEFgenUPMaster.x())/(normMaster);
                    double yVersorCompMaster = (ECEFgenMaster.y() - ECEFgenUPMaster.y())/(normMaster);
                    double zVersorCompMaster = (ECEFgenMaster.z() - ECEFgenUPMaster.z())/(normMaster);

                    xVectorMaster.push_back(xVersorCompMaster);
                    yVectorMaster.push_back(yVersorCompMaster);
                    zVectorMaster.push_back(zVersorCompMaster);

                    cout << "Point" << index++ << "\t" << endl;
                    cout << "x component master:\t" << xVersorCompMaster << endl << endl;
                    cout << "y component master:\t" << yVersorCompMaster << endl << endl;
                    cout << "z component master:\t" << zVersorCompMaster << endl << endl;

                    double xDiffsquareSlave = (ECEFgenSlave.x() - ECEFgenUPSlave.x())*(ECEFgenSlave.x() - ECEFgenUPSlave.x());
                    double yDiffsquareSlave = (ECEFgenSlave.y() - ECEFgenUPSlave.y())*(ECEFgenSlave.y() - ECEFgenUPSlave.y());
                    double zDiffsquareSlave = (ECEFgenSlave.z() - ECEFgenUPSlave.z())*(ECEFgenSlave.z() - ECEFgenUPSlave.z());

                    double normSlave = sqrt(xDiffsquareSlave + yDiffsquareSlave + zDiffsquareSlave);

                    double xVersorCompSlave = (ECEFgenSlave.x() - ECEFgenUPSlave.x())/(normSlave);
                    double yVersorCompSlave = (ECEFgenSlave.y() - ECEFgenUPSlave.y())/(normSlave);
                    double zVersorCompSlave = (ECEFgenSlave.z() - ECEFgenUPSlave.z())/(normSlave);

                    xVectorSlave.push_back(xVersorCompSlave);
                    yVectorSlave.push_back(yVersorCompSlave);
                    zVectorSlave.push_back(zVersorCompSlave);

                    cout << "x component slave:\t" << xVersorCompSlave << endl << endl;
                    cout << "y component slave:\t" << yVersorCompSlave << endl << endl;
                    cout << "z component slave:\t" << zVersorCompSlave << endl << endl;

                    //cout << index++ << "\t" << "punto terra a quota 0 m x:\t" << ECEFgen.x() << endl << "\ty:\t" << ECEFgen.y() << "\tz:\t" << ECEFgen.z() << endl << endl;
                    //cout << "\t" << "punto terra a quota 1400 m x:\t" << ECEFgenUP.x() << endl << "\ty:\t" << ECEFgenUP.y() << "\tz:\t" << ECEFgenUP.z() << endl << endl;
                    //ossimUtmpt UTMgenPt(worldPt);
                    //ossimUtmpt UTMgenPtUP(worldPtUp);
                    //cout << index++ << "\t" << "punto terra a quota 0 m EST:\t" << UTMgenPt.easting() << endl << "\tNORD:\t" << UTMgenPt.northing() <<endl << endl;
                    //cout << "\t" << "punto terra a quota 1400 m EST:\t" << UTMgenPtUP.easting() << endl << "\tNORD:\t" << UTMgenPtUP.northing() <<endl << endl;

                //  raw_master_geom->worldToLocal(point_object, image_point);
                //  cout << image_point << endl; //coordinate immagine corrispondenti al punto centrale
                }
            //}
            double sum = 0;
            for (vector<double>::iterator j = xVectorMaster.begin(); j!=xVectorMaster.end(); j++)
                sum += *j;
            double xVersorMeanMaster = sum/(xVectorMaster.size());

            sum = 0;
            for (vector<double>::iterator j = yVectorMaster.begin(); j!=yVectorMaster.end(); j++)
                sum += *j;
            double yVersorMeanMaster = sum/(yVectorMaster.size());

            sum = 0;
            for (vector<double>::iterator j = zVectorMaster.begin(); j!=zVectorMaster.end(); j++)
                sum += *j;
            double zVersorMeanMaster = sum/(zVectorMaster.size());

            cout << xVersorMeanMaster  << "\t" << yVersorMeanMaster << "\t" << zVersorMeanMaster << "\t" << endl;

            sum = 0;
            for (vector<double>::iterator j = xVectorSlave.begin(); j!=xVectorSlave.end(); j++)
                sum += *j;
            double xVersorMeanSlave = sum/(xVectorSlave.size());

            sum = 0;
            for (vector<double>::iterator j = yVectorSlave.begin(); j!=yVectorSlave.end(); j++)
                sum += *j;
            double yVersorMeanSlave = sum/(yVectorSlave.size());

            sum = 0;
            for (vector<double>::iterator j = zVectorSlave.begin(); j!=zVectorSlave.end(); j++)
                sum += *j;
            double zVersorMeanSlave = sum/(zVectorSlave.size());

            cout << xVersorMeanSlave << "\t" << yVersorMeanSlave  << "\t" << zVersorMeanSlave  << "\t" << endl;

            double xComponentMean = (xVersorMeanMaster + xVersorMeanSlave) /2.0;
            double yComponentMean = (yVersorMeanMaster + yVersorMeanSlave) /2.0;
            double zComponentMean = (zVersorMeanMaster + zVersorMeanSlave) /2.0;

            cout << "X, Y and Z components of the mean versor\t" << endl;
            cout << xComponentMean << "\t" << yComponentMean  << "\t" << zComponentMean  << "\t" << endl;

            ossimGpt meanGroundPt((lat_max + lat_min)/2.0, (lon_max + lon_min)/2.0 , (MaxHeight - MinHeight)/2.0);
            ossimEcefPoint ECEFmeanGroundPt(meanGroundPt);

            cout << "coordinate geografiche a terra del punto medio:\t" << meanGroundPt << endl;
            cout << "coordinate ECEF a terra del punto medio:\t" << ECEFmeanGroundPt << endl;

            // INCLINED PLANE FOR QUASI-EPIPOLAR IMAGES
            // std::vector<double> x = ECEFmeanGroundPt.x();
            // std::vector<double> y = ECEFmeanGroundPt.y();
            // std::vector<double> z = ECEFmeanGroundPt.z();
            double d = -xComponentMean*ECEFmeanGroundPt.x() - yComponentMean*ECEFmeanGroundPt.y() - zComponentMean*ECEFmeanGroundPt.z();

            std::vector<double> xPt ;
            std::vector<double> yPt ;
            std::vector<double> zPt ;

            cout << "d:\t" << d << endl;

            // coordinate x e y dell'angolo in basso a sx
            double frstComp = ECEFmeanGroundPt.x() - 5000;
            double scndComp = ECEFmeanGroundPt.y() - 5000;

            cout << frstComp << "\tx dell'angolo in basso a sx" << endl;
            cout << scndComp << "\ty dell'angolo in basso a sx" << endl;

            for (int i=0; i<3 ; i++)
            {
                for (int j=0; j<3; j++)
                {
                   double yComp = scndComp + (j*5000);
                   yPt.push_back(yComp);
                }
                double xComp = frstComp + (i*5000);
            xPt.push_back(xComp);
            }

            cout << xPt[0] << "\t" << xPt[1] << "\t" << xPt[2] << endl << endl;
            cout << yPt[0] << "\t" << yPt[1] << "\t" << yPt[2] << endl << endl;

            // z computation
            std::vector<double> trdCompPt ;

            for (int i=0; i<3 ; i++)
            {
                for (int j=0; j<3; j++)
                {
                        double zComp = (-d - xComponentMean*xPt[i] - yComponentMean*yPt[j]) / zComponentMean;
                        trdCompPt.push_back(zComp);
                }
            }

            cout << trdCompPt[0] << "\t" << trdCompPt[1] << "\t" << trdCompPt[2] << "\t" <<
                    trdCompPt[3] << "\t" << trdCompPt[4] << "\t" << trdCompPt[5] << "\t" <<
                    trdCompPt[6] << "\t" << trdCompPt[7] << "\t" << trdCompPt[8] << endl;

            vector<ossimEcefPoint> GriddedPt;

            for (int i=0; i<3 ; i++)
            {
                for (int j=0; j<3; j++)
                {
                        ossimEcefPoint Pt(xPt[i], yPt[j], trdCompPt[i*3+j]);
                        GriddedPt.push_back(Pt);
                }
            }

            cout << "Point1\t" << GriddedPt[0] << "\t" << "Point2\t" <<GriddedPt[1] << "\t" << "Point3\t" << GriddedPt[2] << "\t" <<
                  "Point4\t" <<  GriddedPt[3] << "\t" << "Point5\t" << GriddedPt[4] << "\t" << "Point6\t" << GriddedPt[5] << "\t" <<
                  "Point7\t" <<  GriddedPt[6] << "\t" << "Point8\t" << GriddedPt[7] << "\t" << "Point9\t" << GriddedPt[8] << endl;


/*
            double trdCompPt1 = (-d - xComponentMean*xPt[0] - yComponentMean*yPt[0]) / zComponentMean;
            double trdCompPt2 = (-d - xComponentMean*xPt[0] - yComponentMean*yPt[1]) / zComponentMean;
            double trdCompPt3 = (-d - xComponentMean*xPt[0] - yComponentMean*yPt[2]) / zComponentMean;
            double trdCompPt4 = (-d - xComponentMean*xPt[1] - yComponentMean*yPt[0]) / zComponentMean;
            double trdCompPt5 = (-d - xComponentMean*xPt[1] - yComponentMean*yPt[1]) / zComponentMean;
            double trdCompPt6 = (-d - xComponentMean*xPt[1] - yComponentMean*yPt[2]) / zComponentMean;
            double trdCompPt7 = (-d - xComponentMean*xPt[2] - yComponentMean*yPt[0]) / zComponentMean;
            double trdCompPt8 = (-d - xComponentMean*xPt[2] - yComponentMean*yPt[1]) / zComponentMean;
            double trdCompPt9 = (-d - xComponentMean*xPt[2] - yComponentMean*yPt[2]) / zComponentMean;

            cout << trdCompPt1 << "\t" << trdCompPt2 << "\t" << trdCompPt3 << "\t" <<
                    trdCompPt4 << "\t" << trdCompPt5 << "\t" << trdCompPt6 << "\t" <<
                    trdCompPt7 << "\t" << trdCompPt8 << "\t" << trdCompPt9 << endl;

            ossimEcefPoint Pt1(xPt[0], yPt[0], trdCompPt1);
            ossimEcefPoint Pt2(xPt[0], yPt[1], trdCompPt2);
            ossimEcefPoint Pt3(xPt[0], yPt[2], trdCompPt3);
            ossimEcefPoint Pt4(xPt[1], yPt[0], trdCompPt4);
            ossimEcefPoint Pt5(xPt[1], yPt[1], trdCompPt5);
            ossimEcefPoint Pt6(xPt[1], yPt[2], trdCompPt6);
            ossimEcefPoint Pt7(xPt[2], yPt[0], trdCompPt7);
            ossimEcefPoint Pt8(xPt[2], yPt[1], trdCompPt8);
            ossimEcefPoint Pt9(xPt[2], yPt[2], trdCompPt9);

            cout << "Point1\t" << Pt1 << endl;
            cout << "Point2\t" << Pt2 << endl;
            cout << "Point3\t" << Pt3 << endl;
            cout << "Point4\t" << Pt4 << endl;
            cout << "Point5\t" << Pt5 << endl;
            cout << "Point6\t" << Pt6 << endl;
            cout << "Point7\t" << Pt7 << endl;
            cout << "Point8\t" << Pt8 << endl;
            cout << "Point9\t" << Pt9 << endl;
*/


/*
            // From Disparity to DSM
            ossimImageGeometry* point_geom = master_handler->getImageGeometry().get();
            test->computeDSM(mean_conversionF, elev, point_geom);

            // Geocoded DSM generation
            ossimImageHandler *handler_disp = ossimImageHandlerRegistry::instance()->open(ossimFilename("Temp_DSM.tif"));
            handler_disp->setImageGeometry(point_geom);
            ossimImageFileWriter* writer = ossimImageWriterFactoryRegistry::instance()->createWriter(ossimFilename(ap[5]));
            writer->connectMyInputTo(0, handler_disp);
            writer->execute();

*/





          //  for (int i=0; i<3; i++)
          //  {
            //    double trdComp = (-d - xComponentMean*xPt[i] - yComponentMean*yPt[i]) / zComponentMean;
              //  zPt.push_back(trdComp);
          //  }

           // cout << zPt[0] << "\t" << zPt[1] << "\t" << zPt[2] <<endl << endl;

            //double ECEFzComponent = (-d - xComponentMean*x - yComponentMean*y) / zComponentMean;
            //cout << ECEFzComponent << endl;

			for (int i=0 ; i<3 ; i++) //LAT
			{
				for (int j=0 ; j<3 ; j++) //LON
				{
                    ossimGpt punto_terra(lat_max-i*Dlat,lon_min+j*Dlon,MinHeight -50);
                    ossimGpt punto_terra_up(lat_max-i*Dlat,lon_min+j*Dlon,MaxHeight + 50);
					
					ossimDpt punto_img(0.,0.);
					ossimDpt punto_img_up(0.,0.);
				
					raw_master_geom->worldToLocal(punto_terra,punto_img);        //con qst trasf ottengo punto_img della master      
                    raw_master_geom->worldToLocal(punto_terra_up,punto_img_up);

					myfile << "MASTER IMAGE" << "\t" << "Ground point" << punto_terra << "\t" << "Image point" << punto_img << "\t" << "Ground point up" << punto_terra_up << "\t" << "Image point up" << punto_img_up << "\t";    
					
					double DeltaI_Master = punto_img_up.x - punto_img.x;
					double DeltaJ_Master = punto_img_up.y - punto_img.y;
					    
					raw_slave_geom->worldToLocal(punto_terra,punto_img);       
					raw_slave_geom->worldToLocal(punto_terra_up,punto_img_up);    

					myfile << "SLAVE IMAGE" << "\t" << "Ground point" << punto_terra << "\t" << "Image point" << punto_img << "\t" << "Ground point up" << punto_terra_up << "\t" << "Image point up" << punto_img_up << "\t" << endl;    
					
					double DeltaI_Slave = punto_img_up.x - punto_img.x;
					double DeltaJ_Slave = punto_img_up.y - punto_img.y;
					
					conv_factor_J.at<double>(i,j) = DeltaJ_Slave - DeltaJ_Master; // conv_factor for ACROSS-track imgs
					conv_factor_I.at<double>(i,j) = DeltaI_Slave - DeltaI_Master; // conv_factor for ALONG-track imgs 
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
			
            double mean_conversionF = (sqrt((mean_conv_factor_J.val[0]*mean_conv_factor_J.val[0]) + (mean_conv_factor_I.val[0]*mean_conv_factor_I.val[0]))/(MaxHeight -MinHeight + 100));
			
			cout << "J Conversion Factor from pixels to meters\t" << mean_conversionF_J << endl;
			cout << "Standard deviation J Conversion Factor\t" << stDev_conversionF_J << endl << endl;

			cout << "I Conversion Factor from pixels to meters\t" << mean_conversionF_I << endl;
			cout << "Standard deviation I Conversion Factor\t" << stDev_conversionF_I << endl << endl;
			
			cout << "Total Conversion Factor from pixels to meters\t" << mean_conversionF << endl;
			// END CONVERSION FACTOR COMPUTATION ****************************************
			
			myfile << endl << "Master orthorectification parameters" <<endl;
			myfile << master_key << endl;
			myfile << "Slave orthorectification parameters" <<endl;
			myfile << slave_key << endl;
			myfile <<"Conversion Factor from pixels to meters\t" << mean_conversionF_J <<endl;
			myfile <<"Standard deviation Conversion Factor\t" << stDev_conversionF_J <<endl; 
			myfile.close();			
						
/*			
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
*/				
				
			// From Disparity to DSM
			ossimImageGeometry* master_geom = master_handler->getImageGeometry().get();			
			test->computeDSM(mean_conversionF, elev, master_geom);
						
			// Geocoded DSM generation
			ossimImageHandler *handler_disp = ossimImageHandlerRegistry::instance()->open(ossimFilename("Temp_DSM.tif"));
			handler_disp->setImageGeometry(master_geom);       
			ossimImageFileWriter* writer = ossimImageWriterFactoryRegistry::instance()->createWriter(ossimFilename(ap[5]));
			writer->connectMyInputTo(0, handler_disp);
			writer->execute();
            
			delete writer;
			delete test;				
		}
	}     
	catch (const ossimException& e)
	{
		ossimNotify(ossimNotifyLevel_WARN) << e.what() << endl;
		return 1;
	}
  
	return 0;
}
