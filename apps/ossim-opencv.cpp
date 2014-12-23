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

#include <ossim/elevation/ossimElevManager.h>

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
		
     						     								
			cout << "Tile extent:" << "\tLat_min = "<< tempString1 << endl   
								<<"\t\tLon_min = " << tempString2 << endl
								<<"\t\tLat_max = " << tempString3 << endl
								<<"\t\tLon_max = " << tempString4 << endl;
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
	
/*	
   string tempString;
   ossimArgumentParser::ossimParameter stringParam(tempString);
   ossimArgumentParser argumentParser(&argc, argv);
   ossimInit::instance()->addOptions(argumentParser);
   ossimInit::instance()->initialize(argumentParser);

   if(traceDebug())
   {
      ossimNotify(ossimNotifyLevel_DEBUG) << "entered main" << std::endl;
   }
   
   argumentParser.getApplicationUsage()->setApplicationName(argumentParser.getApplicationName());
   argumentParser.getApplicationUsage()->setDescription(argumentParser.getApplicationName()+" takes a spec file as input and produces a product");
   argumentParser.getApplicationUsage()->setCommandLineUsage(argumentParser.getApplicationName()+" [options] <spec_file>");
   argumentParser.getApplicationUsage()->addCommandLineOption("-t or --thumbnail", "thumbnail resolution");
   argumentParser.getApplicationUsage()->addCommandLineOption("-h or --help","Display this information");
 

   if(argumentParser.read("-h") ||
      argumentParser.read("--help")||
      argumentParser.argc() <2)
   {
      argumentParser.getApplicationUsage()->write(std::cout);
      exit(0);
   }

   ossimRefPtr<ossimIgen> igen = new ossimIgen;
   double start=0, stop=0;
   
   ossimMpi::instance()->initialize(&argc, &argv);
   start = ossimMpi::instance()->getTime();

   ossimKeywordlist kwl;
   kwl.setExpandEnvVarsFlag(true);
   
   while(argumentParser.read("-t", stringParam)   ||
         argumentParser.read("--thumbnail", stringParam));
   
   if(ossimMpi::instance()->getRank() > 0)
   {
      // since this is not the master process
      // then it will set the keyword list form the master
      // so set this to empty
      //
      igen->initialize(ossimKeywordlist());
   }
   else if(argumentParser.argc() > 1)
   {
      if(kwl.addFile(argumentParser.argv()[1]))
      {
         if(tempString != "")
         {
            kwl.add("igen.thumbnail",
                    "true",
                    true);
            kwl.add("igen.thumbnail_res",
                    tempString.c_str(),
                    true);
         }
         else
         {
            kwl.add("igen.thumbnail",
                    "false",
                    true);
         }
         kwl.add("igen.thumbnail_res",
                 tempString.c_str(),
                 true);

         igen->initialize(kwl);
      }
   }

*/		
		
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

			// Conversion factor (from pixels to meters) computation
			ossimRefPtr<ossimImageGeometry> raw_master_geom = raw_master_handler->getImageGeometry();    
			ossimRefPtr<ossimImageGeometry> raw_slave_geom = raw_slave_handler->getImageGeometry(); 
			
			ossimGpt ul,ur,lr,ll;
			raw_master_geom->getCornerGpts(ul, ur, lr, ll);
					
			double Dlon = (ur.lon - ul.lon)/2.0;
			double Dlat = (ul.lat - ll.lat)/2.0;
        
			cv::Mat conv_factor = cv::Mat::zeros(3,3, CV_64F);
			
			for (int i=0 ; i<3 ; i++) //LAT
			{
				for (int j=0 ; j<3 ; j++) //LON
				{
					ossimGpt punto_terra(ur.lat-i*Dlat,ul.lon+j*Dlon,200.00);
					ossimGpt punto_terra_up(ur.lat-i*Dlat,ul.lon+j*Dlon,2200.00);	
					ossimDpt punto_img(0.,0.);
					ossimDpt punto_img_up(0.,0.);
				
					raw_master_geom->worldToLocal(punto_terra,punto_img);              
					raw_master_geom->worldToLocal(punto_terra_up,punto_img_up);   
					
					double DeltaI_Master = punto_img_up.x - punto_img.x;
					double DeltaJ_Master = punto_img_up.y - punto_img.y;
        
					raw_slave_geom->worldToLocal(punto_terra,punto_img);       
					raw_slave_geom->worldToLocal(punto_terra_up,punto_img_up);    
					
					double DeltaI_Slave = punto_img_up.x - punto_img.x;
					double DeltaJ_Slave = punto_img_up.y - punto_img.y;
					
					conv_factor.at<double>(i,j) = DeltaJ_Slave - DeltaJ_Master;
				}			
			}
	
			cv::Scalar mean_conv_factor, stDev_conv_factor;
			cv::meanStdDev(conv_factor, mean_conv_factor, stDev_conv_factor);

			double stDev_conversionF = stDev_conv_factor.val[0];
			double mean_conversionF = mean_conv_factor.val[0]/(2200.00-200.00);	        
			
			cout << "Conversion Factor from pixels to meters\t" << mean_conversionF <<endl;
			cout << "Standard deviation Conversion Factor\t" << stDev_conversionF <<endl;
			
			
			char * prova = ap[4];			
			string log(prova);	
			log.erase(log.end()-4, log.end()-0 );
			log = log + "_logfile.txt";			
			
			// Creating and writing the log file
			ofstream myfile;
			myfile.open (log.c_str());
			myfile << "Master orthorectification parameters" <<endl;
			myfile << master_key << endl;
			myfile << "Slave orthorectification parameters" <<endl;
			myfile << slave_key << endl;
			myfile <<"Conversion Factor from pixels to meters\t" << mean_conversionF <<endl;
			myfile <<"Standard deviation Conversion Factor\t" << stDev_conversionF <<endl; 
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
