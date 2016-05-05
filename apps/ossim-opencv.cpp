//----------------------------------------------------------------------------
//
// License:  See top level LICENSE.txt file.
//
// File: ossim-opencv.cpp
//
// Author:  Martina Di Rita
//
// Description: This plugIn is able to extract a geocoded Digital Surface Model
//				from a triplet.
//
//----------------------------------------------------------------------------

#include <ossim/base/ossimArgumentParser.h>
#include <ossim/base/ossimException.h>
#include <ossim/base/ossimRefPtr.h>
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
#include <ossim/imaging/ossimMemoryImageSource.h>
#include <ossim/imaging/ossimTiffWriter.h>

#include <ossim/init/ossimInit.h>

#include <ossim/util/ossimChipperUtil.h>

#include "openCVtestclass.h"

#include <iostream>
#include <fstream>
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
static const std::string PROJECTION_KW           = "projection";
static const std::string INPUT_NB_KW           = "input";

ossimImageHandler* raw_image_handler;

class StereoPair
 {
   ossimString master, slave;
   int id;

   public:
   void setID(int ID)
   {
       id = ID;
   }

   void setPath(ossimString img_master_path, ossimString img_slave_path)
   {
      master = img_master_path;
      slave = img_slave_path;
   }

   ossimString masterPath()
   {
       return master;
   }

   ossimString slavePath()
   {
       return slave;
   }
};



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

        // SISTEMARE, DEVE ESSERE <5, IL PIU GEN POSSIBILE (SOLO CON 2 IMMAGINI)
        if(ap.argc() < 5) //ap.argv[0] is the application name
        {
            ap.writeErrorMessages(ossimNotify(ossimNotifyLevel_NOTICE));
            std::string errMsg = "Few arguments...";
            cout << endl << "Usage: ossim-dsm-app <input_image_1> <input_image_2> ... <input_image_n> <output_results_directory> <output_dsm_name> [options] <n° steps for pyramidal>" << endl;
            cout << "Options:" << endl;
            cout << "--cut-bbox-ll <min_lat> <min_lon> <max_lat> <max_lon> \t Specify a bounding box with the minimum"   << endl;
            cout << "\t\t\t\t\t\t\tlatitude/longitude and max latitude/longitude" << endl;
            cout << "\t\t\t\t\t\t\tin decimal degrees." << endl;
            cout << "--meters <meters> \t\t\t\t\t Specify a size (in meters) for a resampling"   << endl<< endl;
            throw ossimException(errMsg);
        }

        // li definisco qui così ce l'ho a disposizione anche dopo
        ossimKeywordlist master_key;
        ossimKeywordlist slave_key;

        std::string tempString1,tempString2,tempString3,tempString4;
        ossimArgumentParser::ossimParameter stringParam1(tempString1);
        ossimArgumentParser::ossimParameter stringParam2(tempString2);
        ossimArgumentParser::ossimParameter stringParam3(tempString3);
        ossimArgumentParser::ossimParameter stringParam4(tempString4);

        double lat_min;
        double lon_min;
        double lat_max;
        double lon_max;
        double MinHeight;
        double MaxHeight;

        // per leggere un file da terminale
        fstream f_input;
        f_input.open(ap[1], ios::in);

        if (f_input.fail())
        {
            cout << "Missing input file" << endl;
        }

        ossimString s1;
        ossimString s2;
        int id;
        vector<StereoPair> StereoPairList;

        while (f_input >> id >> s1 >> s2) // fino a che leggi un int seguito da due ossimString
        {
            cout <<id <<endl;
            StereoPair ii;

            ii.setPath(s1,s2);
            ii.setID(id);
            cout << ii.masterPath() << endl;
            cout << ii.slavePath() << endl;
            StereoPairList.push_back(ii);
        }

        f_input.close();
        // Leggo quante immagini ho in input
        cout << "numero coppie " << StereoPairList.size() << endl;

        // Per ottenerele path delle immagini
        /*cout << "dir_0_master " << StereoPairList[0].masterPath() << endl;
        cout << "dir_0_slave " << StereoPairList[0].slavePath() << endl;
        cout << "dir_1_master " << StereoPairList[1].masterPath() << endl;
        cout << "dir_1_slave " << StereoPairList[1].slavePath() << endl;
        cout << "dir_2_master " << StereoPairList[2].masterPath() << endl;
        cout << "dir_2_slave " << StereoPairList[2].slavePath() << endl << endl;*/


        /**********************************************/
        /************ BEGIN OF ARG PARSING ************/
        /**********************************************/

        /************ UTM projection ************/
        if(ap.read("--projection", stringParam1) )
        {
            master_key.addPair(PROJECTION_KW, tempString1 );
            slave_key.addPair(PROJECTION_KW, tempString1 );

            cout << "Output DSM is in UTM projection" << endl << endl;
        }

        /************ Tiling ************/
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

        /************ Sampling ************/
        if(ap.read("--meters", stringParam1) )
        {
            master_key.addPair(METERS_KW, tempString1 );
            slave_key.addPair(METERS_KW, tempString1 );
        }
        double finalRes = atof(tempString1.c_str());
        cout << "Orthoimages resolution = " << tempString1 <<" meters"<< endl << endl;

        /************ Default keyword for ortho ************/
        master_key.addPair(OP_KW, "ortho");
        slave_key.addPair(OP_KW, "ortho");

        /************ Resampler ************/
        master_key.addPair(RESAMPLER_FILTER_KW, "box");
        slave_key.addPair(RESAMPLER_FILTER_KW, "box");
        cout << endl << "Resampling filter is box" << endl << endl;

        /************ Number of iteration ************/
        int nsteps;
        ossimArgumentParser::ossimParameter iteration(nsteps);
        if(ap.read("--nsteps", iteration))
        {
            //else nsteps = 1;
        }
        cout << "Total steps number for pyramidal\t " << nsteps << endl;

        /**********************************************/
        /************ END OF ARG PARSING ************/
        /**********************************************/

        double iter = (nsteps-1);

        for(int b = iter ; b >= 0  ; b--)
        {
            iter;

            std::ostringstream strs;
            strs << iter;
            std::string nLev = strs.str();

            // Elevation manager instance
            ossimElevManager* elev = ossimElevManager::instance();
            cout << "elevation database \t" << elev->getNumberOfElevationDatabases() << endl;

            master_key.add( ossimKeywordNames::OUTPUT_FILE_KW, ossimFilename(ap[2]) + ossimString("ortho_images/") + ossimFilename(ap[3]) + ossimString("_level") + nLev + ossimString("_orthoMaster.TIF"));
            slave_key.add( ossimKeywordNames::OUTPUT_FILE_KW, ossimFilename(ap[2]) + ossimString("ortho_images/") + ossimFilename(ap[3]) + ossimString("_level") + nLev + ossimString("_orthoSlave.TIF"));

            double orthoRes = finalRes*pow (2, iter);
            cout << finalRes << " " << "m" << "\t final DSM resolution" << endl;
            cout << orthoRes << " " << "m" << "\t resolution of this level" << endl;
            cout << iter << "\t n° iterations left" << endl << endl;

            std::ostringstream strsRes;
            strsRes << orthoRes;
            std::string ResParam = strsRes.str();

            master_key.addPair(METERS_KW, ResParam);
            slave_key.addPair( METERS_KW, ResParam);

            // Lavoro su due immagini alla volta
            for (int k=0; k<=StereoPairList.size(); k++)
            {
                master_key.addPair("image1.file", StereoPairList[k].masterPath());
                slave_key.addPair("image1.file", StereoPairList[k].slavePath());

                cout << endl << "IMAGE MASTER DIRECTORY:" << " " << StereoPairList[k].masterPath() << endl;
                cout << "IMAGE SLAVE DIRECTORY:"  << " " << StereoPairList[k].slavePath() << endl << endl;

                ap.reportRemainingOptionsAsUnrecognized();
                if (ap.errors())
                {
                    ap.writeErrorMessages(ossimNotify(ossimNotifyLevel_NOTICE));
                    std::string errMsg = "Unknown option...";
                    throw ossimException(errMsg);
                }

                //END PARSER****************************

                ossimImageHandler* raw_master_handler = ossimImageHandlerRegistry::instance()->open(ossimFilename(StereoPairList[k].masterPath()));
                ossimImageHandler* raw_slave_handler = ossimImageHandlerRegistry::instance()->open(ossimFilename(StereoPairList[k].slavePath()));

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
                    ossimGpt groundPoint(lat_max-i*Dlat,lon_min+j*Dlon,MinHeight); //MinHeight -50
                    ossimGpt groundPointUp(lat_max-i*Dlat,lon_min+j*Dlon,MaxHeight); //MaxHeight + 50

                    ossimDpt imagePoint(0.,0.);
                    ossimDpt imagePointUp(0.,0.);

                    raw_master_geom->worldToLocal(groundPoint,imagePoint);        //con qst trasf ottengo imagePoint della nadir
                    raw_master_geom->worldToLocal(groundPointUp,imagePointUp);

                    myfile << "MASTER IMAGE" << "\t" << "Ground point" << groundPoint << "\t" << "Image point" << imagePoint << "\t" << "Ground point up" << groundPointUp << "\t" << "Image point up" << imagePointUp << "\t";

                    double DeltaI_master = imagePointUp.x - imagePoint.x;
                    double DeltaJ_master = imagePointUp.y - imagePoint.y;

                    raw_slave_geom->worldToLocal(groundPoint,imagePoint);
                    raw_slave_geom->worldToLocal(groundPointUp,imagePointUp);

                    myfile << "SLAVE IMAGE" << "\t" << "Ground point" << groundPoint << "\t" << "Image point" << imagePoint << "\t" << "Ground point up" << groundPointUp << "\t" << "Image point up" << imagePointUp << "\t" << endl;

                    double DeltaI_slave = imagePointUp.x - imagePoint.x;
                    double DeltaJ_slave = imagePointUp.y - imagePoint.y;

                    conv_factor_J.at<double>(i,j) = DeltaJ_slave - DeltaJ_master; // conv_factor for ACROSS-track imgs
                    conv_factor_I.at<double>(i,j) = DeltaI_slave - DeltaI_master; // conv_factor for ALONG-track imgs
                    }
                }

                cout << conv_factor_J << endl;
                cout << conv_factor_I << endl;

                cv::Scalar mean_conv_factor_J, stDev_conv_factor_J;
                cv::meanStdDev(conv_factor_J, mean_conv_factor_J, stDev_conv_factor_J);

                cv::Scalar mean_conv_factor_I, stDev_conv_factor_I;
                cv::meanStdDev(conv_factor_I, mean_conv_factor_I, stDev_conv_factor_I);

                double stDev_conversionF_J = stDev_conv_factor_J.val[0];
                double mean_conversionF_J = mean_conv_factor_J.val[0]/(MaxHeight -MinHeight + 100);
                double stDev_conversionF_I = stDev_conv_factor_I.val[0];
                double mean_conversionF_I = mean_conv_factor_I.val[0]/(MaxHeight -MinHeight + 100);

                double mean_conversionF;
                //mean_conversionF.resize(2);

                if (abs(mean_conv_factor_J.val[0]) > abs(mean_conv_factor_I.val[0]))
                    mean_conversionF = mean_conv_factor_J.val[0];
                else mean_conversionF = mean_conv_factor_I.val[0];
                mean_conversionF /= (MaxHeight -MinHeight + 100);

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

                cout << "Start master orthorectification level " << nLev << endl;
                ortho(master_key);

                cout << "Start slave orthorectification level " << nLev << endl;
                ortho(slave_key);

                // ImageHandlers & ImageGeometry instance
                ossimImageHandler* master_handler = ossimImageHandlerRegistry::instance()->open(ossimFilename(ap[2]) + ossimString("ortho_images/") + ossimFilename(ap[3]) + ossimString("_level") + nLev + ossimString("_orthoMaster.TIF"));
                ossimImageHandler* slave_handler = ossimImageHandlerRegistry::instance()->open(ossimFilename(ap[2]) + ossimString("ortho_images/") + ossimFilename(ap[3]) + ossimString("_level") + nLev + ossimString("_orthoSlave.TIF"));

                //if(master_handler && slave_handler && raw_master_handler && raw_slave_handler) // enter if exist both master and slave
                //{
                 // Load ortho images
                 ossimIrect bounds_master = master_handler->getBoundingRect(0);
                 ossimIrect bounds_slave = slave_handler->getBoundingRect(0);

                 ossimRefPtr<ossimImageData> img_master = master_handler->getTile(bounds_master, 0);
                 ossimRefPtr<ossimImageData> img_slave = slave_handler->getTile(bounds_slave, 0);

                 // TPs and disparity map generation
                 openCVtestclass *stereoCV = new openCVtestclass(img_master, img_slave) ;
                 stereoCV->execute(mean_conversionF/1.0);
                 //}

                remove(ossimFilename(ossimFilename(ap[2]) + ossimString("temp_elevation/") + ossimFilename(ap[3])+ossimString(".TIF")));

                cout << "ciclo" << k << endl;


                // From Disparity to DSM
                ossimImageGeometry* master_geom = master_handler->getImageGeometry().get();
                master_handler->saveImageGeometry();

                // CREO UN'UNICA MAPPA DI DISPARITA' MEDIA E GENERO IL DSM
                ossimRefPtr<ossimImageData> finalDSM = stereoCV->computeDSM(elev, master_geom);

                ossimFilename pathDSM;
                if (b == 0)
                   pathDSM = ossimFilename(ap[2]) + ossimString("DSM/") + ossimFilename(ap[3]) + ossimString(".TIF");
                else
                    pathDSM = ossimFilename(ap[42]) + ossimString("temp_elevation/") + ossimFilename(ap[3])+ossimString(".TIF");

                // Create output image chain:
                ossimRefPtr<ossimMemoryImageSource> memSource = new ossimMemoryImageSource;
                memSource->setImage(finalDSM);
                memSource->setImageGeometry(master_geom);
                cout << "size" << master_geom->getImageSize() << endl;
                memSource->saveImageGeometry();

                ossimImageFileWriter* writer = ossimImageWriterFactoryRegistry::instance()->createWriter(pathDSM);
                writer->connectMyInputTo(0, memSource.get());
                writer->execute();

                writer->close();
                writer = 0;
                memSource = 0;
                delete stereoCV;

                // INSERITO TUTTO IN OPENCVTESTCLASS//
                /*
                // Geocoded DSM generation
                ossimImageHandler *handler_disp = ossimImageHandlerRegistry::instance()->open(ossimFilename("DSM_float.tif"));
                handler_disp->setImageGeometry(nadir_geom);
                handler_disp->saveImageGeometry();

                // Add a listener to get percent complete
                ossimStdOutProgress prog(0, true);
                writer->addListener(&prog);
                writer->execute();
                writer->removeListener(&prog);
                writer = 0;*/

                iter --;
                elev = 0;
            }
        }
        cout << endl << "D.A.T.E. Plug-in has successfully generated a Digital Surface Model from your triplet!\n" << endl;
	}     
	catch (const ossimException& e)
	{
		ossimNotify(ossimNotifyLevel_WARN) << e.what() << endl;
		return 1;
	}
  
	return 0;
}
