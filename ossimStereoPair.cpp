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

#include "ossim/imaging/ossimImageHandlerRegistry.h"
#include "ossim/imaging/ossimImageHandler.h"
#include "ossimStereoPair.h"

ossimStereoPair::ossimStereoPair()
{
	
}

void ossimStereoPair::setID(int IDM, int IDS)
{
        id_master = IDM;
        id_slave  = IDS;
}

int ossimStereoPair::get_id_master()
{
        return id_master;

}

int ossimStereoPair::get_id_slave()
{
        return id_slave;

}

void ossimStereoPair::setRawPath(ossimString raw_master_path, ossimString raw_slave_path)
{
	raw_master = raw_master_path;
	raw_slave = raw_slave_path;
}

void ossimStereoPair::setOrthoPath(ossimString ortho_master_path, ossimString ortho_slave_path)
{
        ortho_master = ortho_master_path;
        ortho_slave = ortho_slave_path;
}

ossimString ossimStereoPair::getRawMasterPath()
{
	return raw_master;
}

ossimString ossimStereoPair::getRawSlavePath()
{
	return raw_slave;
}

ossimString ossimStereoPair::getOrthoMasterPath()
{
        return ortho_master;
}

ossimString ossimStereoPair::getOrthoSlavePath()
{
        return ortho_slave;
}

void ossimStereoPair::computeConversionFactor( double longitude_max, double longitude_min, double latitude_max, double latitude_min, double MinimumHeight , double MaximumHeight)
{
        /*******************************************************/
        /************ BEGIN CONV FACTOR COMPUTATION ************/
        /*******************************************************/

        // varibaiel is intialized=True per vedere se le path sono riempite
        // sennò dai errore

        ossimImageHandler* raw_master_handler = ossimImageHandlerRegistry::instance()->open(ossimFilename(raw_master));
        ossimImageHandler* raw_slave_handler = ossimImageHandlerRegistry::instance()->open(ossimFilename(raw_slave));
        cout << "CONVERSION FACTOR COMPUTATION FOR IMAGES: " << endl << endl;
        cout << raw_master<< endl << endl;
        cout << raw_slave<< endl << endl;
        ossimRefPtr<ossimImageGeometry> raw_master_geom = raw_master_handler->getImageGeometry();
        ossimRefPtr<ossimImageGeometry> raw_slave_geom = raw_slave_handler->getImageGeometry();

        // CONVERSION FACTOR (from pixels to meters)

        // Conversion factor computed on the tile and not over all the image
        double Dlon = (longitude_max - longitude_min)/2.0;
        double Dlat = (latitude_max - latitude_min)/2.0;

        // Getting ready the log file

        cv::Mat conv_factor_J = cv::Mat::zeros(3,3, CV_64F);
        cv::Mat conv_factor_I = cv::Mat::zeros(3,3, CV_64F);

        for (int i=0 ; i<3 ; i++) //LAT
        {
            for (int j=0 ; j<3 ; j++) //LON
            {
            ossimGpt groundPoint(latitude_max-i*Dlat,longitude_min+j*Dlon,MinimumHeight); //MinHeight -50
            ossimGpt groundPointUp(latitude_max-i*Dlat,longitude_min+j*Dlon,MaximumHeight); //MaxHeight + 50

            ossimDpt imagePoint(0.,0.);
            ossimDpt imagePointUp(0.,0.);

            raw_master_geom->worldToLocal(groundPoint,imagePoint);        //con qst trasf ottengo imagePoint della nadir
            raw_master_geom->worldToLocal(groundPointUp,imagePointUp);

            //myfile << "MASTER IMAGE" << "\t" << "Ground point" << groundPoint << "\t" << "Image point" << imagePoint << "\t" << "Ground point up" << groundPointUp << "\t" << "Image point up" << imagePointUp << "\t";

            double DeltaI_master = imagePointUp.x - imagePoint.x;
            double DeltaJ_master = imagePointUp.y - imagePoint.y;

            raw_slave_geom->worldToLocal(groundPoint,imagePoint);
            raw_slave_geom->worldToLocal(groundPointUp,imagePointUp);

            //myfile << "SLAVE IMAGE" << "\t" << "Ground point" << groundPoint << "\t" << "Image point" << imagePoint << "\t" << "Ground point up" << groundPointUp << "\t" << "Image point up" << imagePointUp << "\t" << endl;

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
        double mean_conversionF_J = mean_conv_factor_J.val[0]/(MaximumHeight -MinimumHeight + 100);
        double stDev_conversionF_I = stDev_conv_factor_I.val[0];
        double mean_conversionF_I = mean_conv_factor_I.val[0]/(MaximumHeight -MinimumHeight + 100);

        //double mean_conversionF;

        if (abs(mean_conv_factor_J.val[0]) > abs(mean_conv_factor_I.val[0]))
            conversion_factor = mean_conv_factor_J.val[0];
        else conversion_factor = mean_conv_factor_I.val[0];
        conversion_factor /= (MaximumHeight -MinimumHeight + 100);

        cout << "J Conversion Factor from pixels to meters\t" << mean_conversionF_J << endl;
        cout << "Standard deviation J Conversion Factor\t" << stDev_conversionF_J << endl << endl;
        cout << "I Conversion Factor from pixels to meters\t" << mean_conversionF_I << endl;
        cout << "Standard deviation I Conversion Factor\t" << stDev_conversionF_I << endl << endl;
        cout << "Total Conversion Factor from pixels to meters\t" << conversion_factor << endl << endl;

        /*****************************************************/
        /************ END CONV FACTOR COMPUTATION ************/
        /*****************************************************/
}

float ossimStereoPair::getConversionFactor()
{
	return conversion_factor;
}
    
    
    
    
