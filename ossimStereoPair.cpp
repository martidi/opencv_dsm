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
#include <ossim/projection/ossimUtmpt.h>
#include "ossim/imaging/ossimImageHandlerRegistry.h"
#include "ossim/imaging/ossimImageHandler.h"
#include "ossimStereoPair.h"

#include <math.h>
#include <vector>
#include <numeric>

#define PI 3.14159265

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

/*void ossimStereoPair::computeConversionFactor( double longitude_max, double longitude_min, double latitude_max, double latitude_min, double MinimumHeight , double MaximumHeight)
{
        /*******************************************************/
        /************ BEGIN CONV FACTOR COMPUTATION ************/
        /*******************************************************/

        // variabile is intialized=True per vedere se le path sono riempite
        // sennò dai errore

      /*  cout << "Min height for conv. fact." << MinimumHeight << " Max height for conv. fact. " << MaximumHeight << endl;
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
        double mean_conversionF_J = mean_conv_factor_J.val[0]/(MaximumHeight -MinimumHeight);
        double stDev_conversionF_I = stDev_conv_factor_I.val[0];
        double mean_conversionF_I = mean_conv_factor_I.val[0]/(MaximumHeight -MinimumHeight );

        //double mean_conversionF;

        if (abs(mean_conv_factor_J.val[0]) > abs(mean_conv_factor_I.val[0]))
            conversion_factor = mean_conv_factor_J.val[0];
        else conversion_factor = mean_conv_factor_I.val[0];
        conversion_factor /= (MaximumHeight -MinimumHeight);

        cout << "J Conversion Factor from pixels to meters\t" << mean_conversionF_J << endl;
        cout << "Standard deviation J Conversion Factor\t" << stDev_conversionF_J << endl << endl;
        cout << "I Conversion Factor from pixels to meters\t" << mean_conversionF_I << endl;
        cout << "Standard deviation I Conversion Factor\t" << stDev_conversionF_I << endl << endl;
        cout << "Total Conversion Factor from pixels to meters\t" << conversion_factor << endl << endl;

        /*****************************************************/
        /************ END CONV FACTOR COMPUTATION ************/
        /*****************************************************/
//}

//void ossimEpipolarity::epipolarDirection(double MinimumHeight , double MaximumHeight)
void ossimStereoPair::epipolarDirection()
{
    //Per una data I e J (anzi, per un grigliato di I e J), uso gli RPC per scendere a due quote:h1 e h2
    //devo prendere l'immagine e dividerla in n=grid parti uguali
    //mi servono le dimensioni dell'immagine

    //devo capire come settare la minima e la massima altezza da investigare, vorrei un ciclo

    cout <<"EPIPOLAR DIRECTION COMPUTATION " << endl;
    ossimImageHandler* raw_master_handler = ossimImageHandlerRegistry::instance()->open(ossimFilename(raw_master));
    ossimImageHandler* raw_slave_handler = ossimImageHandlerRegistry::instance()->open(ossimFilename(raw_slave));
    ossimRefPtr<ossimImageGeometry> raw_master_geom = raw_master_handler->getImageGeometry();
    ossimRefPtr<ossimImageGeometry> raw_slave_geom = raw_slave_handler->getImageGeometry();

    int grid = 10;
    double MinimumHeight= 500.0;
    double MaximumHeight= 1000.0;
    deltaH = MaximumHeight - MinimumHeight;
    ossimIpt image_size = raw_master_geom->getImageSize();
    cout << "image size " <<image_size << endl;

    double deltaI = image_size.x /(grid +1);
    cout << "delta I " <<deltaI << endl;
    double deltaJ = image_size.y /(grid +1);
    cout << "delta J " <<deltaJ << endl;

    //Create and write the log file
    ofstream epi_direction;
    epi_direction.open("Epipolar_direction_1800.txt");
    std::vector<double> array_angle, array_convFact;

    for (int i=1 ; i<grid+1 ; i++) //LAT
    {
        for (int j=1 ; j<grid+1 ; j++) //LON
        {
        ossimDpt imagePoint_master(deltaI*i,deltaJ*j);
        ossimDpt imagePoint_slave(0.,0.);
        ossimGpt groundPoint_master(0.,0.,MaximumHeight);
        ossimGpt groundPoint_slave(0.,0.,MaximumHeight);
        ossimGpt groundPointDown(0.,0.,MinimumHeight);
        //cout << imagePoint_master << "" << imagePoint_slave << "" << groundPoint_master <<"" << groundPoint_slave << "" << groundPointDown << endl;
        raw_master_geom->localToWorld(imagePoint_master, MaximumHeight, groundPoint_master);  //con qst trasf ottengo groundPoint_master
        raw_master_geom->localToWorld(imagePoint_master,MinimumHeight,groundPointDown);
        //cout << imagePoint_master << "" << imagePoint_slave << "" << groundPoint_master <<"" << groundPoint_slave << "" << groundPointDown << endl;

        //una volta riempito il punto a terra più basso, vado sul piano immagine della slave
        raw_slave_geom->worldToLocal(groundPointDown, imagePoint_slave);
        //dal piano immagine della slave vado a terra alla quota più alta
        raw_slave_geom->localToWorld(imagePoint_slave, MaximumHeight,groundPoint_slave);

        //Geographic --> UTM conversion
        ossimUtmpt UTMgroundPoint_master(groundPoint_master);
        ossimUtmpt UTMgroundPoint_slave(groundPoint_slave);

        //la direzione di epi è data da groundPoint_master e groundPoint_slave
        /*cout << "Epipolar direction " << endl;
        cout << "Point 1 geog " << groundPoint_master << " Point 2 geog " << groundPoint_slave << endl;
        cout << "Point 1 UTM East " << UTMgroundPoint_master.easting() << " Point 1 UTM North " << UTMgroundPoint_master.northing() << endl;
        cout << "Point 2 UTM East " << UTMgroundPoint_slave.easting() << " Point 2 UTM North " << UTMgroundPoint_slave.northing() << endl;
*/
        double DE = UTMgroundPoint_slave.easting() - UTMgroundPoint_master.easting();
        double DN = UTMgroundPoint_slave.northing() - UTMgroundPoint_master.northing();

        // faccio un vettore di double in cui pusho i valori e poi ne faccio la media
        double rotation_angle = atan (DN/DE) * 180 / PI;
        array_angle.push_back(rotation_angle);
        mean_rotation_angle = std::accumulate( array_angle.begin(), array_angle.end(), 0.0)/array_angle.size();

        //cout << mean_rotation_angle << endl;
        // "fixed" for set decimals numbers
        epi_direction << fixed << setprecision(12) << UTMgroundPoint_master.easting() << " " << UTMgroundPoint_master.northing() << " " << UTMgroundPoint_slave.easting()  << " " << UTMgroundPoint_slave.northing() << endl;

        double conversion_factor = (sqrt((DE*DE) + (DN*DN)))/deltaH;
        array_convFact.push_back(conversion_factor);
        mean_conversion_factor = std::accumulate( array_convFact.begin(), array_convFact.end(), 0.0)/array_convFact.size();
        //cout << "Fattore di conversione " << mean_conversion_factor << endl;
        }
    }
    cout << "Angolo di rotazione " << mean_rotation_angle << endl;
    cout << "Fattore di conversione " << mean_conversion_factor << endl;
    epi_direction.close();
}

double ossimStereoPair::getMeanRotationAngle()
{
        return mean_rotation_angle;
}

double ossimStereoPair::getConversionFactor()
{
        return mean_conversion_factor;
}


    
    
    
