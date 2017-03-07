//----------------------------------------------------------------------------
//
// License:  See top level LICENSE.txt file.
//
// File: ossimEpipolarity.cpp
//
// Author:  Martina Di Rita
//
// Description: Class providing OpenCV functions for DSM extraction
//
//----------------------------------------------------------------------------

/*#include <ossim/elevation/ossimElevManager.h>
#include <ossim/imaging/ossimImageSource.h>
#include <ossim/imaging/ossimTiffWriter.h>
#include <ossim/imaging/ossimImageDataFactory.h>
#include <ossim/imaging/ossimMemoryImageSource.h>
#include <ossim/point_cloud/ossimGenericPointCloudHandler.h>
#include <ossim/point_cloud/ossimPointCloudImageHandler.h>
#include <ossim/base/ossimStringProperty.h>*/

#include <ossim/projection/ossimUtmpt.h>
#include "ossim/imaging/ossimImageHandlerRegistry.h"
#include "ossimEpipolarity.h"
//#include "ossimOpenCvTPgenerator.h"
//#include "ossimOpenCvDisparityMapGenerator.h"
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv/cv.h"
#include <opencv2/highgui/highgui.hpp>
// Note: These are purposely commented out to indicate non-use.
// #include <opencv2/nonfree/nonfree.hpp>
// #include <opencv2/nonfree/features2d.hpp>
// Note: These are purposely commented out to indicate non-use.
#include <iostream>
#include <math.h>
#include <vector>
#include <numeric>

#define PI 3.14159265

ossimEpipolarity::ossimEpipolarity()
{
	
}

/*ossimEpipolarity::ossimEpipolarity(ossimRefPtr<ossimImageData> master, ossimRefPtr<ossimImageData> slave)
{
    // Create the OpenCV images
	master_mat.create(cv::Size(master->getWidth(), master->getHeight()), CV_16UC1);
	slave_mat.create(cv::Size(slave->getWidth(), slave->getHeight()), CV_16UC1);
	
	memcpy(master_mat.ptr(), (void*) master->getUshortBuf(), 2*master->getWidth()*master->getHeight());
	memcpy(slave_mat.ptr(), (void*) slave->getUshortBuf(), 2*slave->getWidth()*slave->getHeight());

	cout << "OSSIM->OpenCV image conversion done" << endl;
	
    // Rotation for along-track OPTICAL images
    //********* To be commented for SAR images *********
    //cv::transpose(master_mat, master_mat);
    //cv::flip(master_mat, master_mat, 1);
	
    //cv::transpose(slave_mat, slave_mat);
    //cv::flip(slave_mat, slave_mat, 1);
    //********* To be commented for SAR images *********
}*/




bool ossimEpipolarity::epipolarDirection(ossimString masterName, ossimString slaveName)
{
    //Per una data I e J (anzi, per un grigliato di I e J), uso gli RPC per scendere a due quote:h1 e h2
    //devo prendere l'immagine e dividerla in n=grid parti uguali
    //mi servono le dimensioni dell'immagine

    //devo capire come settare la minima e la massima altezza da investigare, vorrei un ciclo

    cout <<"EPIPOLAR DIRECTION COMPUTATION " << endl;
    ossimImageHandler* raw_master_handler = ossimImageHandlerRegistry::instance()->open(ossimFilename(masterName));
    ossimImageHandler* raw_slave_handler = ossimImageHandlerRegistry::instance()->open(ossimFilename(slaveName));
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
    epi_direction.close();

    return true;
}

double ossimEpipolarity::getMeanRotationAngle()
{
        return mean_rotation_angle;
}

double ossimEpipolarity::getConversionFactor()
{
        return mean_conversion_factor;
}

/*ossimRefPtr<ossimImageData> openCVtestclass::computeDSM( ossimElevManager* elev, ossimImageGeometry* master_geom)
{
    cout<< " " << endl << "MERGING DISPARITY MAPS \t wait few minutes..." << endl << endl;

    cv::Mat fusedDisp = cv::Mat::zeros(fusedDisp_array[0].rows, fusedDisp_array[0].cols, CV_64F);

    // voglio fare la media per ogni x e y delle varie mappe di disparità
    for (int i=0; i< fusedDisp_array[0].rows; i++) // for every row
    {
        for(int j=0; j< fusedDisp_array[0].cols; j++) // for every column
        {
            int num=0.0;

            //if(fabs(error_disp.at<double>(i,j)) < 5)
                //{
                for (unsigned int k=0 ; k < fusedDisp_array.size() ; k++ ) //for every disp map of the vector; dato che la size mio ridà il num di Mat presenti, giusto?
                {   // sommo ad ogni ciclo il valore corrispodente per tutte le i e le j, contando i cicli con num
                    fusedDisp.at<double>(i,j) += fusedDisp_array[k].at<double>(i,j);// /mean_conversionF[k]; // "metric" disparity
                    num++;
                }
                //}
                fusedDisp.at<double>(i,j)  = fusedDisp.at<double>(i,j) /num; //faccio la media

            // Sum between "metric" disparity and coarse dsm
            ossimDpt image_pt(j,i);
            ossimGpt world_pt;
            master_geom->localToWorld(image_pt, world_pt);
            ossim_float64 hgtAboveMSL =  elev->getHeightAboveMSL(world_pt);
            //ossim_float64 hgtAboveMSL =  elev->getHeightAboveEllipsoid(world_pt); //Augusta site
            fusedDisp.at<double>(i,j) += hgtAboveMSL;
            //hgtAboveMSL += fusedDisp.at<double>(i,j);
            //world_pt.height(hgtAboveMSL);
            //image_points.push_back(world_pt);
        }
    }

    // Set the destination image size:
    ossimIpt image_size (fusedDisp.cols , fusedDisp.rows);
    ossimRefPtr<ossimImageData> outImage = ossimImageDataFactory::instance()->create(0, OSSIM_FLOAT32, 1, image_size.x, image_size.y);

    if(outImage.valid())
        outImage->initialize();
    // else
    //  return -1;

    for (int i=0; i< fusedDisp.cols; i++) // for every column
    {
        for(int j=0; j< fusedDisp.rows; j++) // for every row
        {
            outImage->setValue(i,j,fusedDisp.at<double>(j,i));
        }
    }
    return outImage;
}*/




