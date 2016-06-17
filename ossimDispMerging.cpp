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

#include "ossimOpenCvTPgenerator.h"
#include "ossimDispMerging.h"
#include "ossimOpenCvDisparityMapGenerator.h"
#include "ossimImagePreprocess.h"

#include <ossim/imaging/ossimMemoryImageSource.h>
#include "ossim/imaging/ossimImageHandlerRegistry.h"
#include "ossim/imaging/ossimImageHandler.h"
#include "ossim/imaging/ossimImageFileWriter.h"
#include "ossim/imaging/ossimImageWriterFactoryRegistry.h"

#include <opencv2/highgui/highgui.hpp>

ossimDispMerging::ossimDispMerging()
{
	
}


bool ossimDispMerging::execute(vector<ossimStereoPair> StereoPairList)
{
    /*cout << endl << "ortho master path "<<StereoPairList[0].getOrthoMasterPath() << endl << endl;
    cout << "ortho slave path " <<StereoPairList[0].getOrthoSlavePath() << endl << endl;
    cout << "conv_fact " << StereoPairList[0].getConversionFactor() << endl << endl;*/

    int pairsNumb = StereoPairList.size();

    // Faccio ciclo lavorando su una coppia alla volta
    for (int i = 0; i < pairsNumb ; i++)
        {
        cout << endl << endl << "PAIR PROCESSED: " << endl ;
        cout << StereoPairList[i].get_id_master() << "\t" << StereoPairList[i].get_id_slave() << endl ;

        // ImageHandlers & ImageGeometry instance
        master_handler = ossimImageHandlerRegistry::instance()->open(StereoPairList[i].getOrthoMasterPath());
        slave_handler = ossimImageHandlerRegistry::instance()->open(StereoPairList[i].getOrthoSlavePath());

        //if(master_handler && slave_handler && raw_master_handler && raw_slave_handler) // enter if exist both master and slave
        //{

        // Conversion from ossim image to opencv matrix
        imgConversionToMat(); //apre le img ortho e diventano opencv mat

        imgPreProcessing(); // wallis filter

        imgGetHisto(); // histogram computation

        imgConversionTo8bit();      // Conversion from 16 bit to 8 bit

        // TPs generation
        ossimOpenCvTPgenerator *stereoTP = new ossimOpenCvTPgenerator(master_mat_8U, slave_mat_8U) ;
        stereoTP->execute();

        // Disparity map generation
        ossimOpenCvDisparityMapGenerator* dense_matcher = new ossimOpenCvDisparityMapGenerator();
        dense_matcher->execute(master_mat_8U, stereoTP->getWarpedImage(), StereoPairList[i]); // dopo questo execute ho disp metrica

        // Nel vettore globale di cv::Mat immagazzino tutte le mappe di disparità che genero ad ogni ciclo
        disp_array.push_back(dense_matcher->getDisp());
        null_disp_threshold = (dense_matcher->minimumDisp)+0.5;
        }

    cout << endl << "Disparity maps number " << disp_array.size() << endl;

    // DISPARITY MAPS FUSION

    merged_disp = cv::Mat::zeros(disp_array[0].rows, disp_array[0].cols, CV_64F);

    cout<< " " << endl << "DISPARITY MAPS FUSION \t wait few minutes..." << endl;

    //cout << "n° rows\t" << merged_disp.rows << endl;
    //cout << "n° columns\t" << merged_disp.cols << endl;

    for (int i=0; i< disp_array[0].rows; i++) // for every row
    {
        for(int j=0; j< disp_array[0].cols; j++) // for every column
        {
            int num=0.0;

            for (unsigned int k = 0; k < disp_array.size(); k++)  // for every disparity map
            {
                //for (int z = 0; z < k; z++)
                //{
                    //if(fabs(disp_array[z].at<double>(i,j) - disp_array[z+1].at<double>(i,j)) < 5)
                    //{
                        if(disp_array[k].at<double>(i,j) > null_disp_threshold) // sto togliendo i valori minori della threshold
                        {
                            merged_disp.at<double>(i,j) += disp_array[k].at<double>(i,j);
                            num++;
                        }
                    //}

                    else
                    {
                        merged_disp.at<double>(i,j) = disp_array[1].at<double>(i,j);
                    }

                //}
            merged_disp.at<double>(i,j)  = merged_disp.at<double>(i,j) /num;
            }
        }
    }

     //remove(ossimFilename(ossimFilename(ap[2]) + ossimString("temp_elevation/") + ossimFilename(ap[3])+ossimString(".TIF")));
     //cout << "ciclo" << k << endl;

    return true;
}


bool ossimDispMerging::computeDsm(vector<ossimStereoPair> StereoPairList, ossimElevManager *elev, int b, ossimArgumentParser ap)
{
    remove(ossimFilename(ossimFilename(ap[2]) + ossimString("temp_elevation/") + ossimFilename(ap[3])+ossimString(".TIF")));

    // Qui voglio sommare alla mappa di disparità fusa e metrica il dsm coarse
    // poi faccio il geocoding
    // poi esco da ciclo e rinizio a diversa risoluzione

    // From Disparity to DSM
    ossimImageGeometry* master_geom = master_handler->getImageGeometry().get();
    master_handler->saveImageGeometry();

    cout<< " " << endl << "DSM GENERATION \t wait few minutes..." << endl;
    cout << "null_disp_threshold"<< null_disp_threshold<< endl;

    for(int i=0; i< merged_disp.rows; i++)
    {
        for(int j=0; j< merged_disp.cols; j++)
        {
            ossimDpt image_pt(j,i);
            ossimGpt world_pt;

            master_geom->localToWorld(image_pt, world_pt);

            ossim_float64 hgtAboveMSL = elev->getHeightAboveMSL(world_pt);
            //ossim_float64 hgtAboveMSL =  elev->getHeightAboveEllipsoid(world_pt); //Augusta site

            if(merged_disp.at<double>(i,j) >= null_disp_threshold/abs(StereoPairList[i].getConversionFactor()))
            {
                merged_disp.at<double>(i,j) += hgtAboveMSL;


                //hgtAboveMSL += merged_disp.at<double>(i,j);

                //world_pt.height(hgtAboveMSL);

                // image_points.push_back(world_pt);
                // cout <<"punti"<<image_points[i]<<endl;
            }
            //To fill holes with DSM coarse
            else
            {
                merged_disp.at<double>(i,j) = hgtAboveMSL;
            }
        }
    }

    // Set the destination image size:
    ossimIpt image_size (merged_disp.cols , merged_disp.rows);
    finalDSM = ossimImageDataFactory::instance()->create(0, OSSIM_FLOAT32, 1, image_size.x, image_size.y);

    if(finalDSM.valid())
       finalDSM->initialize();
    // else
    //  return -1;

    for (int i=0; i< merged_disp.cols; i++) // for every column
    {
        for(int j=0; j< merged_disp.rows; j++) // for every row
        {
            finalDSM->setValue(i,j,merged_disp.at<double>(j,i));
        }
    }

    ossimFilename pathDSM;
    if (b == 0)
       pathDSM = ossimFilename(ap[2]) + ossimString("DSM/") + ossimFilename(ap[3]) + ossimString(".TIF");
    else
        pathDSM = ossimFilename(ap[2]) + ossimString("temp_elevation/") + ossimFilename(ap[3])+ ossimString(".TIF");

    cout << "path dsm " << pathDSM << endl;

    // Create output image chain:
    ossimRefPtr<ossimMemoryImageSource> memSource = new ossimMemoryImageSource;
    memSource->setImage(finalDSM);
    memSource->setImageGeometry(master_geom);
    cout << "dsm size " << master_geom->getImageSize() << endl;
    memSource->saveImageGeometry();

    ossimImageFileWriter* writer = ossimImageWriterFactoryRegistry::instance()->createWriter(pathDSM);
    writer->connectMyInputTo(0, memSource.get());
    writer->execute();

    writer->close();
    writer = 0;
    memSource = 0;

    return true;
}


ossimRefPtr<ossimImageData> ossimDispMerging::getDsm()
{
    return finalDSM;
}


bool ossimDispMerging::imgConversionToMat()
{
    // Load ortho images
    ossimIrect bounds_master = master_handler->getBoundingRect(0);
    ossimIrect bounds_slave = slave_handler->getBoundingRect(0);

    ossimRefPtr<ossimImageData> img_master = master_handler->getTile(bounds_master, 0);
    ossimRefPtr<ossimImageData> img_slave = slave_handler->getTile(bounds_slave, 0);

    // Create the OpenCV images
    master_mat.create(cv::Size(img_master->getWidth(), img_master->getHeight()), CV_16UC1);
    slave_mat.create(cv::Size(img_slave->getWidth(), img_slave->getHeight()), CV_16UC1);

    memcpy(master_mat.ptr(), (void*) img_master->getUshortBuf(), 2*img_master->getWidth()*img_master->getHeight());
    memcpy(slave_mat.ptr(), (void*) img_slave->getUshortBuf(), 2*img_slave->getWidth()*img_slave->getHeight());

    cout << endl << "OSSIM->OpenCV image conversion done" << endl;

    // Rotation for along-track OPTICAL images
    //********* To be commented for SAR images *********
    //cv::transpose(master_mat, master_mat);
    //cv::flip(master_mat, master_mat, 1);

    //cv::transpose(slave_mat, slave_mat);
    //cv::flip(slave_mat, slave_mat, 1);
    //********* To be commented for SAR images *********

    return true;

}


bool ossimDispMerging::imgPreProcessing()
{
    // ****************************
    // Activate for Wallis filter
    // ****************************
    /*ossimImagePreprocess *preprocess = new ossimImagePreprocess();
    master_mat = preprocess->wallis(master_mat);
    slave_mat = preprocess->wallis(slave_mat);*/

    return true;
}



bool ossimDispMerging::imgGetHisto()
{
    cout << "Histogram computation " << endl;

    /// Establish the number of bins
    int histSize = 256; //forse anche meno

    /// Set the ranges ( for B,G,R) )
    float range[] = { 0, 256 } ;
    const float* histRange = { range };
    ///We want our bins to have the same size (uniform) and to clear the histograms in the beginning, so:
    bool uniform = true; bool accumulate = false;
    /// we create the Mat objects to save our histogram
    cv::Mat hist;

    /// Compute the histograms:
    cv::calcHist( &master_mat, 1, 0, cv::Mat(), hist, 1, &histSize, &histRange, uniform, accumulate );

    /// Create an image to display the histograms
    ///Draw the histograms
    int hist_w = 512; int hist_h = 400;
    int bin_w = cvRound( (double) hist_w/histSize );

    cv::Mat histImage( hist_h, hist_w, CV_8UC3, cv::Scalar( 0,0,0) );

    /// Normalize the result to [ 0, histImage.rows ]
    normalize(hist, hist, 0, histImage.rows, cv::NORM_MINMAX, -1, cv::Mat() );

    /// Draw
    for( int i = 1; i < histSize; i++ )
    {
        line( histImage, cv::Point( bin_w*(i-1), hist_h - cvRound(hist.at<float>(i-1)) ) ,
                         cv::Point( bin_w*(i), hist_h - cvRound(hist.at<float>(i)) ),
                         cv::Scalar( 255, 0, 0), 2, 8, 0  );
    }

    /*int hbins = 30, sbins = 32;
    int histSize[] = {hbins, sbins};
    // hue varies from 0 to 179, see cvtColor
    float hranges[] = { 0, 180 };
    // saturation varies from 0 (black-gray-white) to
    // 255 (pure spectrum color)
    float sranges[] = { 0, 256 };
    const float* ranges[] = { hranges, sranges };
    int channels[] = {0,1};
    cv::MatND hist;
    cout << "a" << endl;
    cv::calcHist(&master_mat, 1, channels, cv::Mat(), hist, 2, histSize, ranges, true, false);

    double maxVal=0;
    cv::minMaxLoc(hist, 0, &maxVal, 0, 0);
cout << "a" << endl;
        int scale = 10;
        cv::Mat histImg = cv::Mat::zeros(sbins*scale, hbins*10, CV_8UC3);

        for( int h = 0; h < hbins; h++ )
            for( int s = 0; s < sbins; s++ )
            {
                float binVal = hist.at<float>(h, s);
                int intensity = cvRound(binVal*255/maxVal);
                cv::rectangle( histImg, cv::Point(h*scale, s*scale),
                            cv::Point( (h+1)*scale - 1, (s+1)*scale - 1),
                            cv::Scalar::all(intensity),
                            CV_FILLED );
            }*/




    cv::namedWindow( "histogram", 1 );
    cv::imshow("histogram", histImage);
    cv::waitKey(0);
    return true;
//V_WINDOW_NORMAL
}


bool ossimDispMerging::imgConversionTo8bit()
{
    double minVal_master, maxVal_master, minVal_slave, maxVal_slave;

    minMaxLoc( master_mat, &minVal_master, &maxVal_master );
    minMaxLoc( slave_mat, &minVal_slave, &maxVal_slave );
    master_mat.convertTo( master_mat_8U, CV_8UC1, 255.0/(maxVal_master - minVal_master), -minVal_master*255.0/(maxVal_master - minVal_master));
    slave_mat.convertTo( slave_mat_8U, CV_8UC1, 255.0/(maxVal_slave - minVal_slave), -minVal_slave*255.0/(maxVal_slave - minVal_slave));

    /*cv::namedWindow( "master_img", CV_WINDOW_NORMAL );
    cv::imshow("master_img", master_mat_8U);

    cv::namedWindow( "slave_img", CV_WINDOW_NORMAL );
    cv::imshow("slave_img", slave_mat_8U);

    cv::waitKey(0);*/

    return true;
}


cv::Mat ossimDispMerging::getMergedDisparity()
{
    return merged_disp;
}


