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


bool ossimDispMerging::execute(vector<ossimStereoPair> StereoPairList, vector<ossimString> orthoListMask)
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

        imgConversionTo8bit();      // Conversion from 16 bit to 8 bit

        // TPs generation
        ossimOpenCvTPgenerator *stereoTP = new ossimOpenCvTPgenerator(master_mat_8U, slave_mat_8U) ;
        stereoTP->execute();

        // Disparity map generation
        ossimOpenCvDisparityMapGenerator* dense_matcher = new ossimOpenCvDisparityMapGenerator();
        dense_matcher->execute(master_mat_8U, stereoTP->getWarpedImage(), StereoPairList[i]); // dopo questo execute ho disp metrica

        // Nel vettore globale di cv::Mat immagazzino tutte le mappe di disparità che genero ad ogni ciclo
        array_metric_disp.push_back(dense_matcher->getDisp());
        null_disp_threshold = (dense_matcher->minimumDisp)+0.5;
        }
    cout << "I'm here " << endl;

    // Only for the first pyramidal level; check if mask file already exist
    fstream f_input;
    f_input.open("Ascending_total_mask.tif");

    if (f_input.fail())
    {
        cout << "Le maschere ancora non sono state generate! " << endl;
        cv::Mat mask_mat;
        vector<cv::Mat> mask_mat_array;

        // Conversion from ossim image to opencv matrix
        for(int i=0; i < orthoListMask.size(); i++)
            {
            // Mask ImageHandlers
            ossimImageHandler* mask_handler = ossimImageHandlerRegistry::instance()->open(orthoListMask[i]);

            // Load ortho images
            ossimIrect bounds_master = mask_handler->getBoundingRect(0);
            ossimRefPtr<ossimImageData> img_master = mask_handler->getTile(bounds_master, 0);

            // Create the OpenCV images
            mask_mat.create(cv::Size(img_master->getWidth(), img_master->getHeight()), CV_16UC1);
            memcpy(mask_mat.ptr(), (void*) img_master->getUshortBuf(), 2*img_master->getWidth()*img_master->getHeight());

            cout << endl << "OSSIM->OpenCV mask conversion done" << endl;

            // Rotation for along-track OPTICAL images
            //********* To be commented for SAR images *********
            //cv::transpose(mask_mat, mask_mat);
            //cv::flip(mask_mat, mask_mat, 1);
            //********* To be commented for SAR images *********

            // così creo ad ogni ciclo un'immagine di tipo cv::Mat, la salvo in un'array per poi sommare le prime tre e
            // le seconde tre

            mask_mat_array.push_back(mask_mat);
            }

        //per ora ne faccio uno solo per asc e desc, poi vorrei farne due separati
        int rowsNumb_asc, colsNumb_asc;
        rowsNumb_asc = mask_mat_array[0].size[0];
        colsNumb_asc = mask_mat_array[0].size[1];
 //cout << "size list ortho mask" << orthoListMask.size() << endl;
 // cout << "size array mask " << mask_mat_array.size() << endl;
        for(int i=0; i < mask_mat_array.size(); i++)
        {
            if (rowsNumb_asc > mask_mat_array[i].size[0]) rowsNumb_asc=mask_mat_array[i].size[0];
            if (colsNumb_asc > mask_mat_array[i].size[1]) colsNumb_asc=mask_mat_array[i].size[1];
        }

        // Ridimensiono tutte le immagini sulle dimensioni più piccole
        for(int i=0; i < mask_mat_array.size(); i++)
        {
            mask_mat_array[i](cv::Rect(0,0,colsNumb_asc,rowsNumb_asc)).copyTo(mask_mat_array[i]);
        }

        /*cout << "righe " << rowsNumb_asc << endl;
        cout << "colonne " << colsNumb_asc << endl;
        cout << "Size mask 0 " << mask_mat_array[0].size[0] << endl;
        cout << "Size mask 1 " << mask_mat_array[1].size() << endl;
        cout << "Size mask 2 " << mask_mat_array[2].size() << endl;
        cout << "Size mask 3 " << mask_mat_array[3].size() << endl;
        cout << "Size mask 4 " << mask_mat_array[4].size() << endl;
        cout << "Size mask 5 " << mask_mat_array[5].size() << endl;*/

        cv::Mat mask_ascending_tot = cv::Mat::zeros(rowsNumb_asc, colsNumb_asc, CV_64F);
        cv::Mat mask_descending_tot = cv::Mat::zeros(rowsNumb_asc, colsNumb_asc, CV_64F);

        // Sommo le tre maschere ascendenti e poi le altre tre discendenti
        mask_ascending_tot = mask_mat_array[0] + mask_mat_array[1] + mask_mat_array[2];
        mask_descending_tot = mask_mat_array[3] + mask_mat_array[4] + mask_mat_array[5];

        // Saving masks on file
        //cv::namedWindow( "Ascending total mask", CV_WINDOW_NORMAL );
        //cv::imshow( "Ascending total mask", mask_ascending_tot);
        cv::imwrite( "Ascending_total_mask.tif", mask_ascending_tot);
        //cv::namedWindow( "Descending total mask", CV_WINDOW_NORMAL );
        //cv::imshow( "Descending total mask", mask_descending_tot);
        cv::imwrite( "Descending_total_mask.tif", mask_descending_tot);

        //cv::waitKey(0);
    }

    cout << endl << "Disparity maps number: " << array_metric_disp.size() << endl;
    /*cout << "Zero " << array_metric_disp[0].rows << " " << array_metric_disp[0].cols << endl;
    cout << "Uno " << array_metric_disp[1].rows << " " << array_metric_disp[1].cols << endl;
    cout << "Due " << array_metric_disp[2].rows << " " << array_metric_disp[2].cols << endl;
    cout << "Tre " << array_metric_disp[3].rows << " " << array_metric_disp[3].cols << endl;
    cout << "Quattro " << array_metric_disp[4].rows << " " << array_metric_disp[4].cols << endl;
    cout << "Cinque " << array_metric_disp[5].rows << " " << array_metric_disp[5].cols << endl;*/
    // sembra che non siano tutte uguali, per ora me ne frego e prendo come dimensione la prima

    // DISPARITY MAPS FUSION
    // Ora fondo le mappe di disparità tutte insieme dando un peso differente (tramite le due maschere appena generate)
    merged_disp = cv::Mat::zeros(array_metric_disp[0].rows, array_metric_disp[0].cols, CV_64F);

    cout<< " " << endl << "DISPARITY MAPS FUSION \t wait few minutes..." << endl;

    cout << "n° rows\t" << merged_disp.rows << endl;
    cout << "n° columns\t" << merged_disp.cols << endl;

    // Creo la matrice con i valori Mediani e dev.st. dell'array ascending
    cv::Mat median_ascending_array = cv::Mat::zeros(array_metric_disp[0].rows, array_metric_disp[0].cols, CV_64F);
    cv::Mat std_ascending_array = cv::Mat::zeros(array_metric_disp[0].rows, array_metric_disp[0].cols, CV_64F);

    // Creo la matrice con i valori Mediani e dev.st. dell'array descending
    cv::Mat median_descending_array = cv::Mat::zeros(array_metric_disp[0].rows, array_metric_disp[0].cols, CV_64F);
    cv::Mat std_descending_array = cv::Mat::zeros(array_metric_disp[0].rows, array_metric_disp[0].cols, CV_64F);

    // Per ogni riga, per ogni colonna, per le prima 3 mappe di disp, calcolo mediana e deviazione standard

    float alpha = 0.5;
    float beta = 0.5;
    for (int i=0; i< array_metric_disp[0].rows; i++) // for every row
    {
        for(int j=0; j< array_metric_disp[0].cols; j++) // for every column
        {
            // ASCENDING MEAN AND ST.DEV COMPUTATION
            vector<double> temp_asc, temp_desc;
            //for (unsigned int k = 0; k < array_metric_disp.size(); k++)  // for every disparity map
            for (unsigned int k = 0; k < 3; k++)  // for every ASCENDING disparity map
            {
                // creo un array con i valori in (i,j) delle k mappe di disparità
                double single_pixel_value = array_metric_disp[k].at<double>(i,j);
                temp_asc.push_back(single_pixel_value);
            }

            // Calcolo media e dev.st. con meanStdDev, poi faccio il push_back del valore nella Mat
            // median_ascending_array.push_back(mean_ascending)

            cv::Scalar mean_ascending, stDev_ascending;
            cv::meanStdDev(temp_asc, mean_ascending, stDev_ascending);
            //median_ascending_array(i,j).push_back(mean_ascending);
            //std_ascending_array.push_back(stDev_ascending);

            median_ascending_array.at<cv::Scalar>(i,j) = mean_ascending;
            std_ascending_array.at<cv::Scalar>(i,j) = stDev_ascending;

            // DESCENDING MEAN AND ST.DEV COMPUTATION
            for (unsigned int k = 3; k < 6; k++)  // for every DESCENDING disparity map
            {
                // creo un array con i valori in (i,j) delle k mappe di disparità
                double single_pixel_value = array_metric_disp[k].at<double>(i,j);
                temp_desc.push_back(single_pixel_value);
            }

            // Calcolo media e dev.st. con meanStdDev, poi faccio il push_back del valore nella Mat
            // median_ascending_array.push_back(mean_ascending)

            cv::Scalar mean_descending, stDev_descending;
            cv::meanStdDev(temp_desc, mean_descending, stDev_descending);
            //median_ascending_array(i,j).push_back(mean_ascending);
            //std_ascending_array.push_back(stDev_ascending);

            median_descending_array.at<cv::Scalar>(i,j) = mean_descending;
            std_descending_array.at<cv::Scalar>(i,j) = stDev_descending;
        }
    }

    // DISPARITY MAP MERGING

    for (int i=0; i< array_metric_disp[0].rows; i++) // for every row
    {
        for(int j=0; j< array_metric_disp[0].cols; j++) // for every column
        {
            int num=0.0;

            // ma questo for sotto serve????????
            for (unsigned int k = 0; k < array_metric_disp.size(); k++)  // for every disparity map
            if(array_metric_disp[k].at<double>(i,j) > null_disp_threshold) // sto togliendo i valori minori della threshold
            {
                merged_disp.at<double>(i,j) += array_metric_disp[k].at<double>(i,j);
                merged_disp.at<double>(i,j) = // formula
                num++;

            }

            else
            {
               num = 1.0;
               merged_disp.at<double>(i,j) = array_metric_disp[0].at<double>(i,j);
            }
            merged_disp.at<double>(i,j)  = merged_disp.at<double>(i,j) /num;
        }
    }


    /*
    //for (int z = 0; z < k; z++)
    //{
        //if(fabs(array_metric_disp[z].at<double>(i,j) - array_metric_disp[z+1].at<double>(i,j)) < 5)
        //{
            if(array_metric_disp[k].at<double>(i,j) > null_disp_threshold) // sto togliendo i valori minori della threshold
            {
                merged_disp.at<double>(i,j) += array_metric_disp[k].at<double>(i,j);
                num++;
            }
        //}

        else
        {
           num = 1.0;
           merged_disp.at<double>(i,j) = array_metric_disp[0].at<double>(i,j);
        }

    //}
merged_disp.at<double>(i,j)  = merged_disp.at<double>(i,j) /num;*/



     //remove(ossimFilename(ossimFilename(ap[2]) + ossimString("temp_elevation/") + ossimFilename(ap[3])+ossimString(".TIF")));

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



bool ossimDispMerging::imgGetHisto(cv::Mat image, double threshold,  double *minHisto, double *maxHisto)
{
    cout << "Histogram computation " << endl;

    double minVal, maxVal;
    minMaxLoc( image, &minVal, &maxVal );
    cout << "min\t" << minVal << " " << "max\t" << maxVal << endl;

    /// Establish the number of bins
    int histSize = maxVal - minVal;

    /// Set the ranges ( for B,G,R) )
    float range[] = { minVal, maxVal } ;
    const float* histRange = { range };
    ///We want our bins to have the same size (uniform) and to clear the histograms in the beginning, so:
    bool uniform = true; bool accumulate = false;
    /// we create the Mat objects to save our histogram
    cv::Mat hist;

    /// Compute the histograms:
    cv::calcHist( &image, 1, 0, cv::Mat(), hist, 1, &histSize, &histRange, uniform, accumulate );



    //Scarto il 10% dei valori più alti
    cout << "size\t" << hist.size() << endl;
    //cout << "ihst\t" << hist << endl;

    //devo sommare tutti i valori dei bin
    //devo trovare i valori di i per cui tolgo il 5%
    int sum = 0;
    bool control = true;
    int max = hist.rows;
    int min = 0;
    int col = image.cols;
    int row = image.rows;
    //double threshold = 5.0;

    cout <<"num colonne istogramma: " <<max << " col master\t" << col << " righe master " << row << endl;

    cout << hist.at<float>(785,0) << endl; // con at<int>  e at<float> sballa il risultato
    while(control)
    {
        sum+=hist.at<float>(max-1,0) ;
        //cout << "somma " << sum << endl;
        max--;
        if (sum*100.0/(col*row) > threshold) control = false;
    }
    //cout << "somma dei valori che scarto " << sum<< endl;
    sum = 0;
    control = true;

    while(control)
    {
        sum+=hist.at<float>(min,0) ;
        //cout << "somma " << sum << endl;
        min++;
        if (sum*100.0/(col*row) > threshold) control = false;
    }


    cout << "indice dell'istogramma fino al quale ho il 95% della distribuzione "<<max << endl;
    //cout << "somma dei valori che scarto " << sum<< endl;
    cout << "indice minimo dell'istogramma "<< min << endl;
    maxVal = minVal + max;
    minVal = minVal + min;

    *maxHisto = maxVal;
    *minHisto = minVal;

    cout << "nuovo max " << maxVal << "nuovo min " << minVal << endl;
    //Ora "i" deve diventare il mio nuovo maxVal su cui stretcho l'istogramma?


    // Histo remapping
    histSize = maxVal - minVal;
    /// Set the ranges ( for B,G,R) )
    float rangeEqual[] = { minVal, maxVal } ;
    histRange = { rangeEqual };
    ///We want our bins to have the same size (uniform) and to clear the histograms in the beginning, so:

    /// Compute the histograms:
    cv::calcHist( &image , 1, 0, cv::Mat(), hist, 1, &histSize, &histRange, uniform, accumulate );




    /// Create an image to display the histograms
    ///Draw the histograms
    int hist_w = 512; int hist_h = 400;
    int bin_w = cvRound( (double) hist_w/histSize);

    cv::Mat histImage( hist_h, hist_w, CV_8UC3, cv::Scalar( 0,0,0) );

    /// Normalize the result to [ 0, histImage.rows ]
    normalize(hist, hist, 0, histImage.rows, cv::NORM_MINMAX, -1, cv::Mat() );

    cout << hist.at<float>(785,0) << endl;

    /// Draw
    for( int i = 1; i < histSize; i++ )
    {
        line( histImage, cv::Point( bin_w*(i-1), hist_h - cvRound(hist.at<float>(i-1)) ) ,
                         cv::Point( bin_w*(i), hist_h - cvRound(hist.at<float>(i)) ),
                         cv::Scalar( 255, 0, 0), 2, 8, 0  );
    }

    /*cv::namedWindow( "histogram", 1 );
    cv::imshow("histogram", histImage);
    cv::waitKey(0);*/
    return true;
//CV_WINDOW_NORMAL
}


bool ossimDispMerging::imgConversionTo8bit()
{
    double minVal_master, maxVal_master, minVal_slave, maxVal_slave;
    double threshold = 5.0;

    imgGetHisto(master_mat, threshold, &minVal_master, &maxVal_master ); // histogram computation and equalization; riempie min e max
    imgGetHisto(slave_mat, threshold, &minVal_slave, &maxVal_slave ); // histogram computation and equalization; riempie min e max

    //minMaxLoc( master_mat, &minVal_master, &maxVal_master );
    //minMaxLoc( slave_mat, &minVal_slave, &maxVal_slave );

    cout << minVal_master << " max master"<< maxVal_master << "min slave " << minVal_slave << " max slave " << maxVal_slave << endl;
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


