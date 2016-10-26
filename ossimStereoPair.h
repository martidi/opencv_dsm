//----------------------------------------------------------------------------
//
// License:  See top level LICENSE.txt file.
//
// File: openCVtestclass.h
//
// Author:  Martina Di Rita
//
// Description: Class providing OpenCV functions for DSM extraction
//
//----------------------------------------------------------------------------
#ifndef ossimStereoPair_HEADER
#define ossimStereoPair_HEADER 1

#include <opencv/cv.h>
#include <ossim/imaging/ossimImageDataFactory.h>
//#include <ossim/base/ossimStringProperty.h>

class ossimStereoPair
{
public:

    ossimString raw_master, raw_slave;
    ossimString ortho_master, ortho_slave;
    int id;
    //float conversion_factor; //=0;
    int id_slave, id_master;
    double deltaH;
    double mean_rotation_angle;
    double mean_conversion_factor;


    ossimStereoPair();
    void setID(int IDM, int IDS);
    int get_id_master();
    int get_id_slave();
    void setRawPath(ossimString raw_master_path, ossimString raw_slave_path);
    void setOrthoPath(ossimString ortho_master_path, ossimString ortho_slave_path);
    ossimString getRawMasterPath();
    ossimString getRawSlavePath();
    ossimString getOrthoMasterPath();
    ossimString getOrthoSlavePath();
    void epipolarDirection();
    double getMeanRotationAngle();
    double getConversionFactor();

    //void computeConversionFactor( double longitude_max, double longitude_min, double latitude_max, double latitude_min, double MinimumHeight , double MaximumHeight);
    //float getConversionFactor();
};

#endif /* #ifndef ossimStereoPair_HEADER */
