//----------------------------------------------------------------------------
//
// License:  See top level LICENSE.txt file.
//
// File: ossimRawImage.h
//
// Author:  Martina Di Rita
//
// Description: Class for individual raw image handling
//
//----------------------------------------------------------------------------
#ifndef ossimRawImage_HEADER
#define ossimRawImage_HEADER 1

#include <opencv/cv.h>
#include <ossim/imaging/ossimImageDataFactory.h>
//#include <ossim/base/ossimStringProperty.h>

class ossimRawImage
{
public:

    int raw_image_id;	
	ossimString raw_image;
	ossimString orbit;

    ossimRawImage();
    void setID(int id);
    int getID();
    void setRawPath(ossimString raw_image_path);
    ossimString getRawPath();
    void setOrbit(ossimString orbit_type);
	ossimString getOrbit();    
    
};

#endif /* #ifndef ossimStereoPair_HEADER */
