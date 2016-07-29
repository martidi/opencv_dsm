//----------------------------------------------------------------------------
//
// License:  See top level LICENSE.txt file.
//
// File: openCVtestclass.h
//
// Author:  Martina Di Rita
//
// Description: Class for individual raw image handling
//
//----------------------------------------------------------------------------

#include "ossim/imaging/ossimImageHandlerRegistry.h"
#include "ossim/imaging/ossimImageHandler.h"
#include "ossimRawImage.h"

ossimRawImage::ossimRawImage()
{
	
}

void ossimRawImage::setID(int id)
{
        raw_image_id = id;
}

int ossimRawImage::getID()
{
        return raw_image_id;
}

void ossimRawImage::setRawPath(ossimString raw_image_path)
{
	raw_image = raw_image_path;
}

ossimString ossimRawImage::getRawPath()
{
	return raw_image;
}

void ossimRawImage::setOrbit(ossimString orbit_type)
{
	orbit = orbit_type;
}

ossimString ossimRawImage::getOrbit()
{
	return orbit;
}
	
	
/*
ossimString ossimRawImage::getOrthoMasterPath()
{
        return ortho_master;
}

void ossimStereoPair::setOrthoPath(ossimString ortho_master_path, ossimString ortho_slave_path)
{
        ortho_master = ortho_master_path;
        ortho_slave = ortho_slave_path;
}*/
    
    
    
