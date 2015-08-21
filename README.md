opencv_dsm
==========

OSSIM tool for DSM generation using stereo imagery

===============

This repository contains the source code of an OSSIM plug-in for Digital Surface Models (DSMs) generation from stereo images.

This is the updated code (based on a new and different approach) concerning the 2014 GSoC project "Photogrammetric image processing: DSM generation tool for OSSIM.

For more information about the project see
http://www.google-melange.com/gsoc/proposal/public/google/gsoc2014/martidi/5629499534213120


This repository only contains the new and updated files, with reference to the structure of the OSSIM repository (http://trac.osgeo.org/ossim/browser/trunk/ossim).

In order to compile and install this OSSIM Plug-In use the following instructions:

	1. Install and compile the latest OSSIM version
	2. In the OSSIM_DEV_HOME/ossim_plugins directory modify the CMakeLists.txt adding the line "SUBDIRS(opencv_dsm)" in the IF(BUILD_OSSIMOPENCV_PLUGIN) command
	3. Open a shell in the OSSIM_DEV_HOME/ossim_plugins
	4. Use the following git commands to download D.A.T.E. plug-In
		$git init 
		$git remote add origin https://github.com/martidi/opencv_dsm/tree/master
		$git pull origin master
	5. Re-compile the OSSIM version enabling the OPENCV plugin option in the configuration file
	
For any doubts or issues please email me: martina.dirita@uniroma1.it
