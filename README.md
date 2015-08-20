opencv_dsm
==========

OSSIM tool for DSM generation using stereo imagery

===============

This is the repository for OSSIM GSoC 2015: it contains the code for the development of an OSSIM plug-In for Digital Surface Models (DSMs) generation from tri-stereo images.

For more information about the project see
https://www.google-melange.com/gsoc/proposal/review/student/google/gsoc2015/martidi/5717271485874176

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
