cmake_minimum_required (VERSION 2.8)

project(ossim-dsm-app )

set(requiredLibs)

# Get the library suffix for lib or lib64.
get_property(LIB64 GLOBAL PROPERTY FIND_LIBRARY_USE_LIB64_PATHS)       
if(LIB64)
   set(LIBSUFFIX 64)
else()
   set(LIBSUFFIX "")
endif()

# OpenThreads - Required:
find_package( OpenThreads )
if( OPENTHREADS_FOUND )
   include_directories( ${OPENTHREADS_INCLUDE_DIR} )
   set( requiredLibs ${requiredLibs} ${OPENTHREADS_LIBRARY} )
else( OPENTHREADS_FOUND )
   message( FATAL_ERROR "Could not find required OpenThreads package!" )
endif( OPENTHREADS_FOUND )

# Requires ossim:
find_package(ossim)
if (OSSIM_FOUND)
   include_directories( ${OSSIM_INCLUDE_DIR} )
   set(requiredLibs ${requiredLibs} ${OSSIM_LIBRARIES} )
else(OSSIM_FOUND)
   message(FATAL_ERROR "Could not find ossim")
endif(OSSIM_FOUND)

SET(requiredLib)

find_package( OpenCV )
if( OPENCV_FOUND )
	include_directories( ${OPENCV_INCLUDE_DIR} )
	set( requiredLib ${requiredLib} ${OPENCV_LIBRARIES} )
else( OPENCV_FOUND )
	message( FATAL_ERROR "Could not find OpenCV package!" )
endif( OPENCV_FOUND )

# Add our include...
include_directories(".")


MESSAGE( STATUS "OPENCV_LIBRARY = ${OPENCV_LIBRARIES}" )
MESSAGE( STATUS "OPENCV_INCLUDE = ${OPENCV_INCLUDE_DIR}" )
message( STATUS "OPENTHREADS_LIBRARY = ${OPENTHREADS_LIBRARY}" )
message( STATUS "OPENTHREADS_INCLUDE = ${OPENTHREADS_INCLUDE_DIR}" )
message( STATUS "OSSIM_INCLUDE_DIR   = ${OSSIM_INCLUDE_DIR}")
message( STATUS "OSSIM_LIBRARIES     = ${OSSIM_LIBRARIES}")
message( STATUS "Required libs       = ${requiredLibs}")

# ---
# ossim-opencv app:
# ---

# Add the executable:
add_executable(ossim-dsm-app ossim-opencv.cpp)

# Set the output dir:
set_target_properties(ossim-dsm-app 
                      PROPERTIES 
                      RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")

# Link the utility and ossim libraries:
message( STATUS "Required libs = ${requiredLibs}" )
target_link_libraries( ossim-dsm-app ${requiredLibs} ossimopencv_dsm_plugin)

# Install app and shared lib:
install(TARGETS ossim-dsm-app
        DESTINATION bin
        LIBRARY DESTINATION)



