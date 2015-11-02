# execute just once
if(AADC_FOUND)
    message(STATUS "AADC config file already found.")
    return()
endif(AADC_FOUND)

set(AADC_OPENCV_FOUND FALSE)
set(AADC_OPENNI2_FOUND FALSE)
set(AADC_ZBAR_FOUND FALSE)

#-------OpenCV------------------------------------------
find_package(OpenCV 
                REQUIRED 
                COMPONENTS 
                    opencv_core 
                    opencv_video 
                    opencv_imgproc 
                    opencv_highgui 
                    opencv_features2d 
                    opencv_nonfree
                PATHS 
                    "/opt/arm-linux-gnueabihf/SDK/gcc-4.8/opencv/3.0.0" 
                    "C:/SDK/opencv/3.0.0/build"
                    "C:/SDK/opencv/3.0.0"
            )

if(NOT OpenCV_LIBS)
    message(FATAL_ERROR "Opencv lib not found. Please specify the OPENCV_DIR")
else(NOT OpenCV_LIBS)
    set(AADC_OPENCV_FOUND TRUE)
	if(NOT UNIX)
		set(OpenCV_INCLUDE_DIR "C:/SDK/opencv/3.0.0/build/include" CACHE PATH "The include directory of Opencv")
	endif(NOT UNIX)		
endif(NOT OpenCV_LIBS)
            
#--------OpenNI2----------------------------------------	
# OpenNI does not provide a CMake config file.
# So we have to set it from the outside.
if(UNIX)
    set(OPENNI2_DIR "/opt/arm-linux-gnueabihf/SDK/gcc-4.8/OpenNi/2.2.0" CACHE PATH "The root directory of OpenNI2")
else(UNIX)
    set(OPENNI2_DIR "C:/SDK/OpenNI2" CACHE PATH "The root directory of OpenNI2")
endif(UNIX)

FIND_LIBRARY(OPENNI2_LIBS
                NAMES 
                    OpenNI2
                PATHS 
                    "${OPENNI2_DIR}/Bin/Arm-Release/"
                    "${OPENNI2_DIR}/Lib"
                PATH_SUFFIXES so
                )
#set(OPENNI2_DIR "/home/parallels/Desktop/OpenNI-master/Platform/Linux/Redist/OpenNI-Bin-Dev-Linux-x64-v1.5.7.10" CACHE PATH "The root directory of OpenNI2")
#message(FATAL_ERROR "${OPENNI2_DIR}")

if(NOT OPENNI2_LIBS)
    message(FATAL_ERROR "OpenNI2 lib not found. Please specify the OPENNI2_DIR")
else(NOT OPENNI2_LIBS)
    set(AADC_OPENNI2_FOUND TRUE)
endif(NOT OPENNI2_LIBS)
                
set(OPENNI2_INCLUDE_DIR "${OPENNI2_DIR}/Include/" CACHE PATH "The include directory of OpenNI2") 

#--------zbar------------------------------------------
# zbar does not provide a CMake config file.
# So we have to set it from the outside.
# currently there is no x64 version for windows available.
if(NOT CMAKE_CL_64)
    if(UNIX)
        set(ZBAR_DIR "/opt/arm-linux-gnueabihf/SDK/gcc-4.6/zbar/0.10/install" CACHE PATH "The root directory of ZBar")
    else(UNIX)
        set(ZBAR_DIR "C:/SDK/ZBar" CACHE PATH "The root directory of ZBar")
    endif(UNIX)

    set(ZBAR_LIB_DIR "${ZBAR_DIR}/lib/" CACHE PATH "The lib directory of ZBar" )
    set(ZBAR_BIN_DIR "${ZBAR_DIR}/bin/" CACHE PATH "The bin directory of ZBar" )

    FIND_LIBRARY(ZBAR_LIBS
                    NAMES 
                        zbar
                        libzbar-0
                    PATHS 
                        "${ZBAR_LIB_DIR}"
                    )


    if(NOT ZBAR_LIBS)
        message(FATAL_ERROR "ZBar lib not found. Please specify the ZBAR_DIR.")
    else(NOT ZBAR_LIBS)
        set(AADC_ZBAR_FOUND TRUE)
    endif(NOT ZBAR_LIBS)
                    
    set(ZBAR_INCLUDE_DIR "${ZBAR_DIR}/include/" CACHE PATH "The include directory of ZBar" )
else(NOT CMAKE_CL_64)
    # hack to avoid running this config file again.
    set(AADC_ZBAR_FOUND TRUE)
endif(NOT CMAKE_CL_64)

if(NOT AADC_OPENCV_FOUND OR NOT AADC_OPENNI2_FOUND OR NOT AADC_ZBAR_FOUND)
    message(FATAL "At least one of the required libraries is not found")
endif(NOT AADC_OPENCV_FOUND OR NOT AADC_OPENNI2_FOUND OR NOT AADC_ZBAR_FOUND)
