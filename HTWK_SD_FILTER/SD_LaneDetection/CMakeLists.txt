set(PROJECT SD_LaneDetection)

#include for opencv
include_directories(${OpenCV_INCLUDE_DIR})

adtf_add_filter(${PROJECT} 
 stdafx.h 
 stdafx.cpp 
 AbstractHough.h 
 AbstractHough.cpp
 HoughDefault.h 
 HoughDefault.cpp 
 HoughAlgorithm.h 
 HoughAlgorithm.cpp 
 LaneDetection.h 
 LaneDetection.cpp 
 HoughFast.h 
 HoughFast.cpp 
 LookUpAngles.h 
 LookUpAngles.cpp 
 SmartDrivingUtilities.h
 RealLaneFinder.h
 RealLaneFinder.cpp
)

#linking libs
target_link_libraries(${PROJECT} ${OpenCV_LIBS})

# set folder (only needed and available for MSVC)
adtf_set_folder(${PROJECT} SD_Filter/main/${PROJECT}) 

# Specify where it should be installed to
adtf_install_plugin(${PROJECT} ${AADC_INSTALL_DIR})