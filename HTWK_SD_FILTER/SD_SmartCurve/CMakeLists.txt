set(PROJECT SD_SmartCurve)

#include for opencv
include_directories(${OpenCV_INCLUDE_DIR})

adtf_add_filter(${PROJECT} 
 stdafx.h 
 stdafx.cpp 
 SmartCurve.h
 SmartCurve.cpp

 IWheelRotation.h
 WheelRotation.h
 WheelRotation.cpp
)

#linking libs
target_link_libraries(${PROJECT} ${OpenCV_LIBS})

# set folder (only needed and available for MSVC)
adtf_set_folder(${PROJECT} SD_Filter/main/${PROJECT}) 

# Specify where it should be installed to
adtf_install_plugin(${PROJECT} ${AADC_INSTALL_DIR})