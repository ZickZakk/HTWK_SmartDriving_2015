set(PROJECT SD_FreestyleSchnapsSaufen)

#include for opencv
include_directories(${OpenCV_INCLUDE_DIR})

adtf_add_filter(${PROJECT} 
	stdafx.h 
	stdafx.cpp 
	Steer.h
	Lane.h
	Freestyle.h 
	Freestyle.cpp
	ICrossRoadDetector.h
	CrossRoadDetector.h
	CrossRoadDetector.cpp
	IDriveAlgorithm.h
	DriveAlgorithm.h
	DriveAlgorithm.cpp
	Server.h
	Server.cpp
	Client.h
	Client.cpp
)

#linking libs
target_link_libraries(${PROJECT} ${OpenCV_LIBS})

# set folder (only needed and available for MSVC)
adtf_set_folder(${PROJECT} SD_Filter/main/${PROJECT}) 

# Specify where it should be installed to
adtf_install_plugin(${PROJECT} ${AADC_INSTALL_DIR})
