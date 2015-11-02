set(PROJECT SD_SimpleDrive)

#include for opencv
include_directories(${OpenCV_INCLUDE_DIR})

adtf_add_filter(${PROJECT} 
 stdafx.h 
 stdafx.cpp 
 SimpleDrive.h
 SimpleDrive.cpp
 SmartDrivingUtilities.h
 ICrossRoadDetector.h
 CrossRoadDetector.h
 CrossRoadDetector.cpp
 IDriveAlgorithm.h
 DriveAlgorithm.h
 DriveAlgorithm.cpp
 PI_Regler.h
 PI_Regler.cpp
 IEmergencySystem.h
 EmergencySystem.h
 EmergencySystem.cpp
 IObstacleDetection.h
 IParkGap.h
 ParkGap.h
 ParkGap.cpp
 IDecisionEvaluator.h
 DecisionEvaluator.h
 DecisionEvaluator.cpp
 Maneuver.h
 Decision.h
 Steer.h
 Lane.h
 Light.h
 Turn.h
	IObstacleDetection.h
	NoiseRemover.h
	NoiseRemover.cpp
	GroundRemover.h
	GroundRemover.cpp
	CoordinateConverter.h
	CoordinateConverter.cpp
	BirdsEyeViewer.h
	BirdsEyeViewer.cpp
	ConnectedComponentFinder.h
	ConnectedComponentFinder.cpp
	Vdar.h
	Vdar.cpp
)

#linking libs
target_link_libraries(${PROJECT} ${OpenCV_LIBS})

# set folder (only needed and available for MSVC)
adtf_set_folder(${PROJECT} SD_Filter/main/${PROJECT}) 

# Specify where it should be installed to
adtf_install_plugin(${PROJECT} ${AADC_INSTALL_DIR})
