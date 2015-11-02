set(FILTER_NAME SD_ObstacleDetection)

# add the include directory of the filter

# declare our filter
adtf_add_filter(${FILTER_NAME}
    stdafx.h
    stdafx.cpp

    obstacledetection.h
    obstacledetection.cpp

    NoiseRemover.h
    NoiseRemover.cpp

    EdgeRemover.h
    EdgeRemover.cpp

    GroundRemover.h
    GroundRemover.cpp

	BirdsEyeViewer.h
	BirdsEyeViewer.cpp

	ConnectedComponentFinder.h
	ConnectedComponentFinder.cpp

	ObstacleExtractor.h
	ObstacleExtractor.cpp

	ObstacleDescription.h
	ObstacleDescription.cpp

	SurfaceExtractor.h
	SurfaceExtractor.cpp

	SurfaceDescription.h
	SurfaceDescription.cpp

	CoordinateConverter.h
	CoordinateConverter.cpp

	ObstacleLocation.h

	../SD_SimpleDrive/IObstacleDetection.h
	Vdar.h
	Vdar.cpp
)

# linking libs
target_link_libraries(${FILTER_NAME} ${OpenCV_LIBS})

# set folder (only needed and available for MSVC)
adtf_set_folder(${FILTER_NAME} SD_Filter/main/${FILTER_NAME}) 

adtf_install_plugin(${FILTER_NAME} ${AADC_INSTALL_DIR})
