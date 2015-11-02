set(PROJECT SD_MacroPlayer)

#include for opencv
include_directories(${OpenCV_INCLUDE_DIR})

adtf_add_filter(${PROJECT} 
  stdafx.h
  stdafx.cpp
  Macro_player.h
  Macro_player.cpp
  #std_functions.h
  #std_functions.cpp
  Maneuver.h
 Light.h
)

#linking libs
target_link_libraries(${PROJECT} ${OpenCV_LIBS})

# set folder (only needed and available for MSVC)
adtf_set_folder(${PROJECT} SD_Filter/main/${PROJECT}) 

# Specify where it should be installed to
adtf_install_plugin(${PROJECT} ${AADC_INSTALL_DIR})