set(PROJECT SD_Emergency_system)

adtf_add_filter(${PROJECT} 
  stdafx.h
  Emergency_system.h 
  Maneuver.h 
  stdafx.cpp 
  Emergency_system.cpp
)

# set folder (only needed and available for MSVC)
adtf_set_folder(${PROJECT} SD_Filter/main/${PROJECT}) 

# Specify where it should be installed to
adtf_install_plugin(${PROJECT} ${AADC_INSTALL_DIR})