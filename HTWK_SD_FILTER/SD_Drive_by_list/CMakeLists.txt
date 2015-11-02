set(PROJECT SD_Drive_by_list)

adtf_add_filter(${PROJECT}
  stdafx.h 
  Drive_by_list.h
  stdafx.cpp
  Drive_by_list.cpp
)

# set folder (only needed and available for MSVC)
adtf_set_folder(${PROJECT} SD_Filter/main/${PROJECT}) 

# Specify where it should be installed to
adtf_install_plugin(${PROJECT} ${AADC_INSTALL_DIR})