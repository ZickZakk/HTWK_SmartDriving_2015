set(PROJECT SD_Simple_fusion)

adtf_add_filter(${PROJECT} 
  Simple_fusion.h 
  stdafx.h 
  stdafx.cpp 
  Simple_fusion.cpp
)

# set folder (only needed and available for MSVC)
adtf_set_folder(${PROJECT} SD_Filter/main/${PROJECT}) 

# Specify where it should be installed to
adtf_install_plugin(${PROJECT} ${AADC_INSTALL_DIR})