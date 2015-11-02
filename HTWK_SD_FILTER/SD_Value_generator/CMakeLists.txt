set(PROJECT SD_Value_generator)

adtf_add_filter(${PROJECT}
  stdafx.h 
  Value_generator.h
  stdafx.cpp
  Value_generator.cpp
)

# set folder (only needed and available for MSVC)
adtf_set_folder(${PROJECT} SD_Filter/main/${PROJECT}) 

# Specify where it should be installed to
adtf_install_plugin(${PROJECT} ${AADC_INSTALL_DIR})