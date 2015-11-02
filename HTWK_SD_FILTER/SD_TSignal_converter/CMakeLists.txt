set(PROJECT SD_TSignal_converter)

adtf_add_filter(${PROJECT} 
  stdafx.h
  TSignal_converter.h 
  stdafx.cpp 
  TSignal_converter.cpp
)

# set folder (only needed and available for MSVC)
adtf_set_folder(${PROJECT} SD_Filter/main/${PROJECT}) 

# Specify where it should be installed to
adtf_install_plugin(${PROJECT} ${AADC_INSTALL_DIR})