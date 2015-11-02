set(FILTER_NAME AADC_DriverModule)
  
adtf_add_filter(${FILTER_NAME}
					stdafx.h
                    stdafx.cpp
					displaywidget.h
                    displaywidget.cpp
                    cDriverFilter.h
					cDriverFilter.cpp
					Maneuverlist.h
    )

    
# set folder (only needed and available for MSVC)
adtf_set_folder(${FILTER_NAME} SD_Filter/main/${FILTER_NAME}) 
    
adtf_install_plugin(${FILTER_NAME} ${AADC_INSTALL_DIR})