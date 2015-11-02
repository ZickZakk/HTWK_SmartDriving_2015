cmake_minimum_required(VERSION 2.8.4 FATAL_ERROR)

cmake_policy(SET CMP0002 NEW)
cmake_policy(SET CMP0003 NEW)
cmake_policy(SET CMP0011 NEW)

# The project and cmake vars must only be defined in case that only the base
# filter should be build.
if(NOT AADC_BUILD_ALL_IN_ONE)
    # define a project name
    project(epetersen)

    #set the install prefix if not already done.
    if(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
        set(CMAKE_INSTALL_PREFIX "${CMAKE_CURRENT_SOURCE_DIR}/../../bin" CACHE PATH "The install directory" FORCE) 
        set(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT FALSE)
    endif(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
    # CMAKE_BUILD_TYPE is empty on Unix systems, but needed by ADTF. Otherwise the ADTF libs
    # can't be found and linked.
    if(UNIX AND NOT CMAKE_BUILD_TYPE)
        set(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING
                "Choose the type of build, options are: None Debug Release RelWithDebInfo MinSizeRel."
                FORCE)
    endif(UNIX AND NOT CMAKE_BUILD_TYPE)
    
    # set toolchain file on Unix
    if(UNIX AND NOT CMAKE_TOOLCHAINFILE)
        set(CMAKE_TOOLCHAIN_FILE "/opt/adtf/toolchainfiles/ARM_toolchain_build_on_target.cmake")
    endif(UNIX AND NOT CMAKE_TOOLCHAINFILE)

endif(NOT AADC_BUILD_ALL_IN_ONE)

find_package(ADTF REQUIRED)
#-------General------------


set(AADC_INSTALL_DIR .)
set(AADC_INSTALL_DIR_LIBS .)

set(AADC_DIR ../)
find_package(AADC REQUIRED)

#--------------sources----------------------------

#add_subdirectory(AADC_JuryTransmitter)
add_subdirectory(SD_Simple_fusion)
add_subdirectory(SD_Emergency_system)
#add_subdirectory(SD_Value_generator)
#add_subdirectory(SD_TSignal_converter)
#add_subdirectory(SD_World_model)
#add_subdirectory(SD_ImageCutter)
#add_subdirectory(SD_LaneDetection)
#add_subdirectory(SD_Drive_by_list)
#add_subdirectory(SD_Extended_fusion)
#add_subdirectory(SD_ObstacleDetection)
#add_subdirectory(SD_CurveDetection)
#add_subdirectory(SD_HorizonCalibration)
add_subdirectory(SD_IPM)
#add_subdirectory(SD_CurveDetection)
#add_subdirectory(SD_MacroRecorder)
add_subdirectory(SD_MacroPlayer)
add_subdirectory(SD_VideoImageWriter)
add_subdirectory(SD_IPM_Calibration)
add_subdirectory(SD_SimpleDrive)
add_subdirectory(SD_SmartCurve)
add_subdirectory(SD_RoadSignEvaluator)
add_subdirectory(SD_DriverFilter)
add_subdirectory(SD_LightFilter)
add_subdirectory(SD_FreestyleSchnapsSaufen)
#add_subdirectory(SD_SteerAngleChecker)
#add_subdirectory(SD_FREESTYLE)
#add_subdirectory(SD_NetIO_server)
#add_subdirectory(SD_NetIO_client)
