cmake_minimum_required(VERSION 2.8.4 FATAL_ERROR)

cmake_policy(SET CMP0002 NEW)
cmake_policy(SET CMP0003 NEW)
cmake_policy(SET CMP0011 NEW)

# The project and cmake vars must only be defined in case that only the base
# filter should be build.
if(NOT AADC_BUILD_ALL_IN_ONE)
    # define a project name
    project(aadc_user)

    # CMAKE_BUILD_TYPE is empty on Unix systems, but needed by ADTF. Otherwise the ADTF libs
    # can't be found and linked.
    if(UNIX AND NOT CMAKE_BUILD_TYPE)
        set(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING
                "Choose the type of build, options are: None Debug Release RelWithDebInfo MinSizeRel."
                FORCE)
    endif(UNIX AND NOT CMAKE_BUILD_TYPE)
    

endif(NOT AADC_BUILD_ALL_IN_ONE)

find_package(ADTF REQUIRED)
#-------General------------

set(AADC_FOUND FALSE)
set(AADC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../)
find_package(AADC REQUIRED)
if (AADC_FOUND)
   message("-- AADC found")
else(AADC_FOUND)
   message(FATAL_ERROR "-- AADC not found")
endif (AADC_FOUND)

#--------------add private macros-----------------
include(${AADC_DIR}/AADC_PRIVATE.cmake)
include(${AADC_DIR}/AADCConfigVersion.cmake)

#--------------sources----------------------------
include_directories(${AADC_DIR}/include)

add_subdirectory(AADC_TemplateFilter)
add_subdirectory(AADC_OpenCVTemplate)
add_subdirectory(SOP_EmergencyBreak)
add_subdirectory(SOP_WheelSpeedController)
add_subdirectory(SOP_ConverterWheels)
add_subdirectory(SOP_ImageProcess)
#add_subdirectory(SOP_ImageProcessing)
#add_subdirectory(SOP_NMPC_Filter)
add_subdirectory(SOP_Position)
add_subdirectory(SOP_SamplingRate)
add_subdirectory(SOP_AutonomousDriving)
add_subdirectory(SOP_StatusTestGenerator)
add_subdirectory(SOP_MarkerDetector)
#add_subdirectory(SOP_RealSense_ImageProcess)
#add_subdirectory(SOP_RearCameraImageProcess)












