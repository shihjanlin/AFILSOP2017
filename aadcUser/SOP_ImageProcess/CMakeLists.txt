set(FILTER_NAME SOP_ImageProcess)

# This is required by CMake
cmake_minimum_required(VERSION 2.8.4 FATAL_ERROR)

INCLUDE_DIRECTORIES(${OpenCV_INCLUDE_DIR})


adtf_add_filter(${FILTER_NAME}

    stdafx.h
    SOP_ImageProcess.h
    SOP_ImageProcess.cpp


    ImageTranslate.h
    ImageTranslate.cpp


    Algorithm/InitialVariable.h
    Algorithm/FunctionType.h
    Algorithm/CameraEnvironment.h



    Algorithm/libSopimgproc.so

)

target_link_libraries(${FILTER_NAME} ${OpenCV_LIBS})

include_directories(${CMAKE_CURRENT_BINARY_DIR}/Algorithm)
find_library(SOPIMGPROCLIB Sopimgproc HINTS ${CMAKE_CURRENT_BINARY_DIR}/../../lib)
target_link_libraries(${FILTER_NAME} ${SOPIMGPROCLIB})


adtf_set_folder(${FILTER_NAME} SOP_ImageProcess) 

# Specify where it should be installed to
adtf_install_plugin(${FILTER_NAME} ${CMAKE_INSTALL_BINARY})

#install dlls for opencv to binary folder 
if(WIN32)
    install(FILES "${OpenCV_DIR}/${OpenCV_ARCH}/${OpenCV_RUNTIME}/bin/opencv_world${OpenCV_VERSION_MAJOR}${OpenCV_VERSION_MINOR}${OpenCV_VERSION_PATCH}d.dll" DESTINATION ${CMAKE_INSTALL_BINARY} CONFIGURATIONS Debug)
    install(FILES "${OpenCV_DIR}/${OpenCV_ARCH}/${OpenCV_RUNTIME}/bin/opencv_world${OpenCV_VERSION_MAJOR}${OpenCV_VERSION_MINOR}${OpenCV_VERSION_PATCH}.dll" DESTINATION ${CMAKE_INSTALL_BINARY} CONFIGURATIONS RelWithDebInfo Release)
    
    install(FILES "${Qt_DIR}/bin/QtTest4.dll" DESTINATION ${CMAKE_INSTALL_BINARY} CONFIGURATIONS Release RelWithDebInfo) 
    install(FILES "${Qt_DIR}/bin/QtTestd4.dll" DESTINATION ${CMAKE_INSTALL_BINARY} CONFIGURATIONS Debug) 
endif(WIN32)
