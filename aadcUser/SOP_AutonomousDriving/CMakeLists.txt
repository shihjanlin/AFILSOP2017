# add the include directory of the filter
include_directories(../include)
include_directories(/home/aadc/thirdparty_libs/ipopt3124/include/coin)
include_directories(/home/aadc/thirdparty_libs/ipopt3124/include/coin/Thirdparty)
include_directories(/home/aadc/thirdparty_libs/eigen334/include)

set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "${CMAKE_CXX_FLAGS} -O3")


set(FILTER_NAME SOP_AutonomousDriving)

# This is required by CMake
cmake_minimum_required(VERSION 2.8.4 FATAL_ERROR)

INCLUDE_DIRECTORIES(${OpenCV_INCLUDE_DIR})


adtf_add_filter(${FILTER_NAME}
    stdafx.h

    SOP_AutonomousDriving.h
    SOP_AutonomousDriving.cpp


    NMPC_Controller.cpp
    Data_Processing.cpp
    State_Control.cpp

    #parameter_settings.h
    #audi_q2_nlp.cpp
    #audi_q2_nlp.h
    #rungekutta.h
    #rungekutta.cpp


    Nmpc/parameter_settings.h
    Nmpc/audi_q2_nlp.cpp
    Nmpc/audi_q2_nlp.h
    Nmpc/rungekutta.h
    Nmpc/rungekutta.cpp


)

target_link_libraries(${FILTER_NAME} ${OpenCV_LIBS})
#link_directories(${CMAKE_CURRENT_SOURCE_DIR}/ipopt3124/lib)
target_link_libraries(${FILTER_NAME} "/home/aadc/thirdparty_libs/ipopt3124/lib/libipopt.so")

adtf_set_folder(${FILTER_NAME} SOP_AutonomousDriving) 

# Specify where it should be installed to
adtf_install_plugin(${FILTER_NAME} ${CMAKE_INSTALL_BINARY})

#install dlls for opencv to binary folder 
if(WIN32)
    install(FILES "${OpenCV_DIR}/${OpenCV_ARCH}/${OpenCV_RUNTIME}/bin/opencv_world${OpenCV_VERSION_MAJOR}${OpenCV_VERSION_MINOR}${OpenCV_VERSION_PATCH}d.dll" DESTINATION ${CMAKE_INSTALL_BINARY} CONFIGURATIONS Debug)
    install(FILES "${OpenCV_DIR}/${OpenCV_ARCH}/${OpenCV_RUNTIME}/bin/opencv_world${OpenCV_VERSION_MAJOR}${OpenCV_VERSION_MINOR}${OpenCV_VERSION_PATCH}.dll" DESTINATION ${CMAKE_INSTALL_BINARY} CONFIGURATIONS RelWithDebInfo Release)
    
    install(FILES "${Qt_DIR}/bin/QtTest4.dll" DESTINATION ${CMAKE_INSTALL_BINARY} CONFIGURATIONS Release RelWithDebInfo) 
    install(FILES "${Qt_DIR}/bin/QtTestd4.dll" DESTINATION ${CMAKE_INSTALL_BINARY} CONFIGURATIONS Debug) 
endif(WIN32)
