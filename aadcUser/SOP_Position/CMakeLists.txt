set(FILTER_NAME SOP_Position)

# include directories
include_directories(${OpenCV_INCLUDE_DIR})

# Specify the filter module target
adtf_add_filter(${FILTER_NAME}
                stdafx.h
                SOP_Position.h
                SOP_Position.cpp
)

target_link_libraries(${FILTER_NAME} ${OpenCV_LIBS})

# set folder (only needed and available for MSVC)
adtf_set_folder(${FILTER_NAME} SOP_Position)

# Specify where it should be installed to
adtf_install_plugin(${FILTER_NAME} ${CMAKE_INSTALL_BINARY})
