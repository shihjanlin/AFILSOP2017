set(FILTER_NAME SOP_wheelSpeedController)
 
# Specify the filter module target
adtf_add_filter(${FILTER_NAME}
                stdafx.h
                SOP_WheelSpeedController.h
                SOP_WheelSpeedController.cpp
)

# set folder (only needed and available for MSVC)
adtf_set_folder(${FILTER_NAME} SOP_wheelSpeedController) 

# Specify where it should be installed to
adtf_install_plugin(${FILTER_NAME} ${CMAKE_INSTALL_BINARY})



