set(FILTER_NAME SOP_SamplingRate)

# Specify the filter module target
adtf_add_filter(${FILTER_NAME}
                stdafx.h
                SOP_SamplingRate.h
                SOP_SamplingRate.cpp
)

# set folder (only needed and available for MSVC)
adtf_set_folder(${FILTER_NAME} SOP_SamplingRate) 

# Specify where it should be installed to
adtf_install_plugin(${FILTER_NAME} ${CMAKE_INSTALL_BINARY})

