# add the include directory of the filter
include_directories(../include)
# declare our filter
set(FILTER_NAME SOP_EmergencyBreak)


adtf_add_filter(${FILTER_NAME}

    stdafx.h
    SOP_EmergencyBreak.h
    SOP_EmergencyBreak.cpp
)


adtf_set_folder(${FILTER_NAME} SOP_EmergencyBreak)


# Specify where it should be installed to
adtf_install_plugin(${FILTER_NAME} ${CMAKE_INSTALL_BINARY})

