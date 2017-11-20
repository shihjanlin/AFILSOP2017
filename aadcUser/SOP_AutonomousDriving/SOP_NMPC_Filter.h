/**
Copyright (c)
Audi Autonomous Driving Cup. All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
3.  All advertising materials mentioning features or use of this software must display the following acknowledgement: “This product includes software developed by the Audi AG and its contributors for Audi Autonomous Driving Cup.”
4.  Neither the name of Audi nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY AUDI AG AND CONTRIBUTORS “AS IS” AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL AUDI AG OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


**********************************************************************
* $Author:: spiesra $  $Date:: 2017-05-12 09:34:53#$ $Rev:: 63109   $
**********************************************************************/
#ifndef _SOP_NMPC_FILTER_H_
#define _SOP_NMPC_FILTER_H_

#include "stdafx.h"
#include "IpIpoptApplication.hpp"

#define OID_SOP_NMPC_FILTER "adtf.aadc.sop_NMPC_Filter"

enum CAR_STATE {CAR_STOP , LANE_FOLLOW};
enum DETECTIONMODE_t {LSEARCH, LTRACE, SL_SEARCH, SL_TRACE};

typedef struct _sop_pin_struct
{
    cInputPin input;
    cOutputPin output;

    cString   ID_name;
    tBufferID ID_value;
    tBufferID ID_timestamp;

    std::vector<tBufferID> ID_value_array;
    std::vector<tBufferID> ID_value_array2;
    std::vector<tBufferID> ID_Timestamp_array;

    tBool     ID_set;

    cObjectPtr<IMediaTypeDescription> m_pDescription;

} sop_pin_struct;

typedef struct _COORDINATE_STRUCT
{
    float X[16];
    float Y[16];

}COORDINATE_STRUCT;

class SOP_NMPC_Controller : public adtf::cTimeTriggeredFilter
{
    /*! set the filter ID and the version */
    ADTF_FILTER(OID_SOP_NMPC_FILTER, "SOP NMPC Controller", adtf::OBJCAT_DataFilter);



private:
    cCriticalSection m_critSecOnPinEvent;
    cCriticalSection m_oSendControlSignal;
    cCriticalSection m_critSecGetData;

protected:


    //Input Signals
    sop_pin_struct car_control_flag;
    sop_pin_struct sampling_rate_trigger;
    //sop_pin_struct reference_input;
    //cString reference_input_ID_name[8];
    sop_pin_struct reference_point;
    sop_pin_struct position_input;
    cString position_input_ID_name[5];


    //Output Signals
    sop_pin_struct steering_output;
    sop_pin_struct speed_output;



    //Parameter
    tFloat32 lane_reference_point[2];
    tFloat32 reference_value[8];
    tFloat32 position_value[5];



    int input_car_state_flag;

    tBool m_bDebugModeEnabled;
    FILE*m_log; // debug file
public:
    /*! default constructor for template class
           \param __info   [in] This is the name of the filter instance.
    */
    SOP_NMPC_Controller(const tChar* __info);

    /*! default destructor */
    virtual ~SOP_NMPC_Controller();

protected:
    /*! Implements the default cFilter state machine call. It will be
    *	    called automatically by changing the filters state and needs
    *	    to be overwritten by the special filter.
    *    Please see page_filter_life_cycle for further information on when the state of a filter changes.
    *
    *    \param [in,out] __exception_ptr   An Exception pointer where exceptions will be put when failed.
    *        If not using the cException smart pointer, the interface has to
    *        be released by calling Unref().
    *    \param  [in] eStage The Init function will be called when the filter state changes as follows:\n
    *    \return Standard Result Code.
    */
    tResult Init(tInitStage eStage, ucom::IException** __exception_ptr);

    /*!
    *   Implements the default cFilter state machine call. It will be
    *   called automatically by changing the filters state and needs
    *   to be overwritten by the special filter.
    *   Please see page_filter_life_cycle for further information on when the state of a filter changes.
    *
    *   \param [in,out] __exception_ptr   An Exception pointer where exceptions will be put when failed.
    *                                   If not using the cException smart pointer, the interface has to
    *                                   be released by calling Unref().
    *   \param  [in] eStage The Init function will be called when the filter state changes as follows:\n   *
    *   \return Returns a standard result code.
    *
    */
    tResult Shutdown(tInitStage eStage, ucom::IException** __exception_ptr = NULL);

    /*! This Function will be called by all pins the filter is registered to.
    *   \param [in] pSource Pointer to the sending pin's IPin interface.
    *   \param [in] nEventCode Event code. For allowed values see IPinEventSink::tPinEventCode
    *   \param [in] nParam1 Optional integer parameter.
    *   \param [in] nParam2 Optional integer parameter.
    *   \param [in] pMediaSample Address of an IMediaSample interface pointers.
    *   \return   Returns a standard result code.
    *   \warning This function will not implement a thread-safe synchronization between the calls from different sources.
    *   You need to synchronize this call by your own. Have a look to adtf_util::__synchronized , adtf_util::__synchronized_obj .
    */
    tResult OnPinEvent(IPin* pSource, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample* pMediaSample);
    tResult Cycle(ucom::IException** __exception_ptr = NULL);
    tResult Start(ucom::IException** __exception_ptr = NULL);

    tResult Stop(ucom::IException** __exception_ptr = NULL);


    //Ipopt
    tResult SetIpopt(void);
    tResult CalculateMPC(void);

    //SOP Function
    tResult CreateInputPins(ucom::IException** __exception_ptr = NULL);
    tResult CreateOutputPins(ucom::IException** __exception_ptr = NULL);
    tResult WriteSignalValue(sop_pin_struct *pin, tFloat32 value, tUInt32 timestamp);
    tResult ReadPinArrayValue(IMediaSample* input_pMediaSample, sop_pin_struct *input_pin, cString *PIN_ID_name, int number_of_array, tFloat32 *output_value);
    tResult ReadReferencePoint(IMediaSample* input_pMediaSample, sop_pin_struct *pin, int number_of_array);



    /*! called if one of the properties is changed
    * \param strProperty the changed property
    * \return standard adtf error code
    */
    tResult PropertyChanged(const char* strProperty);
};



void initialize_bounds();
void collocation_matrix();
void collocation_matrix_2();

//*************************************************************************************************
#endif // _SOP_NOTBREMSE_H_

/*!
*@}
*/
