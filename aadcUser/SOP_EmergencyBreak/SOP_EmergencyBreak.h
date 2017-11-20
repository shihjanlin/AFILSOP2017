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
#ifndef _SOP_EMERGENCY_BREAK_H_
#define _SOP_EMERGENCY_BREAK_H_   

#define OID_SOP_EMERGENCY_BREAK_FILTER "adtf.aadc.sop_EmergencyBreak"

enum CAR_STATE {CAR_STOP , LANE_FOLLOW, TURN_LEFT, TURN_RIGHT, STRAIGHT, PARKING, PULL_OUT_LEFT, PULL_OUT_RIGHT};


/*! @defgroup TemplateFilter
*  @{
*
*  \image html User_Template.PNG "Plugin Template Filter"
*
* This is a small template which can be used by the AADC teams for their own filter implementations.
* \b Dependencies \n
* This plugin needs the following libraries:
*
*
* <b> Filter Properties</b>
* <table>
* <tr><th>Property<th>Description<th>Default
* </table>
*
* <b> Output Pins</b>
* <table>
* <tr><th>Pin<th>Description<th>MajorType<th>SubType
* <tr><td>output_template<td>An example output pin<td>MEDIA_TYPE_TEMPLATE<td>MEDIA_TYPE_TEMPLATE
*</table>
*
* <b> Input Pins</b>
* <table>
* <tr><th>Pin<th>Description<th>MajorType<th>SubType
* <tr><td>input_template<td>An example input pin<td>MEDIA_TYPE_TEMPLATE<td>MEDIA_TYPE_TEMPLATE
* </table>
*
* <b>Plugin Details</b>
* <table>
* <tr><td>Path<td>src/aadcUser/AADC_TemplateFilter
* <tr><td>Filename<td>user_templateFilter.plb
* <tr><td>Version<td>1.0.0
* </table>
*
*
*/


typedef struct _sop_pin_struct
{
    cInputPin input;
    cOutputPin output;

    cString   ID_name;
    tBufferID ID_value;
    tBufferID ID_timestamp;

    std::vector<tBufferID> ID_value_array;
    std::vector<tBufferID> ID_Timestamp_array;

    tBool     ID_set;

    cObjectPtr<IMediaTypeDescription> m_pDescription;

} sop_pin_struct;


//!  Template filter for AADC Teams
/*!
* This is a example filter for the AADC
*/
class SOP_EmergencyBreak : public adtf::cFilter
{
    /*! set the filter ID and the version */
    ADTF_FILTER(OID_SOP_EMERGENCY_BREAK_FILTER, "SOP EmergencyBreak", adtf::OBJCAT_DataFilter);

    sop_pin_struct car_control_flag;

    int input_car_state_flag;

    //the input pin for the set point value
    cInputPin m_oInpuSpeedController;

    //Input pin for the ultrasonic front left data
    cInputPin       m_oInputUssStruct;

    //the output pin for the manipulated value
    cOutputPin m_oOutputSpeedController;

private:


protected:

    cObjectPtr<IMediaTypeDescription> m_pDescriptionAccelerateSignalInput;
    tBufferID m_szIDInputSpeedControllerValue;
    tBufferID m_szIDInputSpeedControllerTs;
    tBool     m_szIDInputSpeedControllerSet;


    cObjectPtr<IMediaTypeDescription> m_pDescriptionOutputSpeed;
    tBufferID m_szIDOutputSpeedControllerValue;
    tBufferID m_szIDOutputSpeedControllerTs;
    tBool     m_szIDOutputSpeedControllerSet;


    //descriptor for ultrasonic sensor data
    cObjectPtr<IMediaTypeDescription> m_pDescriptionUsData;
    std::vector<tBufferID> m_szIDUltrasonicF32Value;
    std::vector<tBufferID>  m_szIDUltrasonicArduinoTimestamp;
    //tBufferID m_szIDUltrasonicF32Value;
    //tBufferID m_szIDUltrasonicArduinoTimestamp;
    tBool m_bIDsUltrasonicSet;

    cCriticalSection m_critSetMinUsValue;
    tSignalValue     m_MinUsValue;
    tFloat32 m_MinBreakDistance;


public:
    /*! default constructor for template class
           \param __info   [in] This is the name of the filter instance.
    */
    SOP_EmergencyBreak(const tChar* __info);

    /*! default destructor */
    virtual ~SOP_EmergencyBreak();

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

    tResult CreateInputPins(ucom::IException** __exception_ptr = NULL);
    tResult CreateOutputPins(ucom::IException** __exception_ptr = NULL);
    tResult TransmitSpeed(tFloat32 value, tUInt32 timestamp);
    tResult ProcessUssStructValue(IMediaSample* pMediaSample);

    /*! called if one of the properties is changed
    * \param strProperty the changed property
    * \return standard adtf error code
    */
    tResult PropertyChanged(const char* strProperty);
};

//*************************************************************************************************
#endif // _SOP_NOTBREMSE_H_

/*!
*@}
*/
