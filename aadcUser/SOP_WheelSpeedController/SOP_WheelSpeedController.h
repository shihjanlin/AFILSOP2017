#ifndef _SOP_WHEELSPEEDCONTROLLER_H_
#define _SOP_WHEELSPEEDCONTROLLER_H_

#include "stdafx.h"

#define OID_SOP_WHEELSPEEDCONTROLLER "adtf.aadc.sop_wheelSpeedController"



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


/*! the main class for the wheel speed controller plugin */
class cSOP_WheelSpeedController : public cTimeTriggeredFilter
{
    /*! This macro does all the plugin setup stuff
    * Warning: This macro opens a "protected" scope see UCOM_IMPLEMENT_OBJECT_INFO(...) in object.h
    */
    ADTF_DECLARE_FILTER_VERSION(OID_SOP_WHEELSPEEDCONTROLLER, "SOP Wheel Speed Controller", OBJCAT_DataFilter, "Wheel Speed Controller", 1, 1, 0, "");

    /*! the input pin for the measured value */
    cInputPin m_oInputMeasWheelSpeed;

    /*! the input pin for the set point value */
    cInputPin m_oInputSetWheelSpeed;

    /*! the output pin for the manipulated value */
    cOutputPin m_oOutputActuator;

   // sop_pin_struct sampling_rate_trigger;


public:
    /*! constructor for  class
    *    \param __info   [in] This is the name of the filter instance.
    */
    cSOP_WheelSpeedController(const tChar* __info);

    /*! Destructor. */
    virtual ~cSOP_WheelSpeedController();

protected: // overwrites cFilter

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

    /*! Implements the default cFilter state machine calls. It will be
    *    called automatically by changing the filters state IFilter::State_Ready -> IFilter::State_Running
    *    and can be overwritten by the special filter.
    *    \param __exception_ptr  [inout] An Exception pointer where exceptions will be put when failed.
    *        If not using the cException smart pointer, the interface has to
    *        be released by calling Unref().
    *    \return Standard Result Code.
    *    \note This method will be also called during the shutdown of a configuration if the Filter is connected to the Message Bus.
    *    (see:  section_message_bus)! This has to be done, to disconnect the Message Bus and to avoid Race Conditions.
    *
    */
    tResult Start(ucom::IException** __exception_ptr = NULL);

    /*!  Implements the default cFilter state machine calls. It will be
    *   called automatically by changing the filters state IFilter::State_Running -> IFilter::State_Ready
    *   and can be overwritten by the special filter.
    *   \param __exception_ptr  [inout] An Exception pointer where exceptions will be put when failed.
    *   If not using the cException smart pointer, the interface has to
    *   be released by calling Unref().
    *   \return Standard Result Code.
    *   \note This method will be also called during the shutdown of a configuration if the Filter is connected to the Message Bus.
    *   (see: section_message_bus)! This has to be done, to disconnect the Message Bus and to avoid Race Conditions.
    */
    tResult Stop(ucom::IException** __exception_ptr = NULL);

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

private:
    /*! creates all the output Pins
    * \param __exception_ptr the exception pointer
    * \return standard adtf error code
    */
    tResult CreateOutputPins(ucom::IException** __exception_ptr = NULL);

    /*! creates all the input Pins
    * \param __exception_ptr the exception pointer
    * \return standard adtf error code
    */
    tResult CreateInputPins(ucom::IException** __exception_ptr = NULL);

    /*! called if one of the properties is changed
    * \param strProperty the changed property
    * \return standard adtf error code
    */
    tResult PropertyChanged(const char* strProperty);

    /*! calculates the manipulated value for the given values, it uses the setpoint in m_setPoint
    * \param i_f64MeasuredValue    the measuredValue
    * \return the controller output for wheel speed
    */
    tFloat32 getControllerValue(tFloat32 MeasuredSpeed);


    /*! holds the last measuredValue */
    tFloat32 MeasuredVariable;
    /*! holds the last setpoint */
    tFloat32 SetPoint;



    /*! media description for the input pin set speed */
    cObjectPtr<IMediaTypeDescription> m_pDescSetSpeed;
    /*! the id for the f32value of the media description for input pin for the set speed */
    tBufferID m_buIDSetSpeedF32Value;
    /*! the id for the arduino time stamp of the media description for input pin for the set speed */
    tBufferID m_buIDSetSpeedArduinoTimestamp;
    /*! indicates of bufferIDs were set */
    tBool m_bInputSetWheelSpeedGetID;


    /*! media description for the input pin measured speed */
    cObjectPtr<IMediaTypeDescription> m_pDescMeasSpeed;
    /*! the id for the f32value of the media description for input pin for the measured speed */
    tBufferID m_buIDMeasSpeedF32Value;
    /*! the id for the arduino time stamp of the media description for input pin for the measured speed */
    tBufferID m_buIDMeasSpeedArduinoTimestamp;
    /*! indicates of bufferIDs were set */
    tBool m_bInputMeasWheelSpeedGetID;

    /*! the critical section for the on pin events */
    cCriticalSection m_critSecOnPinEvent;

    /*! media description for the output pin with speed */
    cObjectPtr<IMediaTypeDescription> m_pDescActuator;
    /*! the id for the f32value of the media description for input pin for the set speed */
    tBufferID m_buIDActuatorF32Value;
    /*! the id for the arduino time stamp of the media description for input pin for the set speed */
    tBufferID m_buIDActuatorArduinoTimestamp;
    /*! indicates of bufferIDs were set */
    tBool m_bInputActuatorGetID;

    // PID-Controller values
    //
    /*! proportional factor for PID Controller */
    tFloat32    PIDKp;
    /*! integral factor for PID Controller */
    tFloat32    PIDKi;
    /*! differential factor for PID Controller */
    tFloat32    PIDKd;

    /*! the sampletime for the pid controller */
    tFloat32 PIDSampleTime;

    /*! holds the accumulatedVariable for the controller */
    tFloat32 accumulatedVariable;


    tBool car_stop_flag;

    tFloat32    soft_starters_P_min_speed;
    tFloat32    soft_starters_N_min_speed;
    tFloat32    soft_starters_Kp;




    tBool m_bDebugModeEnabled;


    FILE*m_log; // debug file

public: // implements ISignalProvider

        tResult ReadProperties(const tChar* strPropertyName);



public: // implements IObject





};

float Limit_Set(float value, float max, float min);

#endif /** @} */ // end of group // _cWheelSpeedController_H_

