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

#include "stdafx.h"
#include "SOP_EmergencyBreak.h"

/// Create filter shell
ADTF_FILTER_PLUGIN("SOP EmergencyBreak", OID_SOP_EMERGENCY_BREAK_FILTER, SOP_EmergencyBreak);


SOP_EmergencyBreak::SOP_EmergencyBreak(const tChar* __info):cFilter(__info)
{
    SetPropertyFloat("Minimum break distance",9.0);

    m_MinBreakDistance = 9.0;


    m_szIDInputSpeedControllerSet  = tFalse;
    m_szIDOutputSpeedControllerSet = tFalse;
    m_bIDsUltrasonicSet            = tFalse;
}

SOP_EmergencyBreak::~SOP_EmergencyBreak()
{

}

tResult SOP_EmergencyBreak::PropertyChanged(const char* strProperty)
{
    m_MinBreakDistance = static_cast<tFloat32>(GetPropertyFloat("Minimum break distance"));

    RETURN_NOERROR;
}


tResult SOP_EmergencyBreak::Init(tInitStage eStage, __exception)
{
    // never miss calling the parent implementation!!
    RETURN_IF_FAILED(cFilter::Init(eStage, __exception_ptr))

    // in StageFirst you can create and register your static pins.
    if (eStage == StageFirst)
    {
        RETURN_IF_FAILED(CreateInputPins(__exception_ptr));
        RETURN_IF_FAILED(CreateOutputPins(__exception_ptr));

    }
    else if (eStage == StageNormal)
    {
        // In this stage you would do further initialisation and/or create your dynamic pins.
        // Please take a look at the demo_dynamicpin example for further reference.
    }
    else if (eStage == StageGraphReady)
    {
        // All pin connections have been established in this stage so you can query your pins
        // about their media types and additional meta data.
        // Please take a look at the demo_imageproc example for further reference.
    }

    RETURN_NOERROR;
}

tResult SOP_EmergencyBreak::CreateInputPins(__exception)
{
    // create description manager
    cObjectPtr<IMediaDescriptionManager> pDescManager;
    RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER,IID_ADTF_MEDIA_DESCRIPTION_MANAGER,(tVoid**)&pDescManager,__exception_ptr));

    // get media tayp
    tChar const * strDescSignalValue = pDescManager->GetMediaDescription("tSignalValue");
    RETURN_IF_POINTER_NULL(strDescSignalValue);
    cObjectPtr<IMediaType> pTypeSignalValue = new cMediaType(0, 0, 0, "tSignalValue", strDescSignalValue,IMediaDescription::MDF_DDL_DEFAULT_VERSION);


    RETURN_IF_FAILED(pTypeSignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionAccelerateSignalInput));

    // create pins
    RETURN_IF_FAILED(m_oInpuSpeedController.Create("Input Speed", pTypeSignalValue, static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_oInpuSpeedController));




    tChar const * strUltrasonicStruct = pDescManager->GetMediaDescription("tUltrasonicStruct");
    RETURN_IF_POINTER_NULL(strUltrasonicStruct);
    cObjectPtr<IMediaType> pTypeUltrasonicStruct = new cMediaType(0, 0, 0, "tUltrasonicStruct", strUltrasonicStruct, IMediaDescription::MDF_DDL_DEFAULT_VERSION);


    //get mediatype description for ultrasonic sensor data type
    RETURN_IF_FAILED(pTypeUltrasonicStruct->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionUsData));


    //create pins for ultrasonic sensor data
    RETURN_IF_FAILED(m_oInputUssStruct.Create("UltrasonicStruct", pTypeUltrasonicStruct, static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_oInputUssStruct));


    tChar const * strIntValue = pDescManager->GetMediaDescription("tIntSignalValue");
    RETURN_IF_POINTER_NULL(strIntValue);
    cObjectPtr<IMediaType> pTypeIntValue = new cMediaType(0, 0, 0, "tIntSignalValue", strIntValue, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
    RETURN_IF_FAILED(pTypeIntValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&car_control_flag.m_pDescription));
    RETURN_IF_FAILED(car_control_flag.input.Create("CarControlFlag", pTypeIntValue, static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&car_control_flag.input));



    RETURN_NOERROR;
}

tResult SOP_EmergencyBreak::CreateOutputPins(__exception)
{
    // create description manager
    cObjectPtr<IMediaDescriptionManager> pDescManager;
    RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER,IID_ADTF_MEDIA_DESCRIPTION_MANAGER,(tVoid**)&pDescManager,__exception_ptr));

    // get media tayp
    tChar const * strDescSignalValue = pDescManager->GetMediaDescription("tSignalValue");
    RETURN_IF_POINTER_NULL(strDescSignalValue);
    cObjectPtr<IMediaType> pTypeSignalValue = new cMediaType(0, 0, 0, "tSignalValue", strDescSignalValue,IMediaDescription::MDF_DDL_DEFAULT_VERSION);


    RETURN_IF_FAILED(pTypeSignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionOutputSpeed));


    // create pin
    RETURN_IF_FAILED(m_oOutputSpeedController.Create("Output Speed", pTypeSignalValue, static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_oOutputSpeedController));
    RETURN_NOERROR;
}


tResult SOP_EmergencyBreak::Shutdown(tInitStage eStage, __exception)
{
    // In each stage clean up everything that you initiaized in the corresponding stage during Init.
    // Pins are an exception:
    // - The base class takes care of static pins that are members of this class.
    // - Dynamic pins have to be cleaned up in the ReleasePins method, please see the demo_dynamicpin
    //   example for further reference.

    if (eStage == StageGraphReady)
    {

    }
    else if (eStage == StageNormal)
    {
    }
    else if (eStage == StageFirst)
    {
    }

    // call the base class implementation
    return cFilter::Shutdown(eStage, __exception_ptr);
}

tResult SOP_EmergencyBreak::OnPinEvent(IPin* pSource,
                                    tInt nEventCode,
                                    tInt nParam1,
                                    tInt nParam2,
                                    IMediaSample* pMediaSample)
{
     __synchronized_obj(m_critSetMinUsValue);

    // first check what kind of event it is
    if (nEventCode == IPinEventSink::PE_MediaSampleReceived)
    {

        // so we received a media sample, so this pointer better be valid.
        RETURN_IF_POINTER_NULL(pMediaSample);
        //RETURN_IF_POINTER_NULL(pSource);


        if (pSource == &m_oInpuSpeedController)
        {
            //write values with zero
            tFloat32 f32Value = 0;
            tUInt32 ui32TimeStamp = 0;

            {

                // focus for sample write lock
                __adtf_sample_read_lock_mediadescription(m_pDescriptionAccelerateSignalInput,pMediaSample,pCoder);

                // get IDs
                if(!m_szIDInputSpeedControllerSet)
                {
                    pCoder->GetID("f32Value", m_szIDInputSpeedControllerValue);
                    pCoder->GetID("ui32ArduinoTimestamp", m_szIDInputSpeedControllerTs);
                    m_szIDInputSpeedControllerSet = tTrue;
                }

                // read data from the media sample with the coder of the descriptor
                //get values from media sample
                pCoder->Get(m_szIDInputSpeedControllerValue, (tVoid*)&f32Value);
                pCoder->Get(m_szIDInputSpeedControllerTs, (tVoid*)&ui32TimeStamp);

            }

            {

                if(input_car_state_flag == CAR_STOP)
                {
                   TransmitSpeed(0, ui32TimeStamp);
                }
                else
                {
                    TransmitSpeed(f32Value, ui32TimeStamp);
                }

                //use mutex
            /*


                if(m_MinUsValue.f32Value < m_MinBreakDistance || input_car_state_flag == CAR_STOP)
                {
                    TransmitSpeed(0, ui32TimeStamp);
                }
                else
                {
                    TransmitSpeed(f32Value, ui32TimeStamp);
                }*/

            }


        }
        else if (pSource == &car_control_flag.input)
        {
            {
                cObjectPtr<IMediaTypeDescription> m_pDescription;
                m_pDescription = car_control_flag.m_pDescription;
                // focus for sample write lock
                __adtf_sample_read_lock_mediadescription(m_pDescription, pMediaSample, pCoder);


                if(car_control_flag.ID_set == tFalse)
                {
                    pCoder->GetID("IntValue", car_control_flag.ID_value);
                    car_control_flag.ID_set = tTrue;
                }
                pCoder->Get(car_control_flag.ID_value, (tVoid*)&input_car_state_flag);
            }
            static int temp_flag = input_car_state_flag;
            if(input_car_state_flag != temp_flag)
            {
                LOG_INFO(adtf_util::cString::Format("(EmergencyBreak) State Flag Input %d",input_car_state_flag));
                temp_flag = input_car_state_flag;
            }


        }
        else if (pSource == &m_oInputUssStruct)
        {
            RETURN_IF_FAILED(ProcessUssStructValue(pMediaSample));
        }
    }

    RETURN_NOERROR;
}


tResult SOP_EmergencyBreak::ProcessUssStructValue(IMediaSample* pMediaSample)
{
    //use mutex
    __synchronized_obj(m_critSetMinUsValue);

    //write values with zero
    tFloat32 &MinValue = m_MinUsValue.f32Value;
    tUInt32  &MinTimeStamp = m_MinUsValue.ui32ArduinoTimestamp;

    tFloat32 buf_Value = 0;
    tUInt32 buf_TimeStamp = 0;

    //init
    MinValue = 400;
    MinTimeStamp = 0;

    int index = 0;

    {

        // focus for sample write lock
        __adtf_sample_read_lock_mediadescription(m_pDescriptionUsData, pMediaSample, pCoder);

        // get IDs
        if(!m_bIDsUltrasonicSet)
        {
            tBufferID idValue, idTimestamp;
            m_szIDUltrasonicF32Value.clear();
            m_szIDUltrasonicArduinoTimestamp.clear();

            pCoder->GetID("tFrontLeft.f32Value", idValue);
            pCoder->GetID("tFrontLeft.ui32ArduinoTimestamp", idTimestamp);
            m_szIDUltrasonicF32Value.push_back(idValue);
            m_szIDUltrasonicArduinoTimestamp.push_back(idTimestamp);

            pCoder->GetID("tFrontCenterLeft.f32Value", idValue);
            pCoder->GetID("tFrontCenterLeft.ui32ArduinoTimestamp", idTimestamp);
            m_szIDUltrasonicF32Value.push_back(idValue);
            m_szIDUltrasonicArduinoTimestamp.push_back(idTimestamp);

            pCoder->GetID("tFrontCenter.f32Value", idValue);
            pCoder->GetID("tFrontCenter.ui32ArduinoTimestamp", idTimestamp);
            m_szIDUltrasonicF32Value.push_back(idValue);
            m_szIDUltrasonicArduinoTimestamp.push_back(idTimestamp);

            pCoder->GetID("tFrontCenterRight.f32Value", idValue);
            pCoder->GetID("tFrontCenterRight.ui32ArduinoTimestamp", idTimestamp);
            m_szIDUltrasonicF32Value.push_back(idValue);
            m_szIDUltrasonicArduinoTimestamp.push_back(idTimestamp);

            pCoder->GetID("tFrontRight.f32Value", idValue);
            pCoder->GetID("tFrontRight.ui32ArduinoTimestamp", idTimestamp);
            m_szIDUltrasonicF32Value.push_back(idValue);
            m_szIDUltrasonicArduinoTimestamp.push_back(idTimestamp);

            pCoder->GetID("tSideLeft.f32Value", idValue);
            pCoder->GetID("tSideLeft.ui32ArduinoTimestamp", idTimestamp);
            m_szIDUltrasonicF32Value.push_back(idValue);
            m_szIDUltrasonicArduinoTimestamp.push_back(idTimestamp);

            pCoder->GetID("tSideRight.f32Value", idValue);
            pCoder->GetID("tSideRight.ui32ArduinoTimestamp", idTimestamp);
            m_szIDUltrasonicF32Value.push_back(idValue);
            m_szIDUltrasonicArduinoTimestamp.push_back(idTimestamp);

            pCoder->GetID("tRearLeft.f32Value", idValue);
            pCoder->GetID("tRearLeft.ui32ArduinoTimestamp", idTimestamp);
            m_szIDUltrasonicF32Value.push_back(idValue);
            m_szIDUltrasonicArduinoTimestamp.push_back(idTimestamp);

            pCoder->GetID("tRearCenter.f32Value", idValue);
            pCoder->GetID("tRearCenter.ui32ArduinoTimestamp", idTimestamp);
            m_szIDUltrasonicF32Value.push_back(idValue);
            m_szIDUltrasonicArduinoTimestamp.push_back(idTimestamp);

            pCoder->GetID("tRearRight.f32Value", idValue);
            pCoder->GetID("tRearRight.ui32ArduinoTimestamp", idTimestamp);
            m_szIDUltrasonicF32Value.push_back(idValue);
            m_szIDUltrasonicArduinoTimestamp.push_back(idTimestamp);

            m_bIDsUltrasonicSet = tTrue;
        }

        // read data from the media sample with the coder of the descriptor
        //get values from media sample
        for(index = 0; index < (int)m_szIDUltrasonicF32Value.size(); index++)
        {
            pCoder->Get(m_szIDUltrasonicF32Value[index], (tVoid*)&buf_Value);
            if(buf_Value <= MinValue && buf_Value > 0)
            {
                pCoder->Get(m_szIDUltrasonicArduinoTimestamp[index], (tVoid*)&buf_TimeStamp);

                if(index == 5 || index == 6)
                {

                }
                else
                {
                    //Save the Values
                    MinValue = buf_Value;
                    MinTimeStamp = buf_TimeStamp;
                }
            }

        }

    }

    RETURN_NOERROR;
}

tResult SOP_EmergencyBreak::TransmitSpeed(tFloat32 speed, tUInt32 timestamp)
{
    //use mutex
    //__synchronized_obj(m_critSecTransmitControl);

    cObjectPtr<IMediaSample> pMediaSample;
    AllocMediaSample((tVoid**)&pMediaSample);

    cObjectPtr<IMediaSerializer> pSerializer;
    m_pDescriptionOutputSpeed->GetMediaSampleSerializer(&pSerializer);
    pMediaSample->AllocBuffer(pSerializer->GetDeserializedSize());

    {
        __adtf_sample_write_lock_mediadescription(m_pDescriptionOutputSpeed, pMediaSample, pCoderOutput);

        if(!m_szIDOutputSpeedControllerSet)
        {
            pCoderOutput->GetID("f32Value", m_szIDOutputSpeedControllerValue);
            pCoderOutput->GetID("ui32ArduinoTimestamp", m_szIDOutputSpeedControllerTs);
            m_szIDOutputSpeedControllerSet = tTrue;
        }

        pCoderOutput->Set(m_szIDOutputSpeedControllerValue, (tVoid*)&speed);
        pCoderOutput->Set(m_szIDOutputSpeedControllerTs, (tVoid*)&timestamp);
    }

    pMediaSample->SetTime(_clock->GetStreamTime());

    m_oOutputSpeedController.Transmit(pMediaSample);

    RETURN_NOERROR;
}

