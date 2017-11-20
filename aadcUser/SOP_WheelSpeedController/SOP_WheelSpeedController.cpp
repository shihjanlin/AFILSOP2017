/**
Copyright (c)
Audi Autonomous Driving Cup. All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
3.  All advertising materials mentioning features or use of this software must display the following acknowledgement: ?This product includes software developed by the Audi AG and its contributors for Audi Autonomous Driving Cup.?
4.  Neither the name of Audi nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY AUDI AG AND CONTRIBUTORS ?AS IS? AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL AUDI AG OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


**********************************************************************
* $Author:: spiesra $  $Date:: 2017-05-12 10:01:55#$ $Rev:: 63111   $
**********************************************************************/


// arduinofilter.cpp : Definiert die exportierten Funktionen f?r die DLL-Anwendung.
//
#include "stdafx.h"
#include "SOP_WheelSpeedController.h"


#define MOTOR_F_MAX			-50		//% Gas
#define MOTOR_F_MIN			-5		//% Gas
#define MOTOR_F_GAS_LIMIT	-50		//% Gas
#define MOTOR_F_BRAKE_LIMIT	20		//Linit der Bremse

#define MOTOR_B_MAX   		50		//445 = 15% Gas
#define MOTOR_B_MIN			9		//339 = 100% Gas
#define MOTOR_B_GAS_LIMIT	50		//445 = 40% Gas
#define MOTOR_B_BRAKE_LIMIT	-20		//Linit der Bremse

#define MOTOR_NULL_POINT	0 	//stop
#define MS_TO_PRC_F		7.76	//	45(Max - Min)/ 5.8(Max SPEED)
#define MS_TO_PRC_B		8.91	//	41(%)/ 4.6(KMH)



#define WSC_PROP_PID_KP "PID::Kp_value"
#define WSC_PROP_PID_KI "PID::Ki_value"
#define WSC_PROP_PID_KD "PID::Kd_value"
#define WSC_PROP_PID_SAMPLE_TIME "PID::Sample_Interval_[msec]"

#define SOFT_STARTERS_POSITIVE_MIN_SPEED "Soft_Starters::Posirive_min_speed"
#define SOFT_STARTERS_NEGATIVE_MIN_SPEED "Soft_Starters::Negative_min_speed"
#define SOFT_STARTERS_KP                 "Soft_Starters::Kp_value"



ADTF_FILTER_PLUGIN("SOP Wheel Speed Controller", OID_SOP_WHEELSPEEDCONTROLLER, cSOP_WheelSpeedController)


//create new media sample
cObjectPtr<IMediaSample> pNewMediaSample;
//allocate memory with the size given by the descriptor
cObjectPtr<IMediaSerializer> pSerializer;
tInt nSize;


tFloat64 Error_last = 0;
tFloat64 Error_last_2 = 0;
tFloat Result_last = 0;

tFloat32 f32Value = 0;
tUInt32 Ui32TimeStamp = 0;

tFloat32  outputValue = 0;
tFloat32  outputTimestampe = 0;


cSOP_WheelSpeedController::cSOP_WheelSpeedController(const tChar* __info) : adtf::cTimeTriggeredFilter(__info),  SetPoint(0)
{
    SetPropertyInt("Sampling rate in ms", 10);
    SetPropertyStr("Sampling rate in ms" NSSUBPROP_DESCRIPTION, "Sets the interval in msec");
    SetPropertyInt("Sampling rate in ms" NSSUBPROP_MIN, 10);
    SetPropertyInt("Sampling rate in ms" NSSUBPROP_MAX, 500);



    SetPropertyFloat(WSC_PROP_PID_KP,1.1);
    SetPropertyBool(WSC_PROP_PID_KP NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyStr(WSC_PROP_PID_KP NSSUBPROP_DESCRIPTION, "The proportional factor Kp for the PID Controller");

    SetPropertyFloat(WSC_PROP_PID_KI,0.001);
    SetPropertyBool(WSC_PROP_PID_KI NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyStr(WSC_PROP_PID_KI NSSUBPROP_DESCRIPTION, "The integral factor Ki for the PID Controller");

    SetPropertyFloat(WSC_PROP_PID_KD,1);
    SetPropertyBool(WSC_PROP_PID_KD NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyStr(WSC_PROP_PID_KD NSSUBPROP_DESCRIPTION, "The differential factor Kd for the PID Controller");

    SetPropertyFloat(WSC_PROP_PID_SAMPLE_TIME,0.010);
    SetPropertyBool(WSC_PROP_PID_SAMPLE_TIME NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyStr(WSC_PROP_PID_SAMPLE_TIME NSSUBPROP_DESCRIPTION, "The sample interval in msec used by the PID controller");


    SetPropertyFloat(SOFT_STARTERS_KP,10);
    SetPropertyBool(SOFT_STARTERS_KP NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyStr(SOFT_STARTERS_KP NSSUBPROP_DESCRIPTION, "The differential factor Kp for the Soft Starters");

    SetPropertyFloat(SOFT_STARTERS_NEGATIVE_MIN_SPEED,-0.4);
    SetPropertyBool(SOFT_STARTERS_NEGATIVE_MIN_SPEED NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyStr(SOFT_STARTERS_NEGATIVE_MIN_SPEED NSSUBPROP_DESCRIPTION, "The negative min speed for the Soft Starters");

    SetPropertyFloat(SOFT_STARTERS_POSITIVE_MIN_SPEED,0.4);
    SetPropertyBool(SOFT_STARTERS_POSITIVE_MIN_SPEED NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyStr(SOFT_STARTERS_POSITIVE_MIN_SPEED NSSUBPROP_DESCRIPTION, "The positive min speed for the Soft Starters");

    m_bDebugModeEnabled = tFalse;
    SetPropertyBool("Debug Output to Console", m_bDebugModeEnabled);


    Error_last_2 = 0;
    Error_last = 0;
    Result_last = 0;

    //m_pISignalRegistry = NULL;
}

cSOP_WheelSpeedController::~cSOP_WheelSpeedController()
{
}


tResult cSOP_WheelSpeedController::PropertyChanged(const char* strProperty)
{
    ReadProperties(strProperty);

    RETURN_NOERROR;
}

tResult cSOP_WheelSpeedController::ReadProperties(const tChar* strPropertyName)
{

    if (NULL == strPropertyName || cString::IsEqual(strPropertyName, WSC_PROP_PID_KP))
    {
        PIDKp = GetPropertyFloat(WSC_PROP_PID_KP);
    }

    if (NULL == strPropertyName || cString::IsEqual(strPropertyName, WSC_PROP_PID_KD))
    {
        PIDKd = GetPropertyFloat(WSC_PROP_PID_KD);
    }

    if (NULL == strPropertyName || cString::IsEqual(strPropertyName, WSC_PROP_PID_KI))
    {
        PIDKi = GetPropertyFloat(WSC_PROP_PID_KI);
    }

    if (NULL == strPropertyName || cString::IsEqual(strPropertyName, WSC_PROP_PID_SAMPLE_TIME))
    {
        PIDSampleTime = GetPropertyFloat(WSC_PROP_PID_SAMPLE_TIME);
    }


    if (NULL == strPropertyName || cString::IsEqual(strPropertyName, SOFT_STARTERS_POSITIVE_MIN_SPEED))
    {
        soft_starters_P_min_speed = GetPropertyFloat(SOFT_STARTERS_POSITIVE_MIN_SPEED);
    }

    if (NULL == strPropertyName || cString::IsEqual(strPropertyName, SOFT_STARTERS_NEGATIVE_MIN_SPEED))
    {
        soft_starters_N_min_speed = GetPropertyFloat(SOFT_STARTERS_NEGATIVE_MIN_SPEED);
    }

    if (NULL == strPropertyName || cString::IsEqual(strPropertyName, SOFT_STARTERS_KP))
    {
        soft_starters_Kp = GetPropertyFloat(SOFT_STARTERS_KP);
    }

    LOG_INFO(adtf_util::cString::Format("PIDKp: %.4f; PIDKi: %.4f; PIDKd: %.4f; Positive Speed: %.4f; Negative Speed: %.4f; KP: %.4f",PIDKp, PIDKi, PIDKd, soft_starters_P_min_speed, soft_starters_N_min_speed, soft_starters_Kp));
    RETURN_NOERROR;
}


tResult cSOP_WheelSpeedController::CreateInputPins(__exception)
{
    // create description manager
    cObjectPtr<IMediaDescriptionManager> pDescManager;
    RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER,IID_ADTF_MEDIA_DESCRIPTION_MANAGER,(tVoid**)&pDescManager,__exception_ptr));

  /*  tChar const * strDescBoolSignalValue = pDescManager->GetMediaDescription("tBoolSignalValue");
    RETURN_IF_POINTER_NULL(strDescBoolSignalValue);
    cObjectPtr<IMediaType> pTypeBoolSignalValue = new cMediaType(0, 0, 0, "tBoolSignalValue", strDescBoolSignalValue, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
    RETURN_IF_FAILED(pTypeBoolSignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&sampling_rate_trigger.m_pDescription));

    RETURN_IF_FAILED(sampling_rate_trigger.input.Create("SamplingRateTrigger", pTypeBoolSignalValue, static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&sampling_rate_trigger.input));*/



    // get media tayp
    tChar const * strDescSignalValue = pDescManager->GetMediaDescription("tSignalValue");
    RETURN_IF_POINTER_NULL(strDescSignalValue);
    cObjectPtr<IMediaType> pTypeSignalValue = new cMediaType(0, 0, 0, "tSignalValue", strDescSignalValue,IMediaDescription::MDF_DDL_DEFAULT_VERSION);

    // set member media description
    RETURN_IF_FAILED(pTypeSignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescMeasSpeed));
    RETURN_IF_FAILED(pTypeSignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescSetSpeed));

    // create pins
    RETURN_IF_FAILED(m_oInputSetWheelSpeed.Create("set_WheelSpeed", pTypeSignalValue, static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_oInputSetWheelSpeed));
    RETURN_IF_FAILED(m_oInputMeasWheelSpeed.Create("measured_wheelSpeed", pTypeSignalValue, static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_oInputMeasWheelSpeed));


    RETURN_NOERROR;
}

tResult cSOP_WheelSpeedController::CreateOutputPins(__exception)
{
    // create description manager
    cObjectPtr<IMediaDescriptionManager> pDescManager;
    RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER,IID_ADTF_MEDIA_DESCRIPTION_MANAGER,(tVoid**)&pDescManager,__exception_ptr));

    // get media tayp
    tChar const * strDescSignalValue = pDescManager->GetMediaDescription("tSignalValue");
    RETURN_IF_POINTER_NULL(strDescSignalValue);
    cObjectPtr<IMediaType> pTypeSignalValue = new cMediaType(0, 0, 0, "tSignalValue", strDescSignalValue,IMediaDescription::MDF_DDL_DEFAULT_VERSION);

    // set member media description
    RETURN_IF_FAILED(pTypeSignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescActuator));

    // create pin
    RETURN_IF_FAILED(m_oOutputActuator.Create("actuator_output", pTypeSignalValue, static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_oOutputActuator));


    RETURN_NOERROR;
}

tResult cSOP_WheelSpeedController::Init(tInitStage eStage, __exception)
{
    RETURN_IF_FAILED(cTimeTriggeredFilter::Init(eStage, __exception_ptr))

    if (eStage == StageFirst)
    {
        RETURN_IF_FAILED(CreateInputPins(__exception_ptr));
        RETURN_IF_FAILED(CreateOutputPins(__exception_ptr));
    }
    else if (eStage == StageNormal)
    {
        ReadProperties(NULL);


    }
    else if(eStage == StageGraphReady)
    {
        // set the flags which indicate if the media descriptions strings were set
        m_bInputMeasWheelSpeedGetID = tFalse;
        m_bInputSetWheelSpeedGetID = tFalse;
        m_bInputActuatorGetID = tFalse;

        tUInt32 t = GetPropertyInt("Sampling rate in ms");
        this->SetInterval(t * 1000);  //cycle time 250 ms

    }

    RETURN_NOERROR;
}

tResult cSOP_WheelSpeedController::Start(__exception)
{
    Error_last_2 = 0;
    Error_last = 0;
    Result_last = 0;
    SetPoint = 0;
    accumulatedVariable = 0;
    car_stop_flag = tTrue;

    m_bDebugModeEnabled = GetPropertyBool("Debug Output to Console");

    if (m_bDebugModeEnabled)
     {
         m_log = fopen("PIDLog.txt","w");
     }



    RETURN_IF_FAILED(cTimeTriggeredFilter::Start(__exception_ptr));

    RETURN_NOERROR;
}

tResult cSOP_WheelSpeedController::Stop(__exception)
{
    if (m_bDebugModeEnabled)
        fclose(m_log);

    RETURN_IF_FAILED(cTimeTriggeredFilter::Stop(__exception_ptr));
    RETURN_NOERROR;
}

tResult cSOP_WheelSpeedController::Shutdown(tInitStage eStage, __exception)
{
    RETURN_IF_FAILED(cTimeTriggeredFilter::Shutdown(eStage, __exception_ptr));

//    if (eStage == StageNormal)
//    {

//    }

    RETURN_NOERROR;
}

tResult cSOP_WheelSpeedController::OnPinEvent(    IPin* pSource, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample* pMediaSample)
{
    __synchronized_obj(m_critSecOnPinEvent);
    if (nEventCode == IPinEventSink::PE_MediaSampleReceived && pMediaSample != NULL)
    {
        RETURN_IF_POINTER_NULL( pMediaSample);

        if (pSource == &m_oInputMeasWheelSpeed)
        {
            //write values with zero
            f32Value = 0;
            Ui32TimeStamp = 0;
            {
                // focus for sample write lock
                //read data from the media sample with the coder of the descriptor
                __adtf_sample_read_lock_mediadescription(m_pDescMeasSpeed,pMediaSample,pCoder);

                if(!m_bInputMeasWheelSpeedGetID)
                {
                    pCoder->GetID("f32Value", m_buIDMeasSpeedF32Value);
                    pCoder->GetID("ui32ArduinoTimestamp", m_buIDMeasSpeedArduinoTimestamp);
                    m_bInputMeasWheelSpeedGetID = tTrue;
                }
                //get values from media sample
                pCoder->Get(m_buIDMeasSpeedF32Value, (tVoid*)&f32Value);
                pCoder->Get(m_buIDMeasSpeedArduinoTimestamp, (tVoid*)&Ui32TimeStamp);
            }

            // write to member variable
            MeasuredVariable = f32Value;


        }
        else if (pSource == &m_oInputSetWheelSpeed)
        {
            {
                //write values with zero
                f32Value = 0;
                Ui32TimeStamp = 0;

                // focus for sample write lock
                __adtf_sample_read_lock_mediadescription(m_pDescSetSpeed,pMediaSample,pCoder);

                if(!m_bInputSetWheelSpeedGetID)
                {
                    pCoder->GetID("f32Value", m_buIDSetSpeedF32Value);
                    pCoder->GetID("ui32ArduinoTimestamp", m_buIDSetSpeedArduinoTimestamp);
                    m_bInputSetWheelSpeedGetID = tTrue;
                }

                // read data from the media sample with the coder of the descriptor
                //get values from media sample
                pCoder->Get(m_buIDSetSpeedF32Value, (tVoid*)&f32Value);
                pCoder->Get(m_buIDSetSpeedArduinoTimestamp, (tVoid*)&Ui32TimeStamp);

                // write to member variable
                SetPoint = f32Value;
            }
        }

    }
    RETURN_NOERROR;
}

tResult cSOP_WheelSpeedController::Cycle(__exception)
{

    outputValue = 0;
    outputTimestampe = 0;
    //calculation
    // if speed = 0 is requested output is immediately set to zero
    if (SetPoint == 0 && MeasuredVariable == 0)
    {
        Error_last_2 = 0;
        Error_last = 0;
        Result_last = 0;
        accumulatedVariable = 0;
        car_stop_flag = tTrue;
    }
    else if (SetPoint > 0 )
    {
        outputValue = getControllerValue(MeasuredVariable);
        //LOG_INFO(adtf_util::cString::Format("Output %f",outputValue));
        outputValue = ((outputValue + (SetPoint * MS_TO_PRC_F)));
        //outputValue = (SetPoint * MS_TO_PRC_F);
        outputValue = Limit_Set(-outputValue, MOTOR_F_BRAKE_LIMIT, MOTOR_F_GAS_LIMIT);

    }
    else if (SetPoint < 0 )
    {

        outputValue = getControllerValue(MeasuredVariable);
        outputValue = ((outputValue + (SetPoint * MS_TO_PRC_B)));
        //outputValue = (SetPoint * MS_TO_PRC_B);
        outputValue = Limit_Set(-outputValue, MOTOR_B_GAS_LIMIT, MOTOR_B_BRAKE_LIMIT);

    }

    if(!m_bInputActuatorGetID)
    {
        AllocMediaSample((tVoid**)&pNewMediaSample);
        m_pDescActuator->GetMediaSampleSerializer(&pSerializer);
        nSize = pSerializer->GetDeserializedSize();
        pNewMediaSample->AllocBuffer(nSize);
    }

    {
        // focus for sample write lock
        //read data from the media sample with the coder of the descriptor
        __adtf_sample_write_lock_mediadescription(m_pDescActuator,pNewMediaSample,pCoderOut);

        if(!m_bInputActuatorGetID)
        {
            pCoderOut->GetID("f32Value", m_buIDActuatorF32Value);
            pCoderOut->GetID("ui32ArduinoTimestamp", m_buIDActuatorArduinoTimestamp);
            m_bInputActuatorGetID = tTrue;
        }
        //get values from media sample
        pCoderOut->Set(m_buIDActuatorF32Value, (tVoid*)&outputValue);
        pCoderOut->Set(m_buIDActuatorArduinoTimestamp, (tVoid*)&outputTimestampe);

    }

    //transmit media sample over output pin
    RETURN_IF_FAILED(pNewMediaSample->SetTime(_clock->GetStreamTime()));
    RETURN_IF_FAILED(m_oOutputActuator.Transmit(pNewMediaSample));

    if (m_bDebugModeEnabled)
     {
        if (m_log)
            fprintf(m_log,"%f %f %f\n", SetPoint, MeasuredVariable, outputValue);
     }


    RETURN_NOERROR;
}

tFloat32 cSOP_WheelSpeedController::getControllerValue(tFloat32 MeasuredSpeed)
{
    //Rn = Rn-1 + Kp * (en - en-1) + Ki * (en + en-1)/2 + Kd * (en ? 2*en-1 + en-2)
    //Result_last  = 0;		//Rn-1
    //Error_last   = 0;		//en-1
    //Error_last_2 = 0;		//en-2

    tFloat32 Result = 0;

    //error:
//    tFloat32 Error = (SetPoint - MeasuredSpeed);

/*    if(car_stop_flag == tTrue && MeasuredSpeed >= soft_starters_N_min_speed && MeasuredSpeed <= soft_starters_P_min_speed)
    {
        Result =  soft_starters_Kp * Error;

        //LOG_INFO(adtf_util::cString::Format("Output111111 %f",Result));

        //Error_last_2 = Error_last;
        //Error_last = Error;
    }
    else
    {
        if(car_stop_flag == tTrue)
        {
            car_stop_flag = tFalse;
            Error_last_2 = 0;
            Error_last = 0;
            Result_last = 0;
        }*/


    //    Result =  Result_last +
    //             (PIDKp * (Error - Error_last)) +
    //             (PIDKi * ((Error + Error_last) / 2)) +
    //             (PIDKd * (Error - (2 * Error_last) + Error_last_2));





      /*     if(outputValue < 50 && outputValue >-50)
           {
                Result =  Result_last +
                   (PIDKp * (Error - Error_last)) +
                   (PIDKi * (Error * PIDSampleTime))+
                   (PIDKd * (Error - (2 * Error_last) + Error_last_2)) / PIDSampleTime;
            }
           else
           {
               Result =  Result_last +
                  (PIDKp * (Error - Error_last)) +
                  (0 * (Error * PIDSampleTime))+
                  (PIDKd * (Error - (2 * Error_last) + Error_last_2)) / PIDSampleTime;
           }

        //LOG_INFO(adtf_util::cString::Format("Output2222222222 %f",Result));
//    }

    Error_last_2 = Error_last;
    Error_last = Error;
    Result_last = Result;   */




    //algorithm:
    //esum = esum + e
    //y = Kp * e + Ki * Ta * esum + Kd * (e  ealt)/Ta
    //ealt = e

    //error:
    tFloat32 Error = (SetPoint - MeasuredSpeed);
    // accumulated error:
    if(outputValue < 50 && outputValue >-50)
        accumulatedVariable += Error * PIDSampleTime;


    Result =  PIDKp * Error +
              (PIDKi * accumulatedVariable)+
              (PIDKd * (Error - Error_last) / PIDSampleTime);


    return Result;
}

float Limit_Set(float value, float max, float min)
{
    if(value >= max)
        return max;
    else if(value <= min)
        return min;

    return value;
}
