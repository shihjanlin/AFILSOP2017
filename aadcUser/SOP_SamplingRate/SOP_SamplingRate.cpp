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
* $Author:: spiesra $  $Date:: 2017-05-12 09:39:39#$ $Rev:: 63110   $
**********************************************************************/

#include "stdafx.h"
#include "SOP_SamplingRate.h"

ADTF_FILTER_PLUGIN("SOP Sampling Rate Trigger", OID_ADTF_SAMPLINGRATE_FILTER, SOP_SamplingRate)

SOP_SamplingRate::SOP_SamplingRate(const tChar* __info) : adtf::cTimeTriggeredFilter(__info)
{
    SetPropertyInt("Sampling rate in ms", 250);
    SetPropertyStr("Sampling rate in ms" NSSUBPROP_DESCRIPTION, "Sets the interval between two sent Sampling Rate triggers in msec");
    SetPropertyInt("Sampling rate in ms" NSSUBPROP_MIN, 10);
    SetPropertyInt("Sampling rate in ms" NSSUBPROP_MAX, 500);
}

SOP_SamplingRate::~SOP_SamplingRate()
{
}

tResult SOP_SamplingRate::CreateOutputPins(__exception)
{
    cObjectPtr<IMediaDescriptionManager> pDescManager;
    RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER, IID_ADTF_MEDIA_DESCRIPTION_MANAGER, (tVoid**)&pDescManager, __exception_ptr));

    tChar const * strDescBoolSignalValue = pDescManager->GetMediaDescription("tBoolSignalValue");
    RETURN_IF_POINTER_NULL(strDescBoolSignalValue);
    cObjectPtr<IMediaType> pTypeBoolSignalValue = new cMediaType(0, 0, 0, "tBoolSignalValue", strDescBoolSignalValue, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
    RETURN_IF_FAILED(pTypeBoolSignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pCoderDescOutput));

    RETURN_IF_FAILED(m_oOutputPin.Create("TriggerAliveSignal", pTypeBoolSignalValue, static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_oOutputPin));

    RETURN_NOERROR;
}

tResult SOP_SamplingRate::Init(tInitStage eStage, __exception)
{
    RETURN_IF_FAILED(cFilter::Init(eStage, __exception_ptr))

    if (eStage == StageFirst)
    {
        tResult nResult = CreateOutputPins(__exception_ptr);
        if (IS_FAILED(nResult))
        {
            THROW_ERROR_DESC(nResult, "Failed to create Output Pins");
        }

    }
    else if (eStage == StageNormal)
    {
        tUInt32 t = GetPropertyInt("Sampling rate in ms");
        this->SetInterval(t * 1000);  //cycle time 250 ms

    }
    else if (eStage == StageGraphReady)
    {
        // no ids were set so far
        m_bIDsBoolSet = tFalse;
    }
    RETURN_NOERROR;
}

tResult SOP_SamplingRate::Cycle(__exception)
{
    // __synchronized_obj(m_critSecOnPinEvent);
    //write values with zero
    tBool value = tTrue;
    tUInt32 timeStamp = 0;

    //create new media sample
    cObjectPtr<IMediaSample> pMediaSample;
    AllocMediaSample((tVoid**)&pMediaSample);

    //allocate memory with the size given by the descriptor
    cObjectPtr<IMediaSerializer> pSerializer;
    m_pCoderDescOutput->GetMediaSampleSerializer(&pSerializer);
    tInt nSize = pSerializer->GetDeserializedSize();
    pMediaSample->AllocBuffer(nSize);

    {
        // focus for sample write lock
        //write date to the media sample with the coder of the descriptor
        __adtf_sample_write_lock_mediadescription(m_pCoderDescOutput, pMediaSample, pCoderOutput);

        // set the ids if not already done
        if (!m_bIDsBoolSet)
        {
            pCoderOutput->GetID("bValue", m_szIDbValue);
            pCoderOutput->GetID("ui32ArduinoTimestamp", m_szIDArduinoTimestamp);
            m_bIDsBoolSet = tTrue;
        }
        timeStamp = adtf_util::cHighResTimer::GetTime();//_clock->GetStreamTime();
        //get values from media sample
        pCoderOutput->Set(m_szIDbValue, (tVoid*)&(value));
        pCoderOutput->Set(m_szIDArduinoTimestamp, (tVoid*)&timeStamp);
    }

    //transmit media sample over output pin
    pMediaSample->SetTime(_clock->GetStreamTime());
    m_oOutputPin.Transmit(pMediaSample);


    RETURN_NOERROR;
}








