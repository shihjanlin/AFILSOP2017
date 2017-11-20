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
#include "SOP_StatusTestGenerator.h"


ADTF_FILTER_PLUGIN("SOP StatusTestGenerator", __guid, SOP_StatusTestGenerator);

cObjectPtr<IMediaTypeDescription> m_pDescription;
cObjectPtr<IMediaSample> ppMediaSample;
cObjectPtr<IMediaSerializer> pSerializer;


SOP_StatusTestGenerator::SOP_StatusTestGenerator(const tChar* __info) :
    QObject(),
    cBaseQtFilter(__info)
{
}

SOP_StatusTestGenerator::~SOP_StatusTestGenerator()
{
}

tHandle SOP_StatusTestGenerator::CreateView()
{
    // create the widget
    QWidget* pWidget = (QWidget*)m_pViewport->VP_GetWindow();
    m_pWidget = new DisplayWidget(pWidget);

    // make the qt connections
    connect(m_pWidget->m_btSendCareStop, SIGNAL(clicked()), this, SLOT(OnCarStope()));
    connect(m_pWidget->m_btSendLaneFollow, SIGNAL(clicked()), this, SLOT(OnLaneFollow()));
    connect(m_pWidget->m_btSendTurnLeft,  SIGNAL(clicked()), this, SLOT(OnTurnLeft()));
    connect(m_pWidget->m_btSendTurnRight,  SIGNAL(clicked()), this, SLOT(OnTurnRight()));
    connect(m_pWidget->m_btSendStraight,  SIGNAL(clicked()), this, SLOT(OnStraight()));
    connect(m_pWidget->m_btSendParking,  SIGNAL(clicked()), this, SLOT(OnParking()));
    connect(m_pWidget->m_btSendPullOutLeft,  SIGNAL(clicked()), this, SLOT(OnPullOutLeft()));
    connect(m_pWidget->m_btSendPullOutRight,  SIGNAL(clicked()), this, SLOT(OnPullOutRight()));


    car_control_flag.ID_set     = tFalse;
    return (tHandle)m_pWidget;
}

tResult SOP_StatusTestGenerator::ReleaseView()
{
    // delete the widget if present
    if (m_pWidget != NULL)
    {
        delete m_pWidget;
        m_pWidget = NULL;
    }
    RETURN_NOERROR;
}

tResult SOP_StatusTestGenerator::Init(tInitStage eStage, __exception)
{
    RETURN_IF_FAILED(cBaseQtFilter::Init(eStage, __exception_ptr));

    if (eStage == StageFirst)
    {
        //get the media description manager for this filter
        cObjectPtr<IMediaDescriptionManager> pDescManager;
        RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER,IID_ADTF_MEDIA_DESCRIPTION_MANAGER,(tVoid**)&pDescManager,__exception_ptr));



        tChar const * strIntValue = pDescManager->GetMediaDescription("tIntSignalValue");
        RETURN_IF_POINTER_NULL(strIntValue);
        cObjectPtr<IMediaType> pTypeIntValue = new cMediaType(0, 0, 0, "tIntSignalValue", strIntValue, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
        RETURN_IF_FAILED(pTypeIntValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&car_control_flag.m_pDescription));
        RETURN_IF_FAILED(car_control_flag.output.Create("CarControlFlag", pTypeIntValue, static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&car_control_flag.output));

        RETURN_NOERROR;
    }
    else if(eStage == StageGraphReady)
    {
        // media descriptions ids not set by now
        m_bIDsBoolValueOutput = tFalse;
    }
    RETURN_NOERROR;
}

tResult SOP_StatusTestGenerator::Start(__exception)
{
    RETURN_IF_FAILED(cBaseQtFilter::Start(__exception_ptr));

    RETURN_NOERROR;
}

tResult SOP_StatusTestGenerator::Run(tInt nActivationCode, const tVoid* pvUserData, tInt szUserDataSize, ucom::IException** __exception_ptr/* =NULL */)
{
    return cBaseQtFilter::Run(nActivationCode, pvUserData, szUserDataSize, __exception_ptr);
}

tResult SOP_StatusTestGenerator::Stop(__exception)
{
    return cBaseQtFilter::Stop(__exception_ptr);
}

tResult SOP_StatusTestGenerator::Shutdown(tInitStage eStage, __exception)
{
    return cBaseQtFilter::Shutdown(eStage, __exception_ptr);
}


void SOP_StatusTestGenerator::OnCarStope()
{
    WriteCarControlFlag(&car_control_flag, CAR_STOP);
}

void SOP_StatusTestGenerator::OnLaneFollow()
{
    WriteCarControlFlag(&car_control_flag, LANE_FOLLOW);
}

void SOP_StatusTestGenerator::OnTurnLeft()
{
    WriteCarControlFlag(&car_control_flag, TURN_LEFT);
}

void SOP_StatusTestGenerator::OnTurnRight()
{
    WriteCarControlFlag(&car_control_flag, TURN_RIGHT);
}

void SOP_StatusTestGenerator::OnStraight()
{
    WriteCarControlFlag(&car_control_flag, STRAIGHT);
}

void SOP_StatusTestGenerator::OnParking()
{
    WriteCarControlFlag(&car_control_flag, PARKING);
}
void SOP_StatusTestGenerator::OnPullOutLeft()
{
    WriteCarControlFlag(&car_control_flag, PULL_OUT_LEFT);
}

void SOP_StatusTestGenerator::OnPullOutRight()
{
    WriteCarControlFlag(&car_control_flag, PULL_OUT_RIGHT);
}



tResult SOP_StatusTestGenerator::WriteCarControlFlag(sop_pin_struct *pin, int value)
{

        AllocMediaSample((tVoid**)&ppMediaSample);
        pin->m_pDescription->GetMediaSampleSerializer(&pSerializer);
        ppMediaSample->AllocBuffer(pSerializer->GetDeserializedSize());
        m_pDescription = pin->m_pDescription;

        {
            __adtf_sample_write_lock_mediadescription(m_pDescription, ppMediaSample, pCoderOutput);

            if(pin->ID_set == tFalse)
            {
                pCoderOutput->GetID("IntValue", pin->ID_value);
                pin->ID_set = tTrue;
            }

            pCoderOutput->Set(pin->ID_value, (tVoid*)&value);

        }

        ppMediaSample->SetTime(_clock->GetStreamTime());
        pin->output.Transmit(ppMediaSample);


    RETURN_NOERROR;
}
