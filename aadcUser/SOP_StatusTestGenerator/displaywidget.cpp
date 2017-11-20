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
#include "displaywidget.h"



DisplayWidget::DisplayWidget(QWidget* parent) : QWidget(parent)
{
    m_pWidget = new QWidget(this);
    m_pWidget->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);

    m_btSendCareStop = new QPushButton(this);
    m_btSendCareStop->setText("Car Stop");
    m_btSendCareStop->setFixedSize(200,50);


    m_btSendLaneFollow = new QPushButton(this);
    m_btSendLaneFollow->setText("Start");
    m_btSendLaneFollow->setFixedSize(200,50);


    m_btSendTurnLeft = new QPushButton(this);
    m_btSendTurnLeft->setText("Turn Left");
    m_btSendTurnLeft->setFixedSize(200,50);

    m_btSendTurnRight = new QPushButton(this);
    m_btSendTurnRight->setText("Turn Right");
    m_btSendTurnRight->setFixedSize(200,50);

    m_btSendStraight = new QPushButton(this);
    m_btSendStraight->setText("Straight");
    m_btSendStraight->setFixedSize(200,50);

    m_btSendParking = new QPushButton(this);
    m_btSendParking->setText("Parking");
    m_btSendParking->setFixedSize(200,50);

    m_btSendPullOutLeft = new QPushButton(this);
    m_btSendPullOutLeft->setText("Pull Out Left");
    m_btSendPullOutLeft->setFixedSize(200,50);

    m_btSendPullOutRight = new QPushButton(this);
    m_btSendPullOutRight->setText("Pull Out Right");
    m_btSendPullOutRight->setFixedSize(200,50);


    m_mainLayout = new QVBoxLayout();
    m_mainLayout->addWidget(m_btSendCareStop, 0,Qt::AlignCenter);
    m_mainLayout->addWidget(m_btSendLaneFollow,0,Qt::AlignCenter);
    m_mainLayout->addWidget(m_btSendTurnLeft, 0,Qt::AlignCenter);
    m_mainLayout->addWidget(m_btSendTurnRight, 0,Qt::AlignCenter);
    m_mainLayout->addWidget(m_btSendStraight, 0,Qt::AlignCenter);
    m_mainLayout->addWidget(m_btSendParking, 0,Qt::AlignCenter);
    m_mainLayout->addWidget(m_btSendPullOutLeft, 0,Qt::AlignCenter);
    m_mainLayout->addWidget(m_btSendPullOutRight, 0,Qt::AlignCenter);
    setLayout(m_mainLayout);

    connect(m_btSendCareStop,  SIGNAL(clicked()), this, SLOT(sendCarStop()));
    connect(m_btSendLaneFollow,  SIGNAL(clicked()), this, SLOT(sendLaneFollow()));
    connect(m_btSendTurnLeft,  SIGNAL(clicked()), this, SLOT(sendTurnLeft()));
    connect(m_btSendTurnRight,  SIGNAL(clicked()), this, SLOT(sendTurnRight()));
    connect(m_btSendStraight,  SIGNAL(clicked()), this, SLOT(sendStraight()));
    connect(m_btSendParking,  SIGNAL(clicked()), this, SLOT(sendParking()));
    connect(m_btSendPullOutLeft,  SIGNAL(clicked()), this, SLOT(sendPullOutLeft()));
    connect(m_btSendPullOutRight,  SIGNAL(clicked()), this, SLOT(sendPullOutRight()));
}








