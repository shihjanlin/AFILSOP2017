/**********************************************************************
Copyright (c)
Audi Autonomous Driving Cup. All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
3.  All advertising materials mentioning features or use of this software must display the following acknowledgement: “This product includes software developed by the Audi AG and its contributors for Audi Autonomous Driving Cup.”
4.  Neither the name of Audi nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY AUDI AG AND CONTRIBUTORS “AS IS” AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL AUDI AG OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


**********************************************************************
* $Author:: spie#$  $Date:: 2017-05-12 09:39:39#$ $Rev:: 63110   $
**********************************************************************/



#ifndef _STATUS_TEST_HEADER
#define _STATUS_TEST_HEADER

#define __guid "adtf.aadc.status_test"

#include "stdafx.h"
#include "displaywidget.h"

enum CAR_STATE {CAR_STOP , LANE_FOLLOW, TURN_LEFT, TURN_RIGHT, STRAIGHT, PARKING, PULL_OUT_LEFT, PULL_OUT_RIGHT};


/*! @defgroup BoolValueGenerator Bool Value Generator
*  @{
*
* With this small helper simple Media Samples of the tBoolSignalValue can be generated. When started a GUI with two buttons is shown. When clicking on “Send Value FALSE” a Media Sample with “False” in the media description element bValue is transmitted, when clicking on “Send Value TRUE” a Media Sample with “True” is transmitted.
*
*  \image html BoolValueGenerator.PNG "Plugin Bool Value Generator"
*
* \b Dependencies \n
* This plugin needs the following libraries:
* \li QT  v.4.7.1
*
* <b> Filter Properties</b>
* <table>
* <tr><th>Property<th>Description<th>Default
* </table>
*
* <b> Output Pins</b>
* <table>
* <tr><th>Pin<th>Description<th>Media Description
* <tr><td>BoolValue<td>Output for the generated Bool Value<td>tBoolSignalValue
*</table>
*
* <b> Input Pins</b>
* <table>
* <tr><th>Pin<th>Description<th>MajorType<th>SubTyp
* </table>
*
* <b>Plugin Details</b>
* <table>
* <tr><td>Path<td>src/aadcDemo/helper/AADC_BoolValueGenerator
* <tr><td>Filename<td>aadc_boolValueGenerator.plb
* <tr><td>Version<td>1.0.0
* </table>
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

/*! This is the main class of the BoolValue Generator Plugin */
class SOP_StatusTestGenerator : public QObject, public cBaseQtFilter
{
    /*! set the filter ID and version */
    ADTF_DECLARE_FILTER_VERSION(__guid, "SOP StatusTestGenerator", OBJCAT_Auxiliary, "Status Generator Filter", 1, 0, 0, "");

    Q_OBJECT


public:
    /*! constructor for template class
    *    \param __info   [in] This is the name of the filter instance.
    */
    SOP_StatusTestGenerator(const tChar* __info);

    /*! default destructor */
    virtual ~SOP_StatusTestGenerator();

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
    virtual tResult Init(tInitStage eStage, ucom::IException** __exception_ptr = NULL);

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
    virtual tResult Start(ucom::IException** __exception_ptr = NULL);

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
    virtual tResult Stop(ucom::IException** __exception_ptr = NULL);

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
    *   \result Returns a standard result code.
    *
    */
    virtual tResult Shutdown(tInitStage eStage, ucom::IException** __exception_ptr = NULL);

    /*!
    *  The adtf::IRunnable interface declares an object as a callback-entry point.
    *  Overwrite these function when handling system events, thread calls or other
    *  activation calls. Have a look to adtf::IRunnable and  adtf::IKernel.
    *  \n
    *  \n For receiving System Events use within your overwritten cFilter::Init function:
    *  \n
    *  \code _kernel->SignalRegister(static_cast<IRunnable*>(this)); \endcode
    *  \n Your nActivationCode for the Run call will be IRunnable::RUN_SIGNAL. The
    *     pvUserData will be a pointer to adtf::tEventInfo.
    *  \warning IMPORTANT: When registered for signals, never forget to unregister:
    *  \code _kernel->SignalUnregister(static_cast<IRunnable*>(this)); \endcode
    *
    * \param [in] nActivationCode The activation type for running.
    * \param [in] pvUserData pointer to a activation structure depending on the activation type.
    * \param [in] szUserDataSize Size of the activation structure. (in byte)
    * \param [in,out] __exception_ptr Address of variable that points to an IException interface.
    *                                    If not using the cException smart pointer, the interface has to
    *                                    be released by calling Unref()..
    *
    * \result Returns a standard result code.
    */
    tResult Run(tInt nActivationCode, const tVoid* pvUserData, tInt szUserDataSize, ucom::IException** __exception_ptr = NULL);


protected: // Implement cBaseQtFilter

    /*! Creates the widget instance
    * \result Returns a standard result code.
    */
    tHandle CreateView();

    /*! Destroys the widget instance
    * \result Returns a standard result code.
    */
    tResult ReleaseView();


private:

    /*! The displayed widget*/
    DisplayWidget *m_pWidget;

    /*! Coder Descriptor */
    cObjectPtr<IMediaTypeDescription> m_pDescriptionBool;
    /*! the id for the bool value output of the media description */
    tBufferID m_szIDBoolValueOutput;
    /*! the id for the arduino timestamp output of the media description */
    tBufferID m_szIDArduinoTimestampOutput;
    /*! indicates if bufferIDs were set */
    tBool m_bIDsBoolValueOutput;

    /*! input pin for the run command*/
    cOutputPin        m_oBoolValuePin;


    sop_pin_struct car_control_flag;


public slots:
    /*! transmits a new mediasample with value false */
    void OnCarStope();

    /*! transmits a new mediasample with value true */
    void OnLaneFollow();
    void OnTurnLeft();
    void OnTurnRight();
    void OnStraight();
    void OnParking();
    void OnPullOutLeft();
    void OnPullOutRight();


    tResult WriteCarControlFlag(sop_pin_struct *pin, int value);

};

#endif /** @} */ // end of group
