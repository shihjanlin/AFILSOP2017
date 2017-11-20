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
* $Author:: spie#$  $Date:: 2017-05-12 09:34:53#$ $Rev:: 63109   $
**********************************************************************/
#ifndef _SOP_IMAGE_PROCESS_FILTER_HEADER_
#define _SOP_IMAGE_PROCESS_FILTER_HEADER_

#include "stdafx.h"
#include "ADTF_OpenCV_helper.h"



enum IMAGEP_ROCESSING {IMAGE_RUN, IMAGE_STOP};

#define OID_ADTF_FILTER_DEF "adtf.sop_image_process" //unique for a filter
#define ADTF_FILTER_DESC "SOP Image Process"  //this appears in the Component Tree in ADTF
#define ADTF_FILTER_VERSION_SUB_NAME "SopImageProvessFilter"//must match with accepted_version_...
#define ADTF_FILTER_VERSION_ACCEPT_LABEL "accepted_version"//sets the version entry
#define ADTF_FILTER_VERSION_STRING "1.0.0"//version string
#define ADTF_FILTER_VERSION_Major 1//this values will be compared, major version change - will not work
#define ADTF_FILTER_VERSION_Minor 0//change will work but notice
#define ADTF_FILTER_VERSION_Build 0//change will work but notice
//the detailed description of the filter
#define ADTF_FILTER_VERSION_LABEL "A SOP Image Process Filter."

/*! @defgroup OpenCVTemplateFilter
*  @{
*
*  \image html User_OpenCVTemplate.PNG "Plugin OpenCV Template Filter"
* This is a small OpenCV template which can be used by the AADC teams for their own filter implementations for image processing.
*
* This plugin needs the following libraries:
* \li OpenCV  v.3.2.0
*
* <b> Filter Properties</b>
* <table>
* <tr><th>Property<th>Description<th>Default
* </table>
*
* <b>Output Pins</b>
* <table>
* <tr><th>Pin<th>Description<th>MajorType<th>SubType
* <tr><td>Video_Output<td>Video Pin for data from camera<td>MEDIA_TYPE_VIDEO<td>MEDIA_SUBTYPE_VIDEO_UNCOMPRESSED
*</table>
*
* <b>Input Pins</b>
* <table>
* <tr><th>Pin<th>Description<th>MajorType<th>SubType
* <tr><td>Video_Input<td>Video Pin for data from camera<td>MEDIA_TYPE_VIDEO<td>MEDIA_SUBTYPE_VIDEO_UNCOMPRESSED
* </table>
*
* <b>Plugin Details</b>
* <table>
* <tr><td>Path<td>src/aadcUser/AADC_OpenCVTemplate
* <tr><td>Filename<td>user_OpenCVTemplate.plb
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



//!  Template filter for OpenCV Image Processing
/*!
* This class is the main class of the OpenCV Template Filter and can be used as template for user specific image processing filters
*/
class SOP_ImageProcess : public adtf::cFilter
{

    /*! This macro does all the plugin setup stuff */
    ADTF_FILTER_VERSION(OID_ADTF_FILTER_DEF,
                        ADTF_FILTER_DESC,
                        adtf::OBJCAT_Auxiliary,
                        ADTF_FILTER_VERSION_SUB_NAME,
                        ADTF_FILTER_VERSION_Major,
                        ADTF_FILTER_VERSION_Minor,
                        ADTF_FILTER_VERSION_Build,
                        ADTF_FILTER_VERSION_LABEL
                       );

protected:
    /*! input for rgb image */
    cVideoPin           m_oVideoInputPin;

    /*! output for rgb image */
    cVideoPin           m_oVideoOutputPin;

    /*! output for rgb image */
    cVideoPin           m_oVideoEdgeOutputPin;


    sop_pin_struct image_debug;
    sop_pin_struct image_processing_control;
    cString        image_processing_control_ID_name[4];
    tFloat32       image_processing_control_value[4];
    sop_pin_struct lane_model_parameter;
    sop_pin_struct steering_angle;
    cString lane_model_ID_name[11];

    int image_processing_control_flag;

    tFloat32 lane_model[11];


public:
    /*! default constructor for template class
        \param __info   [in] This is the name of the filter instance.
    */
    SOP_ImageProcess(const tChar* __info);

    /*! default destructor */
    virtual ~SOP_ImageProcess();

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
    *   \result Returns a standard result code.
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


private: // private methods

    /*! function to set the m_sProcessFormat and the  m_sInputFormat variables
    *   \param pFormat the new format for the input pin
    *   \return Standard Result Code.
    */
    tResult UpdateInputImageFormat(const tBitmapFormat* pFormat);

    /*! function to set the output image format
    *   \param outputImage the new format for the input pin
    *   \return Standard Result Code.
    */
    tResult UpdateOutputImageFormat(const cv::Mat& outputImage);
    tResult UpdateOutputImageEdgeFormat(const cv::Mat& outputImage);


    /*! function to process the mediasample
    *   \param pSample the new media sample
    *   \return Standard Result Code.
    */
    tResult ProcessVideo(IMediaSample* pSample);

    /*! bitmap format of input pin */
    tBitmapFormat m_sInputFormat;

    /*! bitmap format of output pin */
    tBitmapFormat m_sOutputFormat;
    tBitmapFormat m_sOutputEdgeFormat;

    /*! tha last received input image*/
    Mat m_inputImage;

    cCriticalSection m_critSecOnPinEvent;
    cCriticalSection m_oSendSignal;

    tResult Transfer_YUV_to_YUY2(int Im_width, int Im_height, const cv::Mat& image);
    tResult Transfer_YUY2_to_YUV(int Im_width, int Im_height, cv::Mat& image);
    tResult Transfer_YUY2_to_BGR(int Im_width, int Im_height, cv::Mat& image);
    tResult ImageBufferDownsamplingBGR_to_YUY2(int Im_width, int Im_height, const cv::Mat& image, char type);
    tResult DrawImageEdge(int Im_width, int Im_height, cv::Mat& image);
    tResult SetPinValue(tFloat32 value, tUInt32 timestamp);
    tResult WriteSignalValue(sop_pin_struct *pin, tFloat32 value, tUInt32 timestamp);
    tResult WritePinArrayValue(sop_pin_struct *pin, int number_of_array, cString *ID_name, tFloat32 *value);
    tResult ReadPinArrayValue(IMediaSample* input_pMediaSample, sop_pin_struct *input_pin, cString *PIN_ID_name, int number_of_array, tFloat32 *output_value);


    tResult OPENCV_SVM_TEST();
};

/** @} */ // end of group

#endif  //_OPENCVTEMPLATE_FILTER_HEADER_
