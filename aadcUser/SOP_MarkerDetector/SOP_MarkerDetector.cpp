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
* $Author:: kuckal  $  $Date:: 2017-05-22 10:14:17#$ $Rev:: 63667   $
**********************************************************************/

#include "stdafx.h"
#include "aadc_roadSign_enums.h"
#include "SOP_MarkerDetector.h"


ADTF_FILTER_PLUGIN("SOP Marker Detector Plugin", OID_ADTF_MARKERDETECTFILTER, SOP_MarkerDetector)




SOP_MarkerDetector::SOP_MarkerDetector(const tChar* __info) :cFilter(__info)
{

    SetPropertyBool("Debug Output to Console", tFalse);
    SetPropertyStr("Debug Output to Console" NSSUBPROP_DESCRIPTION, "If enabled additional debug information is printed to the console (Warning: decreases performance).");

    SetPropertyStr("Calibration File for used Camera", "");
    SetPropertyBool("Calibration File for used Camera" NSSUBPROP_FILENAME, tTrue);
    SetPropertyStr("Calibration File for used Camera" NSSUBPROP_FILENAME NSSUBSUBPROP_EXTENSIONFILTER, "YML Files (*.yml)");
    SetPropertyStr("Calibration File for used Camera" NSSUBPROP_DESCRIPTION, "Here you have to set the file with calibration paraemters of the used camera");

    SetPropertyStr("Detector Paramater File", "");
    SetPropertyBool("Detector Paramater File" NSSUBPROP_FILENAME, tTrue);
    SetPropertyStr("Detector Paramater File" NSSUBPROP_FILENAME NSSUBSUBPROP_EXTENSIONFILTER, "YML Files (*.yml)");
    SetPropertyStr("Detector Paramater File" NSSUBPROP_DESCRIPTION, "Here you have to set the file with the parameters with the detector params");

    SetPropertyFloat("Size of Markers", 0.117f);
    SetPropertyStr("Size of Markers" NSSUBPROP_DESCRIPTION, "Size (length of one side) of markers in m");


    SetPropertyInt("ROI::XOffset", 600);
    SetPropertyStr("ROI::XOffset" NSSUBPROP_DESCRIPTION, "X Offset for Region of Interest Rectangular");

    SetPropertyInt("ROI::YOffset", 310);
    SetPropertyStr("ROI::YOffset" NSSUBPROP_DESCRIPTION, "Y Offset for Region of Interest Rectangular");

    SetPropertyInt("ROI::Width", 640);
    SetPropertyStr("ROI::Width" NSSUBPROP_DESCRIPTION, "Width of the Region of Interest Rectangular");

    SetPropertyInt("ROI::Height", 300);
    SetPropertyStr("ROI::Height" NSSUBPROP_DESCRIPTION, "Height of the Region of Interest Rectangular");

    SetPropertyBool("ROI::ROI ON/OFF", false);
    SetPropertyStr("ROI::ROI ON/OFF" NSSUBPROP_DESCRIPTION, "ROI ON/OFF");
    SetPropertyBool("ROI::ROI ON/OFF" NSSUBPROP_ISCHANGEABLE, tTrue);



	UCOM_REGISTER_TIMING_SPOT(cString(OIGetInstanceName()) + "::Process::Start", m_oProcessStart);
    UCOM_REGISTER_TIMING_SPOT(cString(OIGetInstanceName()) + "::Process::End", m_oProcessEnd);
	UCOM_REGISTER_TIMING_SPOT(cString(OIGetInstanceName()) + "::MarkerDetection::Start", m_oPreMarkerDetect);
	UCOM_REGISTER_TIMING_SPOT(cString(OIGetInstanceName()) + "::MarkerDetection::End", m_oPostMarkerDetect);
	UCOM_REGISTER_TIMING_SPOT(cString(OIGetInstanceName()) + "::PoseEstimation::Start", m_oPrePoseEstimation);
	UCOM_REGISTER_TIMING_SPOT(cString(OIGetInstanceName()) + "::PoseEstimation::End", m_oPostPoseEstimation);
}

SOP_MarkerDetector::~SOP_MarkerDetector()
{
}

tResult SOP_MarkerDetector::Init(tInitStage eStage, __exception)
{
    RETURN_IF_FAILED(cFilter::Init(eStage, __exception_ptr));
    if (eStage == StageFirst)
    {
        //create the video rgb input pin
        RETURN_IF_FAILED(m_oPinInputVideo.Create("Video_RGB_input", IPin::PD_Input, static_cast<IPinEventSink*>(this)));
        RETURN_IF_FAILED(RegisterPin(&m_oPinInputVideo));

        //create the video rgb output pin
        RETURN_IF_FAILED(m_oPinOutputVideo.Create("Video_RGB_output", IPin::PD_Output, static_cast<IPinEventSink*>(this)));
        RETURN_IF_FAILED(RegisterPin(&m_oPinOutputVideo));

        // create the description manager
        cObjectPtr<IMediaDescriptionManager> pDescManager;
        RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER, IID_ADTF_MEDIA_DESCRIPTION_MANAGER, (tVoid**)&pDescManager, __exception_ptr));

//        // create the description for the road sign pin
//        tChar const * strDesc = pDescManager->GetMediaDescription("tRoadSign");
//        RETURN_IF_POINTER_NULL(strDesc);
//        cObjectPtr<IMediaType> pType = new cMediaType(0, 0, 0, "tRoadSign", strDesc, IMediaDescription::MDF_DDL_DEFAULT_VERSION);

//        // create the road sign OutputPin
//        RETURN_IF_FAILED(m_oPinRoadSign.Create("RoadSign", pType, this));
//        RETURN_IF_FAILED(RegisterPin(&m_oPinRoadSign));
//        // set the description for the road sign pin
//        RETURN_IF_FAILED(pType->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionRoadSign));

        // create the description for the road sign pin
        tChar const * strDescExt = pDescManager->GetMediaDescription("tRoadSignExt");
        RETURN_IF_POINTER_NULL(strDescExt);
        cObjectPtr<IMediaType> pTypeExt = new cMediaType(0, 0, 0, "tRoadSignExt", strDescExt, IMediaDescription::MDF_DDL_DEFAULT_VERSION);

        // create the extended road sign OutputPin
        RETURN_IF_FAILED(m_oPinRoadSignExt.Create("RoadSign_ext", pTypeExt, this));
        RETURN_IF_FAILED(RegisterPin(&m_oPinRoadSignExt));
        // set the description for the extended road sign pin
        RETURN_IF_FAILED(pTypeExt->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionRoadSignExt));

//        //*** create and register the output pin 1***//
//        cObjectPtr<IMediaType> pTypeOut_GCL;
//        RETURN_IF_FAILED(AllocMediaType((tVoid**)&pTypeOut_GCL,
//                                        MEDIA_TYPE_COMMAND,
//                                        MEDIA_SUBTYPE_COMMAND_GCL,
//                                        NULL, NULL, __exception_ptr));
//        RETURN_IF_FAILED(m_outputPinGCL.Create("GCL_Markers", pTypeOut_GCL, static_cast<IPinEventSink*> (this)));
//        RETURN_IF_FAILED(RegisterPin(&m_outputPinGCL));

    }
    else if (eStage == StageNormal)
    {
        ROIWidth = GetPropertyInt("ROI::Width");
        ROIHeight = GetPropertyInt("ROI::Height");
        ROIOffsetX = GetPropertyInt("ROI::XOffset");
        ROIOffsetY = GetPropertyInt("ROI::YOffset");
        ROI_ON_OFF = GetPropertyBool("ROI::ROI ON/OFF");



        // get the propeerties
        m_f32MarkerSize = static_cast<tFloat32>(GetPropertyFloat("Size of Markers"));
        m_bDebugModeEnabled = GetPropertyBool("Debug Output to Console");
        m_bCamaraParamsLoaded = tFalse;

        //Get path of detector parameter file
        cFilename fileDetectorParameter = GetPropertyStr("Detector Paramater File");
        if (fileDetectorParameter.IsEmpty())
        {
            THROW_ERROR_DESC(ERR_INVALID_FILE, "Detector Parameter File for Markers not set");
        }
        //create absolute path for marker configuration file
        ADTF_GET_CONFIG_FILENAME(fileDetectorParameter);
        fileDetectorParameter = fileDetectorParameter.CreateAbsolutePath(".");
        //check if marker configuration file exits
        if (!(cFileSystem::Exists(fileDetectorParameter)))
        {
            THROW_ERROR_DESC(ERR_INVALID_FILE, "Detector Parameter file for Markers not found");
        }
        //create the detector params
        m_detectorParams = aruco::DetectorParameters::create();
        if (!(readDetectorParameters(fileDetectorParameter.GetPtr(), m_detectorParams)))
        {
            THROW_ERROR_DESC(ERR_INVALID_FILE, "Detector Parameter file not valid");
        }

        m_Dictionary = aruco::getPredefinedDictionary(aruco::DICT_5X5_50);

        //Get path of calibration file with camera paramters
        cFilename fileCalibration = GetPropertyStr("Calibration File for used Camera");

        if (fileCalibration.IsEmpty())
        {
            THROW_ERROR_DESC(ERR_INVALID_FILE, "Calibration File for camera not set");
        }

        //Get path of calibration file with camera paramters
        ADTF_GET_CONFIG_FILENAME(fileCalibration);
        fileCalibration = fileCalibration.CreateAbsolutePath(".");
        //check if calibration file with camera paramters exits
        if (!(cFileSystem::Exists(fileCalibration)))
        {
            THROW_ERROR_DESC(ERR_INVALID_FILE, "Calibration File for camera not found");
        }

        // read the calibration file with camera paramters exits and save to member variable
        readCameraParameters(fileCalibration.GetPtr(), m_Intrinsics, m_Distorsion);
        m_bCamaraParamsLoaded = tTrue;

    }
    else if (eStage == StageGraphReady)
    {
        // get the image format of the input video pin
        cObjectPtr<IMediaType> pType;
        RETURN_IF_FAILED(m_oPinInputVideo.GetMediaType(&pType));

        cObjectPtr<IMediaTypeVideo> pTypeVideo;
        RETURN_IF_FAILED(pType->GetInterface(IID_ADTF_MEDIA_TYPE_VIDEO, (tVoid**)&pTypeVideo));

        // set the image format of the input video pin
        UpdateInputImageFormat(pTypeVideo->GetFormat());

        // set the image format of the output video pin
        UpdateOutputImageFormat(pTypeVideo->GetFormat());

        // IDs were not set yet
        m_bIDsRoadSignExtSet = tFalse;
        m_bIDsRoadSignSet = tFalse;
    }
    RETURN_NOERROR;
}

tResult SOP_MarkerDetector::Shutdown(tInitStage eStage, __exception)
{
    return cFilter::Shutdown(eStage, __exception_ptr);
}

tResult SOP_MarkerDetector::OnPinEvent(IPin* pSource,
                                    tInt nEventCode,
                                    tInt nParam1,
                                    tInt nParam2,
                                    IMediaSample* pMediaSample)
{
    switch (nEventCode)
    {
    case IPinEventSink::PE_MediaSampleReceived:
        // a new image was received so the processing is started
        if (pSource == &m_oPinInputVideo)
        {
            UCOM_TIMING_SPOT(m_oProcessStart);
            ProcessVideo(pMediaSample);
            UCOM_TIMING_SPOT(m_oProcessEnd);
        }
        break;
    case IPinEventSink::PE_MediaTypeChanged:
        if (pSource == &m_oPinInputVideo)
        {
            //the input format was changed, so the imageformat has to changed in this filter also
            cObjectPtr<IMediaType> pType;
            RETURN_IF_FAILED(m_oPinInputVideo.GetMediaType(&pType));

            cObjectPtr<IMediaTypeVideo> pTypeVideo;
            RETURN_IF_FAILED(pType->GetInterface(IID_ADTF_MEDIA_TYPE_VIDEO, (tVoid**)&pTypeVideo));

            UpdateInputImageFormat(m_oPinInputVideo.GetFormat());
            UpdateOutputImageFormat(m_oPinInputVideo.GetFormat());
        }
        break;
    default:
        break;
    }
    RETURN_NOERROR;
}



tResult SOP_MarkerDetector::ProcessVideo(adtf::IMediaSample* pISample)
{
    RETURN_IF_POINTER_NULL(pISample);

    //creating new pointer for input data
    const tVoid* l_pSrcBuffer;
    //creating matrix for input image
    Mat inputImage;
    // the results from aruco detection
    vector< int > ids;
    vector< vector< Point2f > > corners, rejected;
    vector< vector< Point2f > > after_tranmit_corners;
    vector< Point2f > transmit_corners;
    vector< Vec3d > rvecs, tvecs;
    //receiving data from input sample, and saving to inputImage
    if (IS_OK(pISample->Lock(&l_pSrcBuffer)))
    {
        //convert to mat
        inputImage = Mat(m_sInputFormat.nHeight, m_sInputFormat.nWidth, CV_8UC3, (tVoid*)l_pSrcBuffer, m_sInputFormat.nBytesPerLine);

		UCOM_TIMING_SPOT(m_oPreMarkerDetect);


        // doing the detection of markers in image
        if(ROI_ON_OFF == tTrue)
        {
            cv::Rect roi(ROIOffsetX, ROIOffsetY, ROIWidth, ROIHeight);
            cv::Mat roiImage(inputImage, roi);
            aruco::detectMarkers(roiImage, m_Dictionary, corners, ids, m_detectorParams, rejected);
            int new_col = 0;
            int new_row = 0;
            if(corners.size() != 0)
            {

                vector< vector< Point2f > >::iterator it;
                for (it = corners.begin(); it != corners.end(); it++)
                {
                    transmit_corners.clear();
                    for (int p = 0; p < 4; p++)
                    {

                        new_col = tInt(it->at(p).x) + ROIOffsetX;
                        new_row = tInt(it->at(p).y) + ROIOffsetY;
                        transmit_corners.push_back(Point2f(new_col, new_row));


                        //LOG_INFO(adtf_util::cString::Format("%d x, y =%d %d",p,  tInt(it->at(p).x), tInt(it->at(p).y)));

                    }
                    after_tranmit_corners.push_back(transmit_corners);
                    //LOG_INFO(adtf_util::cString::Format("-------------------------------"));
                }
                //LOG_INFO(adtf_util::cString::Format("aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa"));
            }

//            if(tran_corners.size() != 0)
//            {

//                vector< vector< Point2f > >::iterator itt;
//                for (itt = tran_corners.begin(); itt != tran_corners.end(); itt++)
//                {
//                    for (int pp = 0; pp < 4; pp++)
//                    {


//                        LOG_INFO(adtf_util::cString::Format("%d x, y =%d %d",pp,  tInt(itt->at(pp).x), tInt(itt->at(pp).y)));

//                    }
//                    LOG_INFO(adtf_util::cString::Format("-------------------------------"));
//                }
//                LOG_INFO(adtf_util::cString::Format("bbbbbbbbbbbbbbbbbbbbbbbbbb"));
//            }
            rectangle(inputImage, Point(ROIOffsetX,ROIOffsetY), Point((ROIOffsetX + ROIWidth), (ROIOffsetY + ROIHeight)), Scalar(0,0,255), 2);
        }
        else
            aruco::detectMarkers(inputImage, m_Dictionary, corners, ids, m_detectorParams, rejected);
        
		UCOM_TIMING_SPOT(m_oPostMarkerDetect);
		UCOM_TIMING_SPOT(m_oPrePoseEstimation);





        // if we have the camera pararmeter available we calculate the pose
        if (m_bCamaraParamsLoaded && ids.size() > 0)
        {

            if(ROI_ON_OFF == tTrue)
            {
                aruco::estimatePoseSingleMarkers(after_tranmit_corners, m_f32MarkerSize, m_Intrinsics, m_Distorsion, rvecs, tvecs);
            }
            else
                aruco::estimatePoseSingleMarkers(corners, m_f32MarkerSize, m_Intrinsics, m_Distorsion, rvecs, tvecs);
        }



		UCOM_TIMING_SPOT(m_oPostPoseEstimation);

        pISample->Unlock(l_pSrcBuffer);
    }
    else
    {
        RETURN_NOERROR;
    }

//    transmitGCL(ids, corners);

    Mat outputImage;
    // 1: nothing is drawn, 2: results are drawn so we need the have copy of the frame otherwise the orginal mediasample is modified
    if (m_oPinOutputVideo.IsConnected())
    {
        // do a deep copy of the image, otherwise the orginal frame is modified
        outputImage = inputImage.clone();
        // draw the marker in the image
        if(ROI_ON_OFF == tTrue)
            aruco::drawDetectedMarkers(outputImage, after_tranmit_corners, ids);
        else
            aruco::drawDetectedMarkers(outputImage, corners, ids);


        if (m_bCamaraParamsLoaded  && ids.size() > 0)
        {
            for (unsigned int i = 0; i < ids.size(); i++)
            {
                aruco::drawAxis(outputImage, m_Intrinsics, m_Distorsion, rvecs[i], tvecs[i], m_f32MarkerSize * 0.5f);
            }
        }
        //creating new media sample for output
        cObjectPtr<IMediaSample> pNewSample;
        RETURN_IF_FAILED(AllocMediaSample((tVoid**)&pNewSample));
        pNewSample->Update(pISample->GetTime(), outputImage.data, m_sOutputFormat.nSize, 0);
        m_oPinOutputVideo.Transmit(pNewSample);
    }

    //print marker info and draw the markers in image
    for (unsigned int i = 0; i < ids.size(); i++)
    {
//        // call the function to transmit a road sign sample with the detected marker
//        sendRoadSignStruct(static_cast<tInt16>(ids[i]), getMarkerArea(corners[i]), pISample->GetTime());

        // call the function to transmit a extended road sign sample with the detected marker if the Tvec in the marker was correctly set
        if (m_bCamaraParamsLoaded)
        {
            if(ROI_ON_OFF == tTrue)
                sendRoadSignStructExt(static_cast<tInt16>(ids[i]), getMarkerArea(after_tranmit_corners[i]), pISample->GetTime(), tvecs[i], rvecs[i]);
            else
                sendRoadSignStructExt(static_cast<tInt16>(ids[i]), getMarkerArea(corners[i]), pISample->GetTime(), tvecs[i], rvecs[i]);

        }
    }

    RETURN_NOERROR;
}

tResult SOP_MarkerDetector::UpdateInputImageFormat(const tBitmapFormat* pFormat)
{
    if (pFormat != NULL)
    {
        m_sInputFormat = (*pFormat);

        LOG_INFO(adtf_util::cString::Format("Marker Detection Filter: Input: Size %d x %d ; BPL %d ; Size %d , PixelFormat; %d", m_sInputFormat.nWidth, m_sInputFormat.nHeight, m_sInputFormat.nBytesPerLine, m_sInputFormat.nSize, m_sInputFormat.nPixelFormat));

    }

    RETURN_NOERROR;
}

tResult SOP_MarkerDetector::UpdateOutputImageFormat(const tBitmapFormat* pFormat)
{
    if (pFormat != NULL)
    {
        m_sOutputFormat = (*pFormat);

        LOG_INFO(adtf_util::cString::Format("Marker Detection Filter: Output: Size %d x %d ; BPL %d ; Size %d, PixelFormat; %d", m_sOutputFormat.nWidth, m_sOutputFormat.nHeight, m_sOutputFormat.nBytesPerLine, m_sOutputFormat.nSize, m_sOutputFormat.nPixelFormat));

        m_oPinOutputVideo.SetFormat(&m_sOutputFormat, NULL);
    }

    RETURN_NOERROR;
}

//tResult SOP_MarkerDetector::sendRoadSignStruct(const tInt16 &i16ID, const tFloat32 &f32MarkerSize, const tTimeStamp &timeOfFrame)
//{
//    // create new media sample
//    cObjectPtr<IMediaSample> pMediaSample;
//    RETURN_IF_FAILED(AllocMediaSample((tVoid**)&pMediaSample));

//    // get the serializer
//    cObjectPtr<IMediaSerializer> pSerializer;
//    m_pDescriptionRoadSign->GetMediaSampleSerializer(&pSerializer);
//    tInt nSize = pSerializer->GetDeserializedSize();

//    // alloc the buffer memory
//    RETURN_IF_FAILED(pMediaSample->AllocBuffer(nSize));

//    {
//        // focus for sample write lock
//        //write date to the media sample with the coder of the descriptor
//        __adtf_sample_write_lock_mediadescription(m_pDescriptionRoadSign, pMediaSample, pCoder);

//        // get IDs
//        if (!m_bIDsRoadSignSet)
//        {
//            pCoder->GetID("i16Identifier", m_szIDRoadSignI16Identifier);
//            pCoder->GetID("f32Imagesize", m_szIDRoadSignF32Imagesize);
//            m_bIDsRoadSignSet = tTrue;
//        }

//        pCoder->Set(m_szIDRoadSignI16Identifier, (tVoid*)&i16ID);
//        pCoder->Set(m_szIDRoadSignF32Imagesize, (tVoid*)&f32MarkerSize);

//        pMediaSample->SetTime(timeOfFrame);
//    }

//    //doing the transmit
//    RETURN_IF_FAILED(m_oPinRoadSign.Transmit(pMediaSample));

//    //print debug info if activated
//    if (m_bDebugModeEnabled)  LOG_INFO(cString::Format("Sign ID %d detected. Area is: %f", i16ID, f32MarkerSize));

//    RETURN_NOERROR;
//}

tResult SOP_MarkerDetector::sendRoadSignStructExt(const tInt16 &i16ID, const tFloat32 &f32MarkerSize, const tTimeStamp &timeOfFrame, const Vec3d &Tvec, const Vec3d &Rvec)
{
    // create new media sample
    cObjectPtr<IMediaSample> pMediaSample;
    RETURN_IF_FAILED(AllocMediaSample((tVoid**)&pMediaSample));

    // get the serializer
    cObjectPtr<IMediaSerializer> pSerializer;
    m_pDescriptionRoadSignExt->GetMediaSampleSerializer(&pSerializer);
    tInt nSize = pSerializer->GetDeserializedSize();

    // alloc the buffer memory
    RETURN_IF_FAILED(pMediaSample->AllocBuffer(nSize));

    {
        // focus for sample write lock
        //write date to the media sample with the coder of the descriptor
        __adtf_sample_write_lock_mediadescription(m_pDescriptionRoadSignExt, pMediaSample, pCoder);

        // get IDs
        if (!m_bIDsRoadSignExtSet)
        {
            pCoder->GetID("i16Identifier", m_szIDRoadSignExtI16Identifier);
            pCoder->GetID("f32Imagesize", m_szIDRoadSignExtF32Imagesize);
            pCoder->GetID("af32RVec[0]", m_szIDRoadSignExtAf32RVec);
            pCoder->GetID("af32TVec[0]", m_szIDRoadSignExtAf32TVec);
            m_bIDsRoadSignExtSet = tTrue;
        }

        pCoder->Set(m_szIDRoadSignExtI16Identifier, (tVoid*)&i16ID);
        pCoder->Set(m_szIDRoadSignExtF32Imagesize, (tVoid*)&f32MarkerSize);
        //convert from cv::Vec3D to array
        tFloat32 rvecFl32array[3] = { tFloat32(Rvec[0]),  tFloat32(Rvec[1]),  tFloat32(Rvec[2]) };
        tFloat32 tvecFl32array[3] = { tFloat32(Tvec[0]),  tFloat32(Tvec[1]),  tFloat32(Tvec[2]) };
        pCoder->Set("af32TVec", (tVoid*)&tvecFl32array[0]);
        pCoder->Set("af32RVec", (tVoid*)&rvecFl32array[0]);

        pMediaSample->SetTime(timeOfFrame);
    }
    //doing the transmit
    RETURN_IF_FAILED(m_oPinRoadSignExt.Transmit(pMediaSample));

    //print debug info if activated
    if (m_bDebugModeEnabled) LOG_INFO(cString::Format("Sign ID %d detected, translation is: %f, %f, %f", i16ID, Tvec[0], Tvec[1], Tvec[2]));
    RETURN_NOERROR;
}


//tResult SOP_MarkerDetector::transmitGCL(const vector< int >& ids, const vector< vector< Point2f > >& corners)
//{
//    IDynamicMemoryBlock* pGCLCmdDebugInfo;

//    if ((ids.size() != corners.size()) || ids.size() == 0 || corners.size() == 0)
//    {

//        cGCLWriter::GetDynamicMemoryBlock(pGCLCmdDebugInfo);
//        cGCLWriter::StoreCommand(pGCLCmdDebugInfo, GCL_CMD_CLEAR);
//        cGCLWriter::StoreCommand(pGCLCmdDebugInfo, GCL_CMD_END);
//    }
//    else
//    {
//        cGCLWriter::GetDynamicMemoryBlock(pGCLCmdDebugInfo);

//        //set color
//        cGCLWriter::StoreCommand(pGCLCmdDebugInfo, GCL_CMD_FGCOL, cColor::Red.GetRGBA());

//        //iterate through all corners
//        vector<int>::const_iterator itIds = ids.begin();
//        for (vector< vector< Point2f > >::const_iterator it = corners.begin(); it != corners.end(); it++, itIds++)
//        {
//            // add the ID as text to middle of marker
//            tInt centerMarkerX = 0;
//            tInt centerMarkerY = 0;
//            for (int p = 0; p < 4; p++)
//            {
//                centerMarkerX += tInt(it->at(p).x);
//                centerMarkerY += tInt(it->at(p).y);
//            }
//            centerMarkerX = centerMarkerX / 4;
//            centerMarkerY = centerMarkerY / 4;
//            cString idTest = cString::FromInt(*itIds);
//            cGCLWriter::StoreCommand(pGCLCmdDebugInfo, GCL_CMD_TEXT_SIZE_HUGE);
//            cGCLWriter::StoreCommand(pGCLCmdDebugInfo, GCL_CMD_TEXT, centerMarkerX, centerMarkerY, idTest.GetLength());
//            cGCLWriter::StoreData(pGCLCmdDebugInfo, idTest.GetLength(), idTest.GetPtr());

//            // draw marker sides as lines
//            for (int j = 0; j < 4; j++)
//            {
//                Point2i p0, p1;
//                p0 = it->at(j);
//                p1 = it->at((j + 1) % 4);
//                cGCLWriter::StoreCommand(pGCLCmdDebugInfo, GCL_CMD_DRAWLINE, p0.x, p0.y, p1.x, p1.y);

//            }
//            //// draw first corner as big circle to check rotation
//            Point2i pFirst;
//            pFirst = it->at(0);
//            cGCLWriter::StoreCommand(pGCLCmdDebugInfo, GCL_CMD_FILLCIRCLE, pFirst.x, pFirst.y, 10, 10);
//        }

//        cGCLWriter::StoreCommand(pGCLCmdDebugInfo, GCL_CMD_END);
//    }

//    //alloc media sample and transmit it over output pin
//    cObjectPtr<IMediaSample> pSample;
//    RETURN_IF_FAILED(AllocMediaSample((tVoid**)&pSample));
//    RETURN_IF_FAILED(pSample->Update(_clock->GetStreamTime(),
//                                     pGCLCmdDebugInfo->GetPtr(), (tInt)pGCLCmdDebugInfo->GetSize(), IMediaSample::MSF_None));
//    RETURN_IF_FAILED(m_outputPinGCL.Transmit(pSample));

//    cGCLWriter::FreeDynamicMemoryBlock(pGCLCmdDebugInfo);


//    RETURN_NOERROR;
//}
