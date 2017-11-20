
#include "stdafx.h"
#include "SOP_ImageProcess.h"
#include "ImageTranslate.h"

//Image Processing
#include "Algorithm/InitialVariable.h"
#include "Algorithm/FunctionType.h"




// define the ADTF property names to avoid errors
ADTF_FILTER_PLUGIN(ADTF_FILTER_DESC,
                   OID_ADTF_FILTER_DEF,
                   SOP_ImageProcess)

cObjectPtr<IMediaTypeDescription> m_pDescription;

//Image Processing
ITS *image_processing;
IMAGE_TRANSLATE *im_T;  	    //Image source down sample Struct (all Image source to type YUV)
IMAGE_BUFFER *VinSource;       //Orginales Bild aus Kamera


char image_algorithm_initial_flag = 0;



// new image for result
cv::Mat outputImage;
cv::Mat outputEdgeImage;
cv::Mat processImage;



SOP_ImageProcess::SOP_ImageProcess(const tChar* __info) : cFilter(__info)
{
    image_processing_control.ID_set = tFalse;
    lane_model_parameter.ID_set = tFalse;
    steering_angle.ID_set= tFalse;
    image_debug.ID_set = tFalse;



    image_algorithm_initial_flag = 0;
}

SOP_ImageProcess::~SOP_ImageProcess()
{
    FreeMemory(image_processing);
    free(image_processing);
    free(VinSource);
    free(im_T);

}

tResult SOP_ImageProcess::Start(__exception)
{


    return cFilter::Start(__exception_ptr);
}

tResult SOP_ImageProcess::Stop(__exception)
{

    return cFilter::Stop(__exception_ptr);
}
tResult SOP_ImageProcess::Init(tInitStage eStage, __exception)
{
    RETURN_IF_FAILED(cFilter::Init(eStage, __exception_ptr));

    if (eStage == StageFirst)
    {
        cObjectPtr<IMediaDescriptionManager> pDescManager;
        RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER, IID_ADTF_MEDIA_DESCRIPTION_MANAGER, (tVoid**)&pDescManager, __exception_ptr));

        tChar const * strDescSignalValue = pDescManager->GetMediaDescription("tSignalValue");
        RETURN_IF_POINTER_NULL(strDescSignalValue);
        cObjectPtr<IMediaType> pTypeSignalValue = new cMediaType(0, 0, 0, "tSignalValue", strDescSignalValue,IMediaDescription::MDF_DDL_DEFAULT_VERSION);




        // Video Input
        RETURN_IF_FAILED(m_oVideoInputPin.Create("VideoInput", IPin::PD_Input, static_cast<IPinEventSink*>(this)));
        RETURN_IF_FAILED(RegisterPin(&m_oVideoInputPin));


        // Video Output
        RETURN_IF_FAILED(m_oVideoOutputPin.Create("VideoOutput", IPin::PD_Output, static_cast<IPinEventSink*>(this)));
        RETURN_IF_FAILED(RegisterPin(&m_oVideoOutputPin));


        // Video Edge Output
        RETURN_IF_FAILED(m_oVideoEdgeOutputPin.Create("VideoEdgeOutput", IPin::PD_Output, static_cast<IPinEventSink*>(this)));
        RETURN_IF_FAILED(RegisterPin(&m_oVideoEdgeOutputPin));


        tChar const * strImageProcessControl = pDescManager->GetMediaDescription("tImageProcessControl");
        RETURN_IF_POINTER_NULL(strImageProcessControl);
        cObjectPtr<IMediaType> pTypeImagePorcessControl = new cMediaType(0, 0, 0, "tImageProcessControl", strImageProcessControl,IMediaDescription::MDF_DDL_DEFAULT_VERSION);
        RETURN_IF_FAILED(pTypeImagePorcessControl->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&image_processing_control.m_pDescription));
        RETURN_IF_FAILED(image_processing_control.input.Create("ImageControlStruct", pTypeImagePorcessControl, static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&image_processing_control.input));

        image_processing_control_ID_name[0] = "AutoControlMode";
        image_processing_control_ID_name[1] = "Reference_k";
        image_processing_control_ID_name[2] = "Reference_m";
        image_processing_control_ID_name[3] = "Reference_b";


        RETURN_IF_FAILED(pTypeSignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&steering_angle.m_pDescription));
        RETURN_IF_FAILED(steering_angle.input.Create("SteeringAngle", pTypeSignalValue, static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&steering_angle.input ));



        RETURN_IF_FAILED(pTypeSignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&image_debug.m_pDescription));
        RETURN_IF_FAILED(image_debug.output.Create("ImageDebug", pTypeSignalValue, static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&image_debug.output));


        tChar const * strLaneModelValue = pDescManager->GetMediaDescription("tLaneCurveData");
        RETURN_IF_POINTER_NULL(strLaneModelValue);
        cObjectPtr<IMediaType> pTypeLaneModel = new cMediaType(0, 0, 0, "tLaneCurveData", strLaneModelValue,IMediaDescription::MDF_DDL_DEFAULT_VERSION);

        RETURN_IF_FAILED(pTypeLaneModel->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&lane_model_parameter.m_pDescription));
        RETURN_IF_FAILED(lane_model_parameter.output.Create("LaneModelStruct", pTypeLaneModel, static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&lane_model_parameter.output));

        lane_model_ID_name[0] = "LaneDetectMode";
        lane_model_ID_name[1] = "LandModel_k";
        lane_model_ID_name[2] = "LandModel_m";
        lane_model_ID_name[3] = "LandModel_b";
        lane_model_ID_name[4] = "LaneWidth";
        lane_model_ID_name[5] = "L_SL_LorR";
        lane_model_ID_name[6] = "SolidlineFlag";
        lane_model_ID_name[7] = "BiasWarn";
        lane_model_ID_name[8] = "StopLineDistance";
        lane_model_ID_name[9] = "Adult_flag";
        lane_model_ID_name[10]= "Child_flag";







        im_T = (IMAGE_TRANSLATE*)calloc(1,sizeof(IMAGE_TRANSLATE));
        VinSource = (IMAGE_BUFFER*)calloc(1,sizeof(IMAGE_BUFFER));
        //check video capture buffer size
        im_T->video_input_buffer_width = m_sInputFormat.nHeight;
        im_T->video_input_buffer_height = m_sInputFormat.nWidth;

        //DownsamplingArrayPrepare_RGB24(IMAGE_WIDTH, IMAGE_HEIGHT, im_T);

        if(image_algorithm_initial_flag == 0)
        {
            image_processing = (ITS*)calloc(1,sizeof(ITS));
            SetITSBuffer(image_processing);
            ResetConstant(image_processing);
            image_algorithm_initial_flag = 1;
            image_processing->traindata = fopen("TrainData.txt","w");

        }

        image_processing->YImg      = &VinSource->Y[0];
        image_processing->UImg      = &VinSource->U[0];
        image_processing->VImg      = &VinSource->V[0];
        image_processing->Showimage = &VinSource->Y[0];
        image_processing->ShowUImg  = &VinSource->U[0];
        image_processing->ShowVImg  = &VinSource->V[0];




    }
    else if (eStage == StageNormal)
    {
    }

    else if (eStage == StageGraphReady)
    {
        processImage.create(IMAGE_HEIGHT, IMAGE_WIDTH, CV_8UC3);
        outputImage.create(IMAGE_HEIGHT, IMAGE_WIDTH, CV_8UC3);
        outputEdgeImage.create(IMAGE_HEIGHT, IMAGE_WIDTH, CV_8UC3);

        // get the image format of the input video pin
        cObjectPtr<IMediaType> pType;
        RETURN_IF_FAILED(m_oVideoInputPin.GetMediaType(&pType));

        cObjectPtr<IMediaTypeVideo> pTypeVideo;
        RETURN_IF_FAILED(pType->GetInterface(IID_ADTF_MEDIA_TYPE_VIDEO, (tVoid**)&pTypeVideo));

        // set the image format of the input video pin
        // set the image format of the input video pin
        if (IS_FAILED(UpdateInputImageFormat(pTypeVideo->GetFormat())))
        {
            LOG_ERROR("Invalid Input Format for this filter");
        }

    }

    RETURN_NOERROR;
}



tResult SOP_ImageProcess::Shutdown(tInitStage eStage, ucom::IException** __exception_ptr)
{
    if (eStage == StageGraphReady)
    {

        processImage.release();
        outputImage.release();
        outputEdgeImage.release();
    }

   // if(image_processing->traindata)
    //    fclose(image_processing->traindata);

    return cFilter::Shutdown(eStage, __exception_ptr);
}

tResult SOP_ImageProcess::OnPinEvent(IPin* pSource, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample* pMediaSample)
{
   // __synchronized_obj(m_critSecOnPinEvent);
    RETURN_IF_POINTER_NULL(pMediaSample);
    RETURN_IF_POINTER_NULL(pSource);
    if (nEventCode == IPinEventSink::PE_MediaSampleReceived)
    {
        if (pSource == &m_oVideoInputPin)
        {
            //check if video format is still unkown
            if (m_sInputFormat.nPixelFormat == IImage::PF_UNKNOWN)
            {
                RETURN_IF_FAILED(UpdateInputImageFormat(m_oVideoInputPin.GetFormat()));
            }

            ProcessVideo(pMediaSample);


                lane_model[0] = (tFloat)image_processing->L_DetectMode;
                if(image_processing->L_DetectMode == LTRACE && image_processing->L_StbCtr == 15)
                {
                    lane_model[1] = image_processing->k;
                    lane_model[2] = image_processing->m;
                    lane_model[3] = image_processing->bm;
                }
                else if(image_processing->L_DetectMode == SL_TRACE && image_processing->SL_LaneModel.L_StbCtr == 25)
                {
                    lane_model[1] = image_processing->SL_LaneModel.k;
                    lane_model[2] = image_processing->SL_LaneModel.m;
                    lane_model[3] = image_processing->SL_LaneModel.bm;
                }
                lane_model[4] = (tFloat)image_processing->L_WAvg;
                lane_model[5] = image_processing->SL_LaneModel.L_SL_LorR;
                if(image_processing->SolidlineL==0 && image_processing->SolidlineR==0)
                    lane_model[6] = 0;
                else if(image_processing->SolidlineL==0 && image_processing->SolidlineR==1)
                    lane_model[6] = 1;
                else if(image_processing->SolidlineL==1 && image_processing->SolidlineR==0)
                    lane_model[6] = 2;
                else if(image_processing->SolidlineL==1 && image_processing->SolidlineR==1)
                    lane_model[6] = 3;
                lane_model[7] = (tFloat)image_processing->L_BiasWarn;

                if(image_processing->Stop_Line.mode == TRACE && image_processing->Stop_Line.stable_counter > 5 && image_processing->Stop_Line.distance > 40 && image_processing->Stop_Line.distance < 200)
                    lane_model[8] = (tFloat)image_processing->Stop_Line.distance;
                else
                    lane_model[8] = 0;

                if(image_processing->adult.mode == TRACE && image_processing->adult.stable_counter > 10 && image_processing->adult.direction == 1)
                    lane_model[9] = 2;
                else if(image_processing->adult.mode == TRACE && image_processing->adult.stable_counter > 10 && image_processing->adult.direction == 0)
                    lane_model[9] = 1;
                else
                    lane_model[9] = 0;

                if(image_processing->child.mode == TRACE && image_processing->child.stable_counter > 10)
                    lane_model[10] = 1;
                else
                    lane_model[10] = 0;


                WritePinArrayValue(&lane_model_parameter, 11,lane_model_ID_name, lane_model);

        else if (pSource == &image_processing_control.input)
        {
            ReadPinArrayValue(pMediaSample,&image_processing_control, image_processing_control_ID_name, 4, image_processing_control_value);
            image_processing->default_k = image_processing_control_value[1];
            image_processing->default_m = image_processing_control_value[2];
            image_processing->default_b = 0;

            image_processing->function_switch.input_flag = (char)image_processing_control_value[0];
//            image_processing->function_switch.input_flag == 0;
//            image_processing->function_switch.input_flag |= LANE_DETECTION;
//            image_processing->function_switch.input_flag |= STOP_LINE_DETECTION;
//            image_processing->function_switch.input_flag |= ADULT_DETECTION;
//            image_processing->function_switch.input_flag |= CHILD_DETECTION;

        }
    }
    else if (nEventCode == IPinEventSink::PE_MediaTypeChanged)
    {
        if (pSource == &m_oVideoInputPin)
        {
            //the input format was changed, so the imageformat has to changed in this filter also
            RETURN_IF_FAILED(UpdateInputImageFormat(m_oVideoInputPin.GetFormat()));
        }
    }
    RETURN_NOERROR;
}

tResult SOP_ImageProcess::ProcessVideo(IMediaSample* pSample)
{

    RETURN_IF_POINTER_NULL(pSample);

    const tVoid* l_pSrcBuffer;


    //receiving data from input sample, and saving to TheInputImage
    if (IS_OK(pSample->Lock(&l_pSrcBuffer)))
    {
        //convert to mat, be sure to select the right pixelformat
        if (tInt32(m_inputImage.total() * m_inputImage.elemSize()) == m_sInputFormat.nSize)
        {
            //copy the data to matrix (make a copy, not change the sample content itself!)
            memcpy(m_inputImage.data, l_pSrcBuffer, size_t(m_sInputFormat.nSize));

            //or just set the data pointer of matrix because we create a new matrix later one
            //m_inputImage.data = (uchar*)(l_pSrcBuffer);

            //Canny(m_inputImage, outputImage, 100, 200);// Detect Edges

        }
        pSample->Unlock(l_pSrcBuffer);

            ImageBufferDownsamplingBGR_to_YUY2(IMAGE_WIDTH, IMAGE_HEIGHT, m_inputImage, 0);

            ITSLANE_MAIN(image_processing);

            Transfer_YUY2_to_BGR(IMAGE_WIDTH, IMAGE_HEIGHT, outputImage);

            DrawImageEdge(IMAGE_WIDTH, IMAGE_HEIGHT, outputEdgeImage);

    }


    if (!outputImage.empty()  && m_oVideoOutputPin.IsConnected())
    {
        UpdateOutputImageFormat(outputImage);

        //create a cImage from CV Matrix (not necessary, just for demonstration9
        cImage newImage;
        newImage.Create(m_sOutputFormat.nWidth, m_sOutputFormat.nHeight, m_sOutputFormat.nBitsPerPixel, m_sOutputFormat.nBytesPerLine, outputImage.data);

        //create the new media sample
        cObjectPtr<IMediaSample> pMediaSample;
        RETURN_IF_FAILED(AllocMediaSample((tVoid**)&pMediaSample));
        //updating media sample
        RETURN_IF_FAILED(pMediaSample->Update(_clock->GetStreamTime(), newImage.GetBitmap(), newImage.GetSize(), IMediaSample::MSF_None));
        //transmitting
        RETURN_IF_FAILED(m_oVideoOutputPin.Transmit(pMediaSample));


    }


    if (!outputEdgeImage.empty()  && m_oVideoEdgeOutputPin.IsConnected())
    {
        UpdateOutputImageEdgeFormat(outputEdgeImage);

        //create a cImage from CV Matrix (not necessary, just for demonstration9
        cImage newImage;
        newImage.Create(m_sOutputEdgeFormat.nWidth, m_sOutputEdgeFormat.nHeight, m_sOutputEdgeFormat.nBitsPerPixel, m_sOutputEdgeFormat.nBytesPerLine, outputEdgeImage.data);

        //create the new media sample
        cObjectPtr<IMediaSample> pMediaSample;
        RETURN_IF_FAILED(AllocMediaSample((tVoid**)&pMediaSample));
        //updating media sample
        RETURN_IF_FAILED(pMediaSample->Update(_clock->GetStreamTime(), newImage.GetBitmap(), newImage.GetSize(), IMediaSample::MSF_None));
        //transmitting
        RETURN_IF_FAILED(m_oVideoEdgeOutputPin.Transmit(pMediaSample));

    }


    RETURN_NOERROR;
}

tResult SOP_ImageProcess::UpdateInputImageFormat(const tBitmapFormat* pFormat)
{
    if (pFormat != NULL)
    {
        //update member variable
        m_sInputFormat = (*pFormat);
        LOG_INFO(adtf_util::cString::Format("Input: Size %d x %d ; BPL %d ; Size %d , PixelFormat; %d", m_sInputFormat.nWidth, m_sInputFormat.nHeight, m_sInputFormat.nBytesPerLine, m_sInputFormat.nSize, m_sInputFormat.nPixelFormat));
        //create the input matrix
        RETURN_IF_FAILED(BmpFormat2Mat(m_sInputFormat, m_inputImage));
    }
    RETURN_NOERROR;
}

tResult SOP_ImageProcess::UpdateOutputImageFormat(const cv::Mat& outputImage)
{
    //check if pixelformat or size has changed
    if (tInt32(outputImage.total() * outputImage.elemSize()) != m_sOutputFormat.nSize)
    {
        Mat2BmpFormat(outputImage, m_sOutputFormat);

        LOG_INFO(adtf_util::cString::Format("Output: Size %d x %d ; BPL %d ; Size %d , PixelFormat; %d", m_sOutputFormat.nWidth, m_sOutputFormat.nHeight, m_sOutputFormat.nBytesPerLine, m_sOutputFormat.nSize, m_sOutputFormat.nPixelFormat));
        //set output format for output pin
        m_oVideoOutputPin.SetFormat(&m_sOutputFormat, NULL);
    }
    RETURN_NOERROR;
}

tResult SOP_ImageProcess::UpdateOutputImageEdgeFormat(const cv::Mat& outputImage)
{
    //check if pixelformat or size has changed
    if (tInt32(outputImage.total() * outputImage.elemSize()) != m_sOutputEdgeFormat.nSize)
    {
        Mat2BmpFormat(outputImage, m_sOutputEdgeFormat);

       // LOG_INFO(adtf_util::cString::Format("Output: Size %d x %d ; BPL %d ; Size %d , PixelFormat; %d", m_sOutputEdgeFormat.nWidth, m_sOutputEdgeFormat.nHeight, m_sOutputEdgeFormat.nBytesPerLine, m_sOutputEdgeFormat.nSize, m_sOutputEdgeFormat.nPixelFormat));
        //set output format for output pin
        m_oVideoEdgeOutputPin.SetFormat(&m_sOutputEdgeFormat, NULL);
    }
    RETURN_NOERROR;
}

tResult SOP_ImageProcess::ImageBufferDownsamplingBGR_to_YUY2(int Im_width, int Im_height, const cv::Mat& image, char type)
{
    int row, col;
    int red = 0;
    int green = 0;
    int blue = 0;
    int index = 0;
    int index2 = 0;
    for(row = 0; row < Im_height; row++)
    {
        for(col = 0; col < Im_width; col++)
        {
            red  = (unsigned char)image.at<Vec3b>((row * 2),(col * 2))[0];
            green = (unsigned char)image.at<Vec3b>((row * 2),(col * 2))[1];
            blue   = (unsigned char)image.at<Vec3b>((row * 2),(col * 2))[2];

            VinSource->Y[index] = uCharLimitSet((int)((0.299*(float)red) + (0.587*(float)green) + (0.114*(float)blue)));
            index++;
            if(index % 2 == 0)
            {
                if(type == 0){
                    //es ist schneller aber unklar
                    VinSource->U[index2] = uCharLimitSet(((-43*red-85*green+128*blue)>> 8)+128);
                    VinSource->V[index2] = uCharLimitSet(((128*red-107*green-21*blue)>> 8)+128);
                }
                else if(type == 1){
                    //es ist langsamer aber kl?rer
                    VinSource->U[index2] = uCharLimitSet(-0.169*red-0.331*green+0.499*blue+128);
                    VinSource->V[index2] = uCharLimitSet(0.499*red-0.418*green-0.0813*blue+128);
                }
                index2++;
            }
        }
    }
    RETURN_NOERROR;
}

tResult SOP_ImageProcess::Transfer_YUV_to_YUY2(int Im_width, int Im_height, const cv::Mat& image)
{
    int row, col;
    int index = 0;
    int index2 = 0;
    for(row = 0; row < Im_height; row++)
    {
        for(col = 0; col < Im_width; col++)
        {
            VinSource->Y[index] = (unsigned char)image.at<Vec3b>(row,col)[0];
            index++;
            if(index % 2 == 0)
            {
                VinSource->U[index2] = (unsigned char)image.at<Vec3b>(row,col)[1];
                VinSource->V[index2] = (unsigned char)image.at<Vec3b>(row,col)[2];
                index2++;
            }
        }
    }
    RETURN_NOERROR;
}

tResult SOP_ImageProcess::Transfer_YUY2_to_YUV(int Im_width, int Im_height, cv::Mat& image)
{
    int row, col;
    int index = 0;
    int index2 = 0;


    for(row = 0; row < Im_height; row++)
    {
        for(col = 0; col < Im_width; col++)
        {
            image.at<Vec3b>(row,col)[0] = VinSource->Y[index];

            image.at<Vec3b>(row,col)[1] = VinSource->U[index2];
            image.at<Vec3b>(row,col)[2] = VinSource->V[index2];


            index++;
            if(index % 2 == 0)
            {
                index2++;
            }
        }
    }
    RETURN_NOERROR;
}

tResult SOP_ImageProcess::Transfer_YUY2_to_BGR(int Im_width, int Im_height, cv::Mat& image)
{
    int row, col;
    int index = 0;
    int index2 = 0;


    for(row = 0; row < Im_height; row++)
    {
        for(col = 0; col < Im_width; col++)
        {
            image.at<Vec3b>(row,col)[0] = uCharLimitSet((298 * (VinSource->Y[index] - 16) + 516 * (VinSource->U[index2]- 128) + 128) >> 8);
            image.at<Vec3b>(row,col)[1] = uCharLimitSet((298 * (VinSource->Y[index] - 16) - 100 * (VinSource->U[index2] - 128) - 208 * (VinSource->V[index2] - 128) + 128) >> 8);
            image.at<Vec3b>(row,col)[2] = uCharLimitSet((298 * (VinSource->Y[index] - 16) + 409 * (VinSource->V[index2] - 128) + 128) >> 8);

            index++;
            if(index % 2 == 0)
            {
                index2++;
            }
        }
    }
    RETURN_NOERROR;
}

tResult SOP_ImageProcess::DrawImageEdge(int Im_width, int Im_height, cv::Mat& image)
{
    int row, col;
    int red = 0;
    int green = 0;
    int blue = 0;
    int index = 0;


    for(row = 0; row < Im_height; row++)
    {
        for(col = 0; col < Im_width; col++)
        {
            red = green = blue = 0;

//            if(image_processing->YImg[index] > image_processing->O_MarkLightTH)
//            {
//                red = green = blue = 255;
//            }
//            else
//            {
//                red = green = blue = 0;
//            }


//            if((image_processing->O_InfoPlane[index] & HEVEINFO) == HEVEINFO)
//            {
//                red = 255;
//                green = blue = 0;
//            }
//            else if((image_processing->O_InfoPlane[index] & VEINFO) == VEINFO)
//            {
//                green = 255;
//                blue = red = 0;
//            }
//            else if((image_processing->O_InfoPlane[index] & HEINFO) == HEINFO)
//            {
//                blue = 255;
//                green = red = 0;
//            }

//            if((image_processing->O_InfoPlane[index] & RSDHEVEINFO) == RSDHEVEINFO)
//            {
//                red = 255;
//                green = blue = 0;
//            }


//            int Y = (image_processing->Hog_InfoPlane[index] * 7);
//            int Y = (image_processing->Gxy_InfoPlane[index]);
//            int U = 128;
//            int V = 128;

//            red   = uCharLimitSet((298 * (Y - 16) + 409 * (V - 128) + 128) >> 8);
//            green = uCharLimitSet((298 * (Y - 16) - 100 * (U - 128) - 208 * (V - 128) + 128) >> 8);
//            blue  = uCharLimitSet((298 * (Y - 16) + 516 * (U- 128) + 128) >> 8);


//            if(image_processing->O_P_InfoPlane[index] == 1)
//            {
//                red = 255;
//                green = blue = 0;
//            }

            switch (image_processing->Axy_InfoPlane[index])
            {
                case 1:
                    red   = 255;
                    green = 0;
                    blue  = 0;
                    break;
                case 2:
                    red   = 255;
                    green = 128;
                    blue  = 0;
                    break;
                case 3:
                    red   = 255;
                    green = 128;
                    blue  = 128;
                    break;
                case 4:
                    red   = 0;
                    green = 255;
                    blue  = 0;
                    break;
                case 5:
                    red   = 128;
                    green = 255;
                    blue  = 0;
                    break;
                case 6:
                    red   = 128;
                    green = 255;
                    blue  = 128;
                    break;
                case 7:
                    red   = 0;
                    green = 0;
                    blue  = 255;
                    break;
                case 8:
                    red   = 0;
                    green = 128;
                    blue  = 255;
                    break;
                case 9:
                    red   = 128;
                    green = 128;
                    blue  = 255;
                    break;
                case 0:
                    red   = 0;
                    green = 0;
                    blue  = 0;
                    break;
                default:
                    red   = 0;
                    green = 0;
                    blue  = 0;
                    break;
            }

//            if((image_processing->L_ColProjection[index] & VEINFO) == VEINFO)
//            {
//                green = 255;
//                blue = red = 0;
//            }
//            else
//            {
//                red = green = blue = 0;
//            }

            image.at<Vec3b>(row,col)[0] = blue;      //B
            image.at<Vec3b>(row,col)[1] = green;     //G
            image.at<Vec3b>(row,col)[2] = red;     //R

            index++;
        }
    }
    RETURN_NOERROR;
}


tResult SOP_ImageProcess::WriteSignalValue(sop_pin_struct *pin, tFloat32 value, tUInt32 timestamp)
{
    //use mutex
    //__synchronized_obj(m_critSecTransmitControl);


    cObjectPtr<IMediaSample> pMediaSample;
    AllocMediaSample((tVoid**)&pMediaSample);

    cObjectPtr<IMediaSerializer> pSerializer;
    pin->m_pDescription->GetMediaSampleSerializer(&pSerializer);
    pMediaSample->AllocBuffer(pSerializer->GetDeserializedSize());

    cObjectPtr<IMediaTypeDescription> m_pDescription;
    m_pDescription = pin->m_pDescription;

    {
        __adtf_sample_write_lock_mediadescription(m_pDescription, pMediaSample, pCoderOutput);

        if(pin->ID_set == tFalse)
        {
            pCoderOutput->GetID("f32Value", pin->ID_value);
            pCoderOutput->GetID("ui32ArduinoTimestamp", pin->ID_timestamp);

            pin->ID_set = tTrue;
        }

        pCoderOutput->Set(pin->ID_value, (tVoid*)&value);
        pCoderOutput->Set(pin->ID_timestamp, (tVoid*)&timestamp);

    }

    pMediaSample->SetTime(_clock->GetStreamTime());
    pin->output.Transmit(pMediaSample);

    RETURN_NOERROR;
}


tResult SOP_ImageProcess::WritePinArrayValue(sop_pin_struct *pin, int number_of_array, cString *ID_name , tFloat32 *value)
{
    //use mutex
    //__synchronized_obj(m_critSecTransmitControl);


    int index = 0;
    int id_set_index = 0;
    tFloat32 output_value = 0;
    tBufferID idValue;

    cObjectPtr<IMediaSample> pMediaSample;
    AllocMediaSample((tVoid**)&pMediaSample);

    cObjectPtr<IMediaSerializer> pSerializer;
    pin->m_pDescription->GetMediaSampleSerializer(&pSerializer);
    pMediaSample->AllocBuffer(pSerializer->GetDeserializedSize());

    cObjectPtr<IMediaTypeDescription> m_pDescription;
    m_pDescription = pin->m_pDescription;

    {
        __adtf_sample_write_lock_mediadescription(m_pDescription, pMediaSample, pCoderOutput);

        for(index = 0; index < number_of_array; index++)
        {
            if(pin->ID_set == tFalse)
            {

                pin->ID_value_array.clear();
                for(id_set_index = 0; id_set_index < number_of_array; id_set_index++)
                {
                    pCoderOutput->GetID(ID_name[id_set_index], idValue);
                    pin->ID_value_array.push_back(idValue);

                }
                pin->ID_set = tTrue;
            }
            output_value = value[index];
            pCoderOutput->Set(pin->ID_value_array[index], (tVoid*)&output_value);
        }
    }

    pMediaSample->SetTime(_clock->GetStreamTime());
    pin->output.Transmit(pMediaSample);

    RETURN_NOERROR;
}

tResult SOP_ImageProcess::ReadPinArrayValue(IMediaSample* input_pMediaSample, sop_pin_struct *input_pin, cString *PIN_ID_name, int number_of_array, tFloat32 *output_value)
{
    //use mutex
    //__synchronized_obj(m_critSecGetData);


    int index = 0;
    int id_set_index = 0;
    tFloat32 buf_Value = 0;
    tBufferID idValue;

    m_pDescription = input_pin->m_pDescription;

    {
        __adtf_sample_read_lock_mediadescription(m_pDescription, input_pMediaSample, pCoder);

        for(index = 0; index < number_of_array; index++)
        {
            if(input_pin->ID_set == tFalse)
            {

                input_pin->ID_value_array.clear();

                for(id_set_index = 0; id_set_index < number_of_array; id_set_index++)
                {
                    pCoder->GetID(PIN_ID_name[id_set_index], idValue);
                    input_pin->ID_value_array.push_back(idValue);

                }
                input_pin->ID_set = tTrue;
            }
            pCoder->Get(input_pin->ID_value_array[index], (tVoid*)&buf_Value);
            output_value[index] = buf_Value;
        }
    }

    RETURN_NOERROR;
}

