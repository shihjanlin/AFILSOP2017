#include "SOP_AutonomousDriving.h"

tFloat32 f32Value = 0;
tUInt32 Ui32TimeStamp = 0;
int maneuverIndex = 0;
int sectorIndex = 0;

clock_t ts;
double duration;

// define the ADTF property names to avoid errors
ADTF_FILTER_PLUGIN(ADTF_FILTER_DESC, OID_ADTF_FILTER_DEF, SOP_AutonomousDriving)


SOP_AutonomousDriving::SOP_AutonomousDriving(const tChar* __info) : adtf::cTimeTriggeredFilter(__info)
{
    image_info_input.ID_set     = tFalse;
    image_processing_control.ID_set= tFalse;
    state_flag.ID_set           = tFalse;
    position_input.ID_set       = tFalse;
    wheel_speed_input.ID_set    = tFalse;
    m_bIDsUltrasonicSet         = tFalse;
    reference_point.ID_set      = tFalse;
    car_position_pin.ID_set     = tFalse;
    speed_output.ID_set         = tFalse;
    steering_output.ID_set      = tFalse;
    distance_overall_input.ID_set = tFalse;
    input_road_sign_ext.ID_set = tFalse;
    position_initial_pin.ID_set = tFalse;
    Obstacle_output.ID_set = tFalse;
    ParkingSpace_output.ID_set = tFalse;
    m_bIDsDriverStructSet = tFalse;

    SetPropertyStr("Configuration","stopLines.xml");
    SetPropertyBool("Configuration" NSSUBPROP_FILENAME, tTrue);
    SetPropertyStr("Configuration" NSSUBPROP_FILENAME NSSUBSUBPROP_EXTENSIONFILTER, "XML Files (*.xml)");
    SetPropertyStr("Configuration" NSSUBPROP_DESCRIPTION, "Configuration file for the stop lines coordinates");

    SetPropertyInt("SamplingRate::State Control Sampling rate in ms", 50);
    SetPropertyStr("SamplingRate::State Control Sampling rate in ms" NSSUBPROP_DESCRIPTION, "Sets the interval in msec");
    SetPropertyInt("SamplingRate::State Control Sampling rate in ms" NSSUBPROP_MIN, 10);
    SetPropertyInt("SamplingRate::State Control Sampling rate in ms" NSSUBPROP_MAX, 500);

    SetPropertyInt("SamplingRate::MPC Sampling rate in ms", 100);
    SetPropertyStr("SamplingRate::MPC Sampling rate in ms" NSSUBPROP_DESCRIPTION, "Sets the interval in msec");
    SetPropertyInt("SamplingRate::MPC Sampling rate in ms" NSSUBPROP_MIN, 10);
    SetPropertyInt("SamplingRate::MPC Sampling rate in ms" NSSUBPROP_MAX, 500);

    SetPropertyFloat("EmergencyBreak::Front Minimum break time in S",0.6);
    SetPropertyFloat("EmergencyBreak::Rear Minimum break time in S",0.5);
    SetPropertyFloat("EmergencyBreak::Minimum break distance in cm", 10);


    SetPropertyFloat("Lane Following::maximum speed", 0.8);
    SetPropertyFloat("Lane Following::minimum speed", 0.5);
    SetPropertyFloat("Lane Following::NMPC Weighting factor::High speed y", 4);
    SetPropertyFloat("Lane Following::NMPC Weighting factor::Low speed y", 8);

    SetPropertyFloat("Crossing::Marker distance in cm", 150);
    SetPropertyFloat("Crossing::Stop Line distance in cm", 75);
    SetPropertyFloat("Crossing::Stop Line stop distance in cm", 45);

    SetPropertyFloat("Crossing::TurnLeft::stop crossing in cm", 0.0);
    SetPropertyFloat("Crossing::TurnLeft::no stopcrossing in cm", -2.0);
    SetPropertyFloat("Crossing::TurnLeft::turn around counter max 3*N", 7);
    SetPropertyFloat("Crossing::TurnLeft::open image counter max 3*N", 18);
    SetPropertyFloat("Crossing::TurnLeft::NMPC Weighting factor::x", 20);
    SetPropertyFloat("Crossing::TurnLeft::NMPC Weighting factor::y", 10);

    SetPropertyFloat("Crossing::TurnRight::stop crossing in cm", 0.0);
    SetPropertyFloat("Crossing::TurnRight::no stop crossing in cm", -11.0);
    SetPropertyFloat("Crossing::TurnRight::turn around counter max 2*N", 6);
    SetPropertyFloat("Crossing::TurnRight::open image counter max 2*N", 12);
    SetPropertyFloat("Crossing::TurnRight::NMPC Weighting factor::x", 30);
    SetPropertyFloat("Crossing::TurnRight::NMPC Weighting factor::y", 10);

    SetPropertyFloat("Crossing::Straight::turn around counter max 4*N", 20);
    SetPropertyFloat("Crossing::Straight::open image counter max 4*N", 17);
    SetPropertyFloat("Crossing::Straight::NMPC Weighting factor::x", 5);
    SetPropertyFloat("Crossing::Straight::NMPC Weighting factor::y", 20);

    SetPropertyFloat("Crossing::ultra distance::1 Left section", 90);
    SetPropertyFloat("Crossing::ultra distance::2 Front section", 140);
    SetPropertyFloat("Crossing::ultra distance::3 Right section", 100);
    SetPropertyFloat("Crossing::ultra distance::4 Middle section", 80);

    SetPropertyFloat("Pullout::Left::turn around counter max 3*N", 2);
    SetPropertyFloat("Pullout::Left::open image counter max 3*N", 18);
    SetPropertyFloat("Pullout::Left::NMPC Weighting factor::x", 20);
    SetPropertyFloat("Pullout::Left::NMPC Weighting factor::y", 10);

    SetPropertyFloat("Pullout::Right::turn around counter max 3*N", 5);
    SetPropertyFloat("Pullout::Right::open image counter max 3*N", 18);
    SetPropertyFloat("Pullout::Right::NMPC Weighting factor::x", 20);
    SetPropertyFloat("Pullout::Right::NMPC Weighting factor::y", 5);

    SetPropertyFloat("Avoidance::NMPC Weighting factor::x", 0);
    SetPropertyFloat("Avoidance::NMPC Weighting factor::y", 40);
    SetPropertyFloat("Avoidance::Initial distance", 100);
    SetPropertyFloat("Avoidance::Lane change speed", 0.5);
    SetPropertyFloat("Avoidance::Side detection distance", 50);
    SetPropertyFloat("Avoidance::Come back counter", 20);


    SetPropertyFloat("Parking::Marker distance in cm", 70);
    SetPropertyFloat("Parking::Forward::turn around counter max 3*N", 3);
    SetPropertyFloat("Parking::Backward::turn around counter max 6*N", 6);
    SetPropertyFloat("Parking::Slot distance 0", 8);
    SetPropertyFloat("Parking::Slot distance 1", -88);
    SetPropertyFloat("Parking::Slot distance 2", -135);
    SetPropertyFloat("Parking::Slot distance 3", -186);
    SetPropertyFloat("Parking::Slot distance 4", -234);
    SetPropertyFloat("Parking::NMPC Weighting factor::x", 5);
    SetPropertyFloat("Parking::NMPC Weighting factor::y", 50);

    SetPropertyFloat("Obstacles::detection distance", 120);

    SetPropertyFloat("Car following::detection distance", 120);
    SetPropertyFloat("Car following::preceding vehicle minimum speed", 0.1);

    m_bDebugModeEnabled = tFalse;
    SetPropertyBool("Mode switch::Debug Output to Console", m_bDebugModeEnabled);

    m_bJuryModelEnabled = tFalse;
    SetPropertyBool("Mode switch::Jury Model on/off", m_bJuryModelEnabled);

    crossing_stopLine_mode = tFalse;
    SetPropertyBool("Mode switch::Stop line mode on/off", crossing_stopLine_mode);

    KI_adult = tTrue;
    SetPropertyBool("KI switch::Adult on/off", KI_adult);

    KI_child = tFalse;
    SetPropertyBool("KI switch::Child on/off", KI_child);


    m_log = 0;
}

SOP_AutonomousDriving::~SOP_AutonomousDriving()
{
}
tResult SOP_AutonomousDriving::PropertyChanged(const char* strProperty)
{
    front_min_break_time = static_cast<tFloat32>(GetPropertyFloat("EmergencyBreak::Front Minimum break time in S"));
    rear_min_break_time = static_cast<tFloat32>(GetPropertyFloat("EmergencyBreak::Rear Minimum break time in S"));
    min_break_distance  = static_cast<tFloat32>(GetPropertyFloat("EmergencyBreak::Minimum break distance in cm"));

    MPC_sampling_rate = static_cast<tFloat32>(GetPropertyFloat("SamplingRate::MPC Sampling rate in ms"));
    state_control_sampling_rate = static_cast<tFloat32>(GetPropertyFloat("SamplingRate::State Control Sampling rate in ms"));

    lane_follow_maxSpeed = static_cast<tFloat32>(GetPropertyFloat("Lane Following::maximum speed"));
    lane_follow_minSpeed = static_cast<tFloat32>(GetPropertyFloat("Lane Following::minimum speed"));
    weightFact_LaneFollow_HY = static_cast<tFloat32>(GetPropertyFloat("Lane Following::NMPC Weighting factor::High speed y"));
    weightFact_LaneFollow_LY = static_cast<tFloat32>(GetPropertyFloat("Lane Following::NMPC Weighting factor::Low speed y"));

    crossing_marker_distance = static_cast<tFloat32>(GetPropertyFloat("Crossing::Marker distance in cm"));
    crossing_stop_line_distance = static_cast<tFloat32>(GetPropertyFloat("Crossing::Stop Line distance in cm"));
    stop_crossing_stopline = static_cast<tFloat32>(GetPropertyFloat("Crossing::Stop Line stop distance in cm"));

    stop_crossing_left = static_cast<tFloat32>(GetPropertyFloat("Crossing::TurnLeft::stop crossing in cm"));
    nostop_crossing_left = static_cast<tFloat32>(GetPropertyFloat("Crossing::TurnLeft::no stopcrossing in cm"));
//     = static_cast<tFloat32>(GetPropertyFloat("Crossing::TurnLeft::turn around counter max 3*N"));
//     = static_cast<tFloat32>(GetPropertyFloat("Crossing::TurnLeft::open image counter max 3*N"));
    weightFact_TurnLeft_X = static_cast<tFloat32>(GetPropertyFloat("Crossing::TurnLeft::NMPC Weighting factor::x"));
    weightFact_TurnLeft_Y = static_cast<tFloat32>(GetPropertyFloat("Crossing::TurnLeft::NMPC Weighting factor::y"));

    stop_crossing_right = static_cast<tFloat32>(GetPropertyFloat("Crossing::TurnRight::stop crossing in cm"));
    nostop_crossing_right = static_cast<tFloat32>(GetPropertyFloat("Crossing::TurnRight::no stop crossing in cm"));

//     = static_cast<tFloat32>(GetPropertyFloat("Crossing::TurnRight::turn around counter max 2*N");
//     = static_cast<tFloat32>(GetPropertyFloat("Crossing::TurnRight::open image counter max 2*N";
    weightFact_TurnRight_X = static_cast<tFloat32>(GetPropertyFloat("Crossing::TurnRight::NMPC Weighting factor::x"));
    weightFact_TurnRight_Y = static_cast<tFloat32>(GetPropertyFloat("Crossing::TurnRight::NMPC Weighting factor::y"));

//     = static_cast<tFloat32>(GetPropertyFloat("Crossing::Straight::turn around counter max 4*N"));
//     = static_cast<tFloat32>(GetPropertyFloat("Crossing::Straight::open image counter max 4*N"));
    weightFact_Straight_X = static_cast<tFloat32>(GetPropertyFloat("Crossing::Straight::NMPC Weighting factor::x"));
    weightFact_Straight_Y = static_cast<tFloat32>(GetPropertyFloat("Crossing::Straight::NMPC Weighting factor::y"));

    crossing_left_distance = static_cast<tFloat32>(GetPropertyFloat("Crossing::ultra distance::1 Left section"));
    crossing_front_distance = static_cast<tFloat32>(GetPropertyFloat("Crossing::ultra distance::2 Front section"));
    crossing_right_distance = static_cast<tFloat32>(GetPropertyFloat("Crossing::ultra distance::3 Right section"));
    crossing_middle_distance = static_cast<tFloat32>(GetPropertyFloat("Crossing::ultra distance::4 Middle section"));

//     = static_cast<tFloat32>(GetPropertyFloat("Pullout::Left::turn around counter max 3*N"));
//     = static_cast<tFloat32>(GetPropertyFloat("Pullout::Left::open image counter max 3*N"));
    weightFact_PullOutLeft_X = static_cast<tFloat32>(GetPropertyFloat("Pullout::Left::NMPC Weighting factor::x"));
    weightFact_PullOutLeft_Y = static_cast<tFloat32>(GetPropertyFloat("Pullout::Left::NMPC Weighting factor::y"));

//     = static_cast<tFloat32>(GetPropertyFloat("Pullout::Right::turn around counter max 3*N");
//     = static_cast<tFloat32>(GetPropertyFloat("Pullout::Right::open image counter max 3*N");
    weightFact_PullOutRight_X = static_cast<tFloat32>(GetPropertyFloat("Pullout::Right::NMPC Weighting factor::x"));
    weightFact_PullOutRight_Y = static_cast<tFloat32>(GetPropertyFloat("Pullout::Right::NMPC Weighting factor::y"));

    weightFact_Avoidance_X = static_cast<tFloat32>(GetPropertyFloat("Avoidance::NMPC Weighting factor::x"));
    weightFact_Avoidance_Y = static_cast<tFloat32>(GetPropertyFloat("Avoidance::NMPC Weighting factor::y"));
    avoidance_initial_distance = static_cast<tFloat32>(GetPropertyFloat("Avoidance::Initial distance"));
    avoidance_laneChange_speed = static_cast<tFloat32>(GetPropertyFloat("Avoidance::Lane change speed"));
    avoidance_side_distance = static_cast<tFloat32>(GetPropertyFloat("Avoidance::Side detection distance"));
    avoidance_comeBack_counter = static_cast<tFloat32>(GetPropertyFloat("Avoidance::Come back counter"));


//    crossing_stopLine_mode = static_cast<tFloat32>(GetPropertyFloat("Avoidance::NMPC Weighting factor::y"));

    parking_marker_distance = static_cast<tFloat32>(GetPropertyFloat("Parking::Marker distance in cm"));
//     = static_cast<tFloat32>(GetPropertyFloat("Parking::Forward::turn around counter max 3*N");
//     = static_cast<tFloat32>(GetPropertyFloat("Parking::Backward::turn around counter max 6*N");
    slot0_distance = static_cast<tFloat32>(GetPropertyFloat("Parking::Slot distance 0"));
    slot1_distance = static_cast<tFloat32>(GetPropertyFloat("Parking::Slot distance 1"));
    slot2_distance = static_cast<tFloat32>(GetPropertyFloat("Parking::Slot distance 2"));
    slot3_distance = static_cast<tFloat32>(GetPropertyFloat("Parking::Slot distance 3"));
    slot4_distance = static_cast<tFloat32>(GetPropertyFloat("Parking::Slot distance 4"));
    weightFact_Parking_X = static_cast<tFloat32>(GetPropertyFloat("Parking::NMPC Weighting factor::x"));
    weightFact_Parking_Y = static_cast<tFloat32>(GetPropertyFloat("Parking::NMPC Weighting factor::y"));

    obstacle_detect_distance = static_cast<tFloat32>(GetPropertyFloat("Obstacles::detection distance"));

    carFollowing_detect_distance = static_cast<tFloat32>(GetPropertyFloat("Car following::detection distance"));
    carFollowing_preVeh_minSpeed = static_cast<tFloat32>(GetPropertyFloat("Car following::preceding vehicle minimum speed"));


    RETURN_NOERROR;
}
tResult SOP_AutonomousDriving::Start(__exception)
{
    car_position_first_flag = tFalse;
    current_car_state_flag = CAR_STOP;
    input_state_flag = CAR_STOP;
    car_speed = 0;
    output_steering = 0;
    car_curve_a = 0;
    car_curve_b = 0;
    stop_line_distance = 0;
    crossing_flag = 0; // tFalse
    parking_Ready_flag = 0;
    marker_update_counter = 0;
    stop_decision_flag = STOP_DECISION;
    avoidance_permit_flag = 0;
    last_speed = 0;
    last_steering = 0;

//    marker_read_counter = 0;
    avoidance.flag = tFalse;
    lane_change_left = 0;
    avoidance_straight_flag = 0;
    avoidance_back_flag = 0;

    MPC_sampling_rate_counter = 0;
    state_control_sampling_rate_counter = 0;

    road_marker_ID = NO_TRAFFIC_SIGN;

    speed_change_counter = 0;

    lane_follow_speed = lane_follow_minSpeed;
    no_lane_follow_speed = 0.5;

    pedestrian_stop_counter = 0;


    obstacle_from_Ultrasonic.x = 0;
    obstacle_from_Ultrasonic.y = 0;

    crossing_vehicle = NO_VEHICLES;
    crossing_vehicle_left = NO_VEHICLES;
    crossing_vehicle_right = NO_VEHICLES;
    crossing_vehicle_front = NO_VEHICLES;
    crossing_pedestrian = NO_PESDESTRIAN;

    front_min_break_distance =  min_break_distance;
    rear_min_break_distance  = -min_break_distance;

    // initialize translation and rotation vectors
    m_Tvec = Mat(3,1,CV_32F,Scalar::all(0));
    m_Rvec = Mat(3,1,CV_32F,Scalar::all(0));

    average_distance = 400;
    last_average_distance = 400;
    car_following_flag = tFalse;
    relative_speed = 0;
    pre_veh_speed = 2;


    adult_flag = 0;
    child_flag = 0;

    LOG_INFO(adtf_util::cString::Format("AutonomousDrivingFilter Star Front break time: %g S, Rear break time: %g S", front_min_break_time, rear_min_break_time));
    LOG_INFO(adtf_util::cString::Format("Sampling rate   State Control: %g mS, MPC: %gmS", state_control_sampling_rate, MPC_sampling_rate));

    m_bDebugModeEnabled = GetPropertyBool("Mode switch::Debug Output to Console");
    m_bJuryModelEnabled = GetPropertyBool("Mode switch::Jury Model on/off");
    crossing_stopLine_mode = GetPropertyBool("Mode switch::Stop line mode on/off");
    KI_adult = GetPropertyBool("KI switch::Adult on/off");
    KI_child = GetPropertyBool("KI switch::Child on/off");



    if (m_bDebugModeEnabled)
    {
        m_log = fopen("AutoDriv_Log.txt","w");
    }

    car_cur_position.X_Position   = 0;// - CAMERA_TO_CENTER;
    car_cur_position.Y_Position   = 0;
    car_cur_position.HeadingAngle = 0;
    temp_HeadingAngle = 0;


    image_processing_function_switch |= LANE_DETECTION;
    image_processing_function_switch |= STOP_LINE_DETECTION;

    light_flag[HEAD   ] = tFalse;
    light_flag[BRAKE  ] = tFalse;
    light_flag[REVERSE] = tFalse;
    light_flag[HAZARD ] = tFalse;
    light_flag[LEFT   ] = tFalse;
    light_flag[RIGHT  ] = tFalse;
    ToggleLights(HEAD, tFalse);
    ToggleLights(BRAKE, tFalse);
    ToggleLights(REVERSE, tFalse);
    ToggleLights(HAZARD, tFalse);
    ToggleLights(LEFT, tFalse);
    ToggleLights(RIGHT, tFalse);

    ToggleLights(HEAD, tTrue);
    light_flag[HEAD   ] = tTrue;
    pull_out_light_counter = 0;


    pedestrian_flag = tFalse;

    position_input_flag = tFalse;

    ultra_read_counter = 0;

    ManeuverList.id = 0;
    ManeuverList.id_counter = 0;
    ManeuverList.state = action_STOP;
    memset(ManeuverList.action, 0, 150 * 2 * sizeof(short));

    ManeuverList.send_ready_flag = tFalse;
    ManeuverList.stop_flag = tFalse;

    ResetDigitialMap();

    RETURN_IF_FAILED(SetIpopt());
    RETURN_IF_FAILED(cTimeTriggeredFilter::Start(__exception_ptr));

    RETURN_NOERROR;
}

tResult SOP_AutonomousDriving::Stop(__exception)
{
    if (m_bDebugModeEnabled)
        fclose(m_log);

    ToggleLights(HEAD, tFalse);
    ToggleLights(BRAKE, tFalse);
    ToggleLights(REVERSE, tFalse);
    ToggleLights(HAZARD, tFalse);
    ToggleLights(LEFT, tFalse);
    ToggleLights(RIGHT, tFalse);
    CloseIpopt();

    RETURN_IF_FAILED(cTimeTriggeredFilter::Stop(__exception_ptr));
    RETURN_NOERROR;
}
tResult SOP_AutonomousDriving::Init(tInitStage eStage, __exception)
{
    RETURN_IF_FAILED(cTimeTriggeredFilter::Init(eStage, __exception_ptr));

    if (eStage == StageFirst)
    {

        RETURN_IF_FAILED(CreateInputPins(__exception_ptr));
        RETURN_IF_FAILED(CreateOutputPins(__exception_ptr));

    }
    else if (eStage == StageNormal)
    {
        LoadConfiguration();
    }

    else if (eStage == StageGraphReady)
    {
        this->SetInterval(1000);  //cycle time 1 ms
    }

    RETURN_NOERROR;
}

tResult SOP_AutonomousDriving::CreateInputPins(__exception)
{
    // create description manager
    cObjectPtr<IMediaDescriptionManager> pDescManager;
    RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER, IID_ADTF_MEDIA_DESCRIPTION_MANAGER, (tVoid**)&pDescManager, __exception_ptr));


    tChar const * strUltrasonicStruct = pDescManager->GetMediaDescription("tUltrasonicStruct");
    RETURN_IF_POINTER_NULL(strUltrasonicStruct);
    cObjectPtr<IMediaType> pTypeUltrasonicStruct = new cMediaType(0, 0, 0, "tUltrasonicStruct", strUltrasonicStruct, IMediaDescription::MDF_DDL_DEFAULT_VERSION);


    RETURN_IF_FAILED(pTypeUltrasonicStruct->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionUsData));
    RETURN_IF_FAILED(m_oInputUssStruct.Create("UltrasonicStruct", pTypeUltrasonicStruct, static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_oInputUssStruct));


    tChar const * strLaneModelValue = pDescManager->GetMediaDescription("tLaneCurveData");
    RETURN_IF_POINTER_NULL(strLaneModelValue);
    cObjectPtr<IMediaType> pTypeLaneModel = new cMediaType(0, 0, 0, "tLaneCurveData", strLaneModelValue,IMediaDescription::MDF_DDL_DEFAULT_VERSION);

    RETURN_IF_FAILED(pTypeLaneModel->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&image_info_input.m_pDescription));
    RETURN_IF_FAILED(image_info_input.input.Create("LaneInfo", pTypeLaneModel, static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&image_info_input.input));


    image_info_ID_name[0] = "LaneDetectMode";
    image_info_ID_name[1] = "LandModel_k";
    image_info_ID_name[2] = "LandModel_m";
    image_info_ID_name[3] = "LandModel_b";
    image_info_ID_name[4] = "LaneWidth";
    image_info_ID_name[5] = "L_SL_LorR";
    image_info_ID_name[6] = "SolidlineFlag";
    image_info_ID_name[7] = "BiasWarn";
    image_info_ID_name[8] = "StopLineDistance";
    image_info_ID_name[9] = "Adult_flag";
    image_info_ID_name[10]= "Child_flag";


    // create the description for the position pin
    tChar const * strDescPosition = pDescManager->GetMediaDescription("tPosition");
    RETURN_IF_POINTER_NULL(strDescPosition);
    cObjectPtr<IMediaType> pTypePosition = new cMediaType(0, 0, 0, "tPosition", strDescPosition, IMediaDescription::MDF_DDL_DEFAULT_VERSION);

    // set the description for the extended marker pin
    RETURN_IF_FAILED(pTypePosition->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&position_input.m_pDescription));
    // create the position OutputPin
    RETURN_IF_FAILED(position_input.input.Create("Position", pTypePosition, this));
    RETURN_IF_FAILED(RegisterPin(&position_input.input));

    position_input_ID_name[0] = "f32x";
    position_input_ID_name[1] = "f32y";
    position_input_ID_name[2] = "f32radius";
    position_input_ID_name[3] = "f32speed";
    position_input_ID_name[4] = "f32heading";

    // get media tayp
    tChar const * strDescSignalValue = pDescManager->GetMediaDescription("tSignalValue");
    RETURN_IF_POINTER_NULL(strDescSignalValue);
    cObjectPtr<IMediaType> pTypeSignalValue = new cMediaType(0, 0, 0, "tSignalValue", strDescSignalValue,IMediaDescription::MDF_DDL_DEFAULT_VERSION);


    RETURN_IF_FAILED(pTypeSignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&wheel_speed_input.m_pDescription));
    RETURN_IF_FAILED(wheel_speed_input.input.Create("WheelSpeed", pTypeSignalValue, static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&wheel_speed_input.input));



    RETURN_IF_FAILED(pTypeSignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&distance_overall_input.m_pDescription));
    RETURN_IF_FAILED(distance_overall_input.input.Create("distance_overall", pTypeSignalValue, static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&distance_overall_input.input));


    tChar const * strIntValue = pDescManager->GetMediaDescription("tIntSignalValue");
    RETURN_IF_POINTER_NULL(strIntValue);
    cObjectPtr<IMediaType> pTypeIntValue = new cMediaType(0, 0, 0, "tIntSignalValue", strIntValue, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
    RETURN_IF_FAILED(pTypeIntValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&state_flag.m_pDescription));
    RETURN_IF_FAILED(state_flag.input.Create("StateFlag", pTypeIntValue, static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&state_flag.input));


    // create the description for the road sign Ext pin
    tChar const * strDescExt = pDescManager->GetMediaDescription("tRoadSignExt");
    RETURN_IF_POINTER_NULL(strDescExt);
    cObjectPtr<IMediaType> pTypeExt = new cMediaType(0, 0, 0, "tRoadSignExt", strDescExt, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
    RETURN_IF_FAILED(pTypeExt->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&input_road_sign_ext.m_pDescription));
    RETURN_IF_FAILED(input_road_sign_ext.input.Create("RoadSign_ext", pTypeExt, static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&input_road_sign_ext.input));

    // input jury struct
    tChar const * strDesc1 = pDescManager->GetMediaDescription("tJuryStruct");
    RETURN_IF_POINTER_NULL(strDesc1);
    cObjectPtr<IMediaType> pType1 = new cMediaType(0, 0, 0, "tJuryStruct", strDesc1, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
    RETURN_IF_FAILED(m_JuryStructInputPin.Create("Jury_Struct", pType1, this));
    RETURN_IF_FAILED(RegisterPin(&m_JuryStructInputPin));
    RETURN_IF_FAILED(pType1->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescJuryStruct));

    // input maneuver list
    tChar const * strDesc3 = pDescManager->GetMediaDescription("tManeuverList");
    RETURN_IF_POINTER_NULL(strDesc3);
    cObjectPtr<IMediaType> pType3 = new cMediaType(0, 0, 0, "tManeuverList", strDesc3, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
    RETURN_IF_FAILED(m_ManeuverListInputPin.Create("Maneuver_List", pType3, this));
    RETURN_IF_FAILED(RegisterPin(&m_ManeuverListInputPin));
    RETURN_IF_FAILED(pType3->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescManeuverList));




    RETURN_NOERROR;
}

tResult SOP_AutonomousDriving::CreateOutputPins(__exception)
{
    // create description manager
    cObjectPtr<IMediaDescriptionManager> pDescManager;
    RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER,IID_ADTF_MEDIA_DESCRIPTION_MANAGER,(tVoid**)&pDescManager,__exception_ptr));

    // Video Output
    RETURN_IF_FAILED(m_oVideoOutputPin.Create("Video_Visualization", IPin::PD_Output, static_cast<IPinEventSink*>(this)));
    RETURN_IF_FAILED(RegisterPin(&m_oVideoOutputPin));


    tChar const * strImageProcessControl = pDescManager->GetMediaDescription("tImageProcessControl");
    RETURN_IF_POINTER_NULL(strImageProcessControl);
    cObjectPtr<IMediaType> pTypeImagePorcessControl = new cMediaType(0, 0, 0, "tImageProcessControl", strImageProcessControl,IMediaDescription::MDF_DDL_DEFAULT_VERSION);

    RETURN_IF_FAILED(pTypeImagePorcessControl->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&image_processing_control.m_pDescription));
    RETURN_IF_FAILED(image_processing_control.output.Create("ImageControlStruct", pTypeImagePorcessControl, static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&image_processing_control.output));

    image_processing_control_ID_name[0] = "AutoControlMode";
    image_processing_control_ID_name[1] = "Reference_k";
    image_processing_control_ID_name[2] = "Reference_m";
    image_processing_control_ID_name[3] = "Reference_b";

    // get media tayp
    tChar const * strDescSignalValue = pDescManager->GetMediaDescription("tSignalValue");
    RETURN_IF_POINTER_NULL(strDescSignalValue);
    cObjectPtr<IMediaType> pTypeSignalValue = new cMediaType(0, 0, 0, "tSignalValue", strDescSignalValue,IMediaDescription::MDF_DDL_DEFAULT_VERSION);


    RETURN_IF_FAILED(pTypeSignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&steering_output.m_pDescription));
    RETURN_IF_FAILED(steering_output.output.Create("StreeingOutput", pTypeSignalValue, static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&steering_output.output));

    RETURN_IF_FAILED(pTypeSignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&speed_output.m_pDescription));
    RETURN_IF_FAILED(speed_output.output.Create("SpeedOutput", pTypeSignalValue, static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&speed_output.output));

    tChar const * strDescBoolSignalValue = pDescManager->GetMediaDescription("tBoolSignalValue");
    RETURN_IF_POINTER_NULL(strDescSignalValue);
    cObjectPtr<IMediaType> pTypeBoolSignalValue = new cMediaType(0, 0, 0, "tBoolSignalValue", strDescBoolSignalValue, IMediaDescription::MDF_DDL_DEFAULT_VERSION);

    RETURN_IF_FAILED(pTypeBoolSignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionBool));
    RETURN_IF_FAILED(m_oOutputHeadLight.Create("headLightEnabled", pTypeBoolSignalValue, static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_oOutputHeadLight));

    RETURN_IF_FAILED(m_oOutputBrakeLight.Create("brakeLightEnabled", pTypeBoolSignalValue, static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_oOutputBrakeLight));

    RETURN_IF_FAILED(m_oOutputReverseLight.Create("reverseLightEnabled", pTypeBoolSignalValue, static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_oOutputReverseLight));

    RETURN_IF_FAILED(m_oOutputTurnLeft.Create("turnSignalLeftEnabled", pTypeBoolSignalValue, static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_oOutputTurnLeft));

    RETURN_IF_FAILED(m_oOutputTurnRight.Create("turnSignalRightEnabled", pTypeBoolSignalValue, static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_oOutputTurnRight));

    RETURN_IF_FAILED(m_oOutputHazzard.Create("hazzardLightEnabled", pTypeBoolSignalValue, static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_oOutputHazzard));

    RETURN_IF_FAILED(position_initial_pin.output.Create("Position Initial Flag", pTypeBoolSignalValue, static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&position_initial_pin.output));



    // output driver struct
    tChar const * strDesc2 = pDescManager->GetMediaDescription("tDriverStruct");
    RETURN_IF_POINTER_NULL(strDesc2);
    cObjectPtr<IMediaType> pType2 = new cMediaType(0, 0, 0, "tDriverStruct", strDesc2, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
    RETURN_IF_FAILED(m_DriverStructOutputPin.Create("Driver_Struct", pType2, static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_DriverStructOutputPin));
    RETURN_IF_FAILED(pType2->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescDriverStruct));


    // output Obstacle
    tChar const * strObstacle = pDescManager->GetMediaDescription("tObstacle");
    RETURN_IF_POINTER_NULL(strObstacle);
    cObjectPtr<IMediaType> pTypeObstacle = new cMediaType(0, 0, 0, "tObstacle", strObstacle, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
    RETURN_IF_FAILED(pTypeObstacle->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&Obstacle_output.m_pDescription));
    RETURN_IF_FAILED(Obstacle_output.output.Create("Obstacle", pTypeObstacle, static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&Obstacle_output.output));


    // output ParkingSpace
    tChar const * strParkingSpace = pDescManager->GetMediaDescription("tParkingSpace");
    RETURN_IF_POINTER_NULL(strParkingSpace);
    cObjectPtr<IMediaType> pTypeParkingSpace = new cMediaType(0, 0, 0, "tParkingSpace", strParkingSpace, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
    RETURN_IF_FAILED(pTypeParkingSpace->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&ParkingSpace_output.m_pDescription));
    RETURN_IF_FAILED(ParkingSpace_output.output.Create("ParkingSpace", pTypeParkingSpace, static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&ParkingSpace_output.output));


    // create the description for the position pin
    tChar const * strDescEKFPosition = pDescManager->GetMediaDescription("tEKFPosition");
    RETURN_IF_POINTER_NULL(strDescEKFPosition);
    cObjectPtr<IMediaType> pTypeEKFPosition = new cMediaType(0, 0, 0, "tEKFPosition", strDescEKFPosition, IMediaDescription::MDF_DDL_DEFAULT_VERSION);

    // set the description for the extended marker pin
    RETURN_IF_FAILED(pTypeEKFPosition->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&EKF_position_output.m_pDescription));
    // create the position OutputPin
    RETURN_IF_FAILED(EKF_position_output.output.Create("EKF_Position", pTypeEKFPosition, this));
    RETURN_IF_FAILED(RegisterPin(&EKF_position_output.output));


    position_output_ID_name[0] = "f32EKFx";
    position_output_ID_name[1] = "f32EKFy";
    position_output_ID_name[2] = "f32EKFradius";
    position_output_ID_name[3] = "f32EKFspeed";
    position_output_ID_name[4] = "f32EKFheading";


    RETURN_NOERROR;
}

tResult SOP_AutonomousDriving::Shutdown(tInitStage eStage, __exception)
{
    RETURN_IF_FAILED(cTimeTriggeredFilter::Shutdown(eStage, __exception_ptr));

//    if (eStage == StageGraphReady)
//    {
//    }

    RETURN_NOERROR;
}

tResult SOP_AutonomousDriving::OnPinEvent(IPin* pSource, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample* pMediaSample)
{
    //mutex to prevent racing conditions for samples coming from different threads
    __synchronized_obj(m_oCritSectionInputData);


    if (nEventCode == IPinEventSink::PE_MediaSampleReceived)
    {
        RETURN_IF_POINTER_NULL(pMediaSample);

        if (pSource == &m_oInputUssStruct)
        {
            // LOG_INFO(adtf_util::cString::Format("OnpinEvent Uss-------------------"));

            RETURN_IF_FAILED(ProcessUssStructValue(pMediaSample, ultrasonic_value));
//                      LOG_INFO(adtf_util::cString::Format("Front L to R: %g, %g, %g, %g, %g", ultrasonic_value[0], ultrasonic_value[2], ultrasonic_value[4], ultrasonic_value[6], ultrasonic_value[8]));
            //          LOG_INFO(adtf_util::cString::Format("Side  L to R: %g, %g", ultrasonic_value[10], ultrasonic_value[12]));
            //          LOG_INFO(adtf_util::cString::Format("Rear  L to R: %g, %g, %g", ultrasonic_value[14], ultrasonic_value[16], ultrasonic_value[18]));
            CalculateUltrasonicWorldCoordinate(ultrasonic_value);
//            if (m_log) fprintf(m_log,"%g %g %g %g %g\n", ultrasonic_value[0], ultrasonic_value[2], ultrasonic_value[4], ultrasonic_value[6], ultrasonic_value[8]);
        }
        else if (pSource == &image_info_input.input)
        {
            //LOG_INFO(adtf_util::cString::Format("OnpinEvent Image-------------------"));

            ReadPinArrayValue(pMediaSample,&image_info_input, image_info_ID_name, 11, reference_value);
            stop_line_distance = reference_value[8];
            adult_flag = (int)reference_value[9];
            child_flag = (int)reference_value[10];

            // LOG_INFO(adtf_util::cString::Format("a = %g, b = %g, c = %g", reference_value[1],reference_value[2],reference_value[3]));
//            LOG_INFO(adtf_util::cString::Format("adult = %d, child = %d", adult_flag,child_flag));

        //    if(input_state_flag == LANE_FOLLOW)
            CalculateTrackingPoint();

            //          LOG_INFO(adtf_util::cString::Format("stop_line_distance = %g", stop_line_distance));
        }
        else if (pSource == &position_input.input)
        {
            // LOG_INFO(adtf_util::cString::Format("OnpinEvent Possition-------------------"));

            ReadPinArrayValue(pMediaSample,&position_input, position_input_ID_name, 5, position_value);

//            LOG_INFO(adtf_util::cString::Format("x:%f Y:%f Radius:%f Speed:%f Heading:%f", position_value[X], position_value[Y], position_value[RADIUS], position_value[SPEED], position_value[HEADING]));
//            if(car_speed != 0 || car_position_first_flag == tFalse)
//            {
//                car_cur_position.X_Position   = position_value[X];// - CAMERA_TO_CENTER;
//                car_cur_position.Y_Position   = position_value[Y];
//                car_cur_position.HeadingAngle = position_value[HEADING];
//                car_cur_position.radius       = position_value[RADIUS];
////                LOG_INFO(adtf_util::cString::Format("SOP_AutonomousDr position_value[X]    [Y] %f %f", position_value[X], position_value[Y]));
//                //                car_est_position.X_Position   = position_value[X];// - CAMERA_TO_CENTER;
//                //                car_est_position.Y_Position   = position_value[Y];
//                //                car_est_position.HeadingAngle = position_value[HEADING];

//                if(car_position_first_flag == tFalse && car_cur_position.HeadingAngle != 0)
//                    car_position_first_flag = tTrue;
//            }
            car_cur_position.X_Position   = position_value[X];// - CAMERA_TO_CENTER;
            car_cur_position.Y_Position   = position_value[Y];
            car_cur_position.HeadingAngle = position_value[HEADING];
            car_cur_position.radius       = position_value[RADIUS];
            if(position_input_flag == tFalse)
            {
                position_input_flag = tTrue;

                ResetExtendedKF();
                 ExtendedKF();
            }
        }
        else if (pSource == &wheel_speed_input.input)
        {
            //  LOG_INFO(adtf_util::cString::Format("OnpinEvent Speed-------------------"));

            //write values with zero
            f32Value = 0;
            Ui32TimeStamp = 0;
            {
               cObjectPtr<IMediaTypeDescription> wheel_speed_m_pDescription;
                wheel_speed_m_pDescription = wheel_speed_input.m_pDescription;
                // focus for sample write lock
                //read data from the media sample with the coder of the descriptor
                __adtf_sample_read_lock_mediadescription(wheel_speed_m_pDescription,pMediaSample,pCoder);

                if(!wheel_speed_input.ID_set)
                {
                    pCoder->GetID("f32Value", wheel_speed_input.ID_value);
                    pCoder->GetID("ui32ArduinoTimestamp", wheel_speed_input.ID_timestamp);
                    wheel_speed_input.ID_set = tTrue;
                }
                //get values from media sample
                pCoder->Get(wheel_speed_input.ID_value, (tVoid*)&f32Value);
                pCoder->Get(wheel_speed_input.ID_timestamp, (tVoid*)&Ui32TimeStamp);
            }

            // write to member variable
            car_speed = f32Value;
        }

        else if (pSource == &distance_overall_input.input)
        {
            //  LOG_INFO(adtf_util::cString::Format("OnpinEvent Speed-------------------"));

            //write values with zero
            f32Value = 0;
            Ui32TimeStamp = 0;
            {
                cObjectPtr<IMediaTypeDescription> distance_overall_m_pDescription;
                distance_overall_m_pDescription = distance_overall_input.m_pDescription;
                // focus for sample write lock
                //read data from the media sample with the coder of the descriptor
                __adtf_sample_read_lock_mediadescription(distance_overall_m_pDescription,pMediaSample,pCoder);

                if(!distance_overall_input.ID_set)
                {
                    pCoder->GetID("f32Value", distance_overall_input.ID_value);
                    pCoder->GetID("ui32ArduinoTimestamp", distance_overall_input.ID_timestamp);
                    distance_overall_input.ID_set = tTrue;
                }
                //get values from media sample
                pCoder->Get(distance_overall_input.ID_value, (tVoid*)&f32Value);
                pCoder->Get(distance_overall_input.ID_timestamp, (tVoid*)&Ui32TimeStamp);
            }

            // write to member variable
            distance_overall = f32Value;
        }

        else if (pSource == &input_road_sign_ext.input)
        {
            // process RoadSignExt sample

            RETURN_IF_FAILED(ProcessRoadSignStructExt(pMediaSample));
            marker_update_counter = 0;

        }

        else if (pSource == &state_flag.input && m_bJuryModelEnabled == tFalse)
        {
            // LOG_INFO(adtf_util::cString::Format("OnpinEvent State-------------------"));
            cObjectPtr<IMediaTypeDescription> state_flag_m_pDescription;
            state_flag_m_pDescription = state_flag.m_pDescription;
            // focus for sample write lock
            __adtf_sample_read_lock_mediadescription(state_flag_m_pDescription, pMediaSample, pCoder);


            if(state_flag.ID_set == tFalse)
            {
                pCoder->GetID("IntValue", state_flag.ID_value);
                state_flag.ID_set = tTrue;
            }
            pCoder->Get(state_flag.ID_value, (tVoid*)&input_state_flag);
            LOG_INFO(adtf_util::cString::Format("State Flag Input %d",input_state_flag));

            if(input_state_flag == CAR_STOP)
            {
                TransmitBoolValue(&position_initial_pin.output, tFalse, 0);
                current_car_state_flag = CAR_STOP;
                position_input_flag = tFalse;
            }
            else if(position_input_flag == tFalse)
                TransmitBoolValue(&position_initial_pin.output, tTrue, 0);


        }

        else if (pSource == &m_JuryStructInputPin && m_pDescJuryStruct != NULL)
        {
            tInt8 i8ActionID = -2;
            tInt16 i16entry = -1;

            {
                // focus for sample read lock
                __adtf_sample_read_lock_mediadescription(m_pDescJuryStruct,pMediaSample,pCoder);
                // get the IDs for the items in the media sample
                if(!m_bIDsJuryStructSet)
                {
                    pCoder->GetID("i8ActionID", m_szIDJuryStructI8ActionID);
                    pCoder->GetID("i16ManeuverEntry", m_szIDJuryStructI16ManeuverEntry);
                    m_bIDsJuryStructSet = tTrue;
                }

                pCoder->Get(m_szIDJuryStructI8ActionID, (tVoid*)&i8ActionID);
                pCoder->Get(m_szIDJuryStructI16ManeuverEntry, (tVoid*)&i16entry);
            }
            if(ManeuverList.id_counter != 0)
            {
                ManeuverList.state = i8ActionID;
                ManeuverList.id = i16entry;

                switch (ManeuverList.state)
                {
                case action_GETREADY:
                    if(ManeuverList.send_ready_flag == tFalse)
                    {
                        ManeuverList.send_ready_flag = tTrue;
                        TransmitBoolValue(&position_initial_pin.output, tTrue, 0);
                    }
//                    OnSendState(stateCar_READY, maneuver_ID);
                    LOG_INFO(cString::Format("Driver Module: Received Request Ready with maneuver ID %d",ManeuverList.id));
                    break;
                case action_START:

//                    if(ManeuverList.stop_flag == tTrue)
//                    {
//                        TransmitBoolValue(&position_initial_pin.output, tTrue, 0);
//                        ManeuverList.stop_flag = tFalse;
//                    }

                    if(ManeuverList.action[ManeuverList.id][0] == PULL_OUT_LEFT || ManeuverList.action[ManeuverList.id][0] == PULL_OUT_RIGHT)
                        current_car_state_flag = ManeuverList.action[ManeuverList.id][0];
                    else
                        current_car_state_flag = LANE_FOLLOW;
                    OnSendState(stateCar_RUNNING, ManeuverList.id);
                    LOG_INFO(cString::Format("Driver Module: Received Run with maneuver ID %d",ManeuverList.id));
                    break;
                case action_STOP:
                    current_car_state_flag = CAR_STOP;
                    output_steering = 0;
                    last_steering = 0;


                    if(position_input_flag == tTrue)
                    {
                        TransmitBoolValue(&position_initial_pin.output, tFalse, 0);
                        position_input_flag = tFalse;
                    }

                    if(ManeuverList.send_ready_flag == tFalse)
                    {
                        ManeuverList.send_ready_flag = tTrue;
                        TransmitBoolValue(&position_initial_pin.output, tTrue, 0);
                    }
//                    if(ManeuverList.stop_flag == tFalse)
//                        ManeuverList.stop_flag = tTrue;
//                    OnSendState(stateCar_READY, ManeuverList.id);
                    LOG_INFO(cString::Format("Driver Module: Received Stop with maneuver ID %d",ManeuverList.id));
                    break;
                }
            }
            else
                LOG_INFO(cString::Format("No maneuver List"));
        }
        else if (pSource == &m_ManeuverListInputPin && m_pDescManeuverList != NULL)
        {

            {
                // focus for sample read lock
                __adtf_sample_read_lock_mediadescription(m_pDescManeuverList,pMediaSample,pCoder);


                std::vector<tSize> vecDynamicIDs;

                // retrieve number of elements by providing NULL as first paramter
                tSize szBufferSize = 0;
                if(IS_OK(pCoder->GetDynamicBufferIDs(NULL, szBufferSize)))
                {
                    // create a buffer depending on the size element
                    tChar* pcBuffer = new tChar[szBufferSize];
                    vecDynamicIDs.resize(szBufferSize);
                    // get the dynamic ids (we already got the first "static" size element)
                    if (IS_OK(pCoder->GetDynamicBufferIDs(&(vecDynamicIDs.front()), szBufferSize)))
                    {
                        // iterate over all elements
                        for (tUInt32 nIdx = 0; nIdx < vecDynamicIDs.size(); ++nIdx)
                        {
                            // get the value and put it into the buffer
                            pCoder->Get(vecDynamicIDs[nIdx], (tVoid*)&pcBuffer[nIdx]);
                        }

                        // set the resulting char buffer to the string object
                        m_strManeuverFileString = (const tChar*) pcBuffer;
                    }

                    // cleanup the buffer
                    delete pcBuffer;
                }

            }
            LoadManeuverList();
        }


    }

    else if (nEventCode == IPinEventSink::PE_MediaTypeChanged)
    {

    }
    RETURN_NOERROR;
}


tResult SOP_AutonomousDriving::Cycle(__exception)
{

    if(state_control_sampling_rate_counter >= state_control_sampling_rate)
    {
        if(m_bJuryModelEnabled == tTrue && ManeuverList.state == action_START && position_input_flag == tTrue)
            current_car_state_flag = DrivingModeDecision(current_car_state_flag);
        else if(m_bJuryModelEnabled == tFalse &&  position_input_flag == tTrue)
            current_car_state_flag = DrivingModeDecision_TestModel();

//        LOG_INFO(adtf_util::cString::Format("Current State Flag %d",current_car_state_flag));
        if(current_car_state_flag == CAR_STOP)
            WriteSignalValue(&speed_output, 0, 0);

        ProcessVideo();

        state_control_sampling_rate_counter = 0;
    }

    else if(MPC_sampling_rate_counter >= MPC_sampling_rate)
    {

        AutoControl(current_car_state_flag);

//        image_processing_function_switch |= LANE_DETECTION;
//        image_processing_function_switch |= STOP_LINE_DETECTION;
//        image_processing_function_switch |= ADULT_DETECTION;
//        image_processing_function_switch |= CHILD_DETECTION;

//        image_processing_function_switch &= ~LANE_DETECTION;
//        image_processing_function_switch &= ~ADULT_DETECTION;

        image_processing_control_value[0] = (float)image_processing_function_switch;
        image_processing_control_value[1] = car_curve_a;
        image_processing_control_value[2] = car_curve_b;
        image_processing_control_value[3] = car_curve_c;
        WritePinArrayValue(&image_processing_control, 4,image_processing_control_ID_name, image_processing_control_value);

        MPC_sampling_rate_counter = 0;
    }


    if(ManeuverList.send_ready_flag == tTrue)
    {
//        if(position_input_flag == tTrue)
//            LOG_INFO(cString::Format("ManeuverList.id_counter %d   position_input_flag True",ManeuverList.id_counter));
//        else
//             LOG_INFO(cString::Format("ManeuverList.id_counter %d   position_input_flag False",ManeuverList.id_counter));
        if(ManeuverList.id_counter != 0 && position_input_flag == tTrue)
        {
            OnSendState(stateCar_READY, ManeuverList.id);
//             LOG_INFO(cString::Format("Send Ready"));
            ManeuverList.send_ready_flag = tFalse;
        }

    }


    if(marker_update_counter < 120)
        marker_update_counter++;

    MPC_sampling_rate_counter++;
    state_control_sampling_rate_counter++;


    RETURN_NOERROR;
}

tResult SOP_AutonomousDriving::ProcessUssStructValue(IMediaSample* pMediaSample, tFloat32 *output_value)
{
    tFloat32 buf_Value = 0;
    tUInt32 buf_TimeStamp = 0;


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
            if(buf_Value > 0)
            {
                pCoder->Get(m_szIDUltrasonicArduinoTimestamp[index], (tVoid*)&buf_TimeStamp);


                if(buf_Value <= 0)
                    output_value[(index * 2)] = 400;
                else
                    output_value[(index * 2)] = buf_Value;

//                if(buf_Value <= 0)
//                    output_value[(index * 2)] += 400;
//                else
//                    output_value[(index * 2)] += buf_Value;

//                output_value[(index * 2)] = output_value[(index * 2)] / 2;

                //output_value[(3 * 2)] = 400;

                output_value[(index * 2) + 1] = buf_TimeStamp;
            }

        }

    }

    RETURN_NOERROR;
}

tResult SOP_AutonomousDriving::ProcessVideo(void)
{
#define IMAGE_HEIGHT 460
#define IMAGE_WIDTH  430
#define IMAGE_HALF_HEIGHT (IMAGE_HEIGHT/2)
#define IMAGE_HALF_WIDTH  (IMAGE_WIDTH/2)

    //  __synchronized_obj(m_oCritSectionInputData);


    // new image for result
    cv::Mat outputImage;


    int row, col;
    int red = 0;
    int green = 0;
    int blue = 0;
    int index = 0;
    double y_coordinate = 0;
    double car_y_coordinate = 0;
    double distance2 = IMAGE_HALF_HEIGHT - 8;
    double distance = IMAGE_HALF_HEIGHT - 8;


    outputImage.create(IMAGE_HEIGHT, IMAGE_WIDTH, CV_8UC3);

    // __synchronized_obj(m_critSecImageData);

    for(row = 0; row < IMAGE_HEIGHT; row++)
    {
        if(row < IMAGE_HALF_HEIGHT - 8)
        {
            if(reference_value[0] == LTRACE || reference_value[0] == SL_TRACE)
            {
                y_coordinate = (reference_value[1] *(distance * distance)) + (reference_value[2] * distance) + reference_value[3];

                distance--;
            }
            car_y_coordinate = (car_curve_a *(distance2 * distance2)) + (car_curve_b * distance2);
            distance2--;
        }

        for(col = 0; col < IMAGE_WIDTH; col++)
        {
            red = green = blue = 0;

            if(row < IMAGE_HALF_HEIGHT - 8)
            {
                if(reference_value[0] == LTRACE)
                {
                    /*if(col == (int)(y_coordinate + (IMAGE_WIDTH/2)))
                    {
                        blue  = 255;  green = 128;  red   = 0;
                    }
                    else */if(col == (int)(y_coordinate + (reference_value[4] / 2)+ (IMAGE_WIDTH / 2)))
                    {
                        blue  = 128;  green = 128;  red   = 0;
                    }
                    else if(col == (int)((IMAGE_WIDTH / 2) + y_coordinate - (reference_value[4] / 2)))
                    {
                        blue  = 128;  green = 128;  red   = 0;
                    }
                }
                else if(reference_value[0] == SL_TRACE)
                {
                    /*if(col == (int)(y_coordinate + (IMAGE_WIDTH/2)))
                    {
                        blue  = 255;  green = 128;  red   = 0;
                    }
                    else */if(reference_value[5] == SL_Right &&  col == (int)(y_coordinate + (reference_value[4] / 2)+ (IMAGE_WIDTH / 2)))
                    {
                        blue  = 128;  green = 128;  red   = 0;
                    }
                    else if(reference_value[5] == SL_Right &&  col == (int)((IMAGE_WIDTH / 2) + y_coordinate - (reference_value[4] / 2)))
                    {
                        blue  = 128;  green = 128;  red   = 128;
                    }
                    else if(reference_value[5] == SL_Left && col == (int)((IMAGE_WIDTH / 2) + y_coordinate - (reference_value[4] / 2)))
                    {
                        blue  = 128;  green = 128;  red   = 0;
                    }
                    else if(reference_value[5] == SL_Left && col == (int)(y_coordinate + (reference_value[4] / 2)+ (IMAGE_WIDTH / 2)))
                    {
                        blue  = 128;  green = 128;  red   = 128;
                    }
                }

                if(col == (int)(((IMAGE_WIDTH/2) + car_y_coordinate) + (35/2)))
                {
                    blue  = 0;  green = 0;  red   = 255;
                }
                else if(col == (int)(((IMAGE_WIDTH/2) + car_y_coordinate) - (35 / 2)))
                {
                    blue  = 0;  green = 0;  red   = 255;
                }
            }


            outputImage.at<Vec3b>(row,col)[0] = blue;    //B
            outputImage.at<Vec3b>(row,col)[1] = green;   //G
            outputImage.at<Vec3b>(row,col)[2] = red;     //R
        }
    }

    //Draw Tracking Point
    float image_lane_X = 0;
    float image_lane_Y = 0;
    if(reference_value[0] == LTRACE || reference_value[0] == SL_TRACE)
    {
        for(index = 0; index <= N; index++)
        {
            image_lane_X =  ref_lane_coord_in_image.X[index];
            image_lane_Y =  ref_lane_coord_in_image.Y[index];

            image_lane_X = (IMAGE_HALF_HEIGHT - 8) - (image_lane_X);
            image_lane_Y = (image_lane_Y) + (IMAGE_WIDTH/2);


            if(image_lane_X > IMAGE_HEIGHT)
                image_lane_X = IMAGE_HEIGHT - 5;
            else if(image_lane_X < 0)
                image_lane_X = 5;

            if(image_lane_Y > IMAGE_WIDTH)
                image_lane_Y = IMAGE_WIDTH - 5;
            else if(image_lane_Y < 0)
                image_lane_Y = 5;

            circle(outputImage, Point(image_lane_Y,image_lane_X), 2, Scalar(0,128,255), -1);
        }
    }

    //Draw Rear Camera Tracking Point
    if(rear_camera_ref_value[0] == LTRACE || rear_camera_ref_value[0] == SL_TRACE)
    {
        for(index = 0; index <= N; index++)
        {
            image_lane_X =  rear_ref_lane_coord_in_image.X[index];
            image_lane_Y =  -rear_ref_lane_coord_in_image.Y[index];

            image_lane_X = (IMAGE_HALF_HEIGHT-8) + (image_lane_X);
            image_lane_Y = (image_lane_Y) + (IMAGE_WIDTH/2);


            if(image_lane_X > IMAGE_HEIGHT)
                image_lane_X = IMAGE_HEIGHT - 5;
            else if(image_lane_X < 0)
                image_lane_X = 5;

            if(image_lane_Y > IMAGE_WIDTH)
                image_lane_Y = IMAGE_WIDTH - 5;
            else if(image_lane_Y < 0)
                image_lane_Y = 5;

            circle(outputImage, Point(image_lane_Y,image_lane_X), 2, Scalar(0,128,255), -1);
        }
    }


    //Draw MAP range
    rectangle(outputImage, Point(0,0), Point(IMAGE_WIDTH-1,IMAGE_HEIGHT-1), Scalar(0,255,0), 2);

    //Draw Car Center
    //rectangle(outputImage, Point(IMAGE_WIDTH/2,0), Point(IMAGE_WIDTH/2+1,IMAGE_HEIGHT), Scalar(0,255,0), 1);

    //Draw Car
    rectangle(outputImage, Point(200,200), Point(230,260), Scalar(255,255,0), 2);
    rectangle(outputImage, Point(200,212), Point(204,222), Scalar(128,128,255), 1);  //Front Wheel Left
    rectangle(outputImage, Point(226,212), Point(229,222), Scalar(128,128,255), 1);  //Front Wheel Right
    rectangle(outputImage, Point(200,238), Point(204,248), Scalar(128,128,255), 1);  //Rear Wheel Left
    rectangle(outputImage, Point(226,238), Point(229,248), Scalar(128,128,255), 1);  //Rear Wheel Rifht
    char text[50];

    if(marker_update_counter < 100)
    {
        sprintf(text, "%d" ,road_marker_ID);
        putText(outputImage, text, Point((IMAGE_HALF_WIDTH - marker_lateral - 6), ((IMAGE_HALF_HEIGHT - 8) - marker_distance - 4)), 0, 0.3, Scalar(255,128,128),1);
        rectangle(outputImage, Point((IMAGE_HALF_WIDTH - marker_lateral - 7), ((IMAGE_HALF_HEIGHT - 8) - marker_distance - 14)), Point((IMAGE_HALF_WIDTH - marker_lateral + 7), ((IMAGE_HALF_HEIGHT - 8) - marker_distance)), Scalar(0,255,0), 1);
        sprintf(text, "%dcm" ,marker_distance);
        putText(outputImage, text, Point((IMAGE_HALF_WIDTH - marker_lateral + 10), ((IMAGE_HALF_HEIGHT - 8) - marker_distance - 4)), 0, 0.5, Scalar(255,255,255),1);
    }
    if(m_bJuryModelEnabled == tTrue && ManeuverList.id_counter != 0)
    {  
        switch (ManeuverList.action[ ManeuverList.id][0])
        {
            case PULL_OUT_LEFT:
                sprintf(text, "Running: %d pull_out_left", ManeuverList.id);
                break;
            case PULL_OUT_RIGHT:
                sprintf(text, "Running: %d pull_out_right", ManeuverList.id);
                break;
            case TURN_LEFT:
                sprintf(text, "Running: %d left", ManeuverList.id);
                break;
            case TURN_RIGHT:
                sprintf(text, "Running: %d right", ManeuverList.id);
                break;
            case STRAIGHT:
                sprintf(text, "Running: %d straight", ManeuverList.id);
                break;
            case PARKING:
                sprintf(text, "Running: %d cross_parking %d", ManeuverList.id, ManeuverList.action[ManeuverList.id][1]);
            case CAR_STOP:
                sprintf(text, "Running: %d STOP!!!", ManeuverList.id);
                break;
        }

        putText(outputImage, text, Point(10, IMAGE_HEIGHT - 20), 0, 0.5, Scalar(0,255,0),1);

    }
    else
        putText(outputImage, "NO Maneuver List!!", Point(10, IMAGE_HEIGHT - 20), 0, 0.5, Scalar(0,0,255),1);


    //    __synchronized_obj(m_critSecUltrasonicData);
    {
        //Draw Ultrasonic
        for(index = 0; index < 10; index++)
        {
            row =  (IMAGE_HEIGHT/2) - ult_world_coord[index][X];
            col = (IMAGE_WIDTH/2) + ult_world_coord[index][Y];


            if(row > IMAGE_HEIGHT)
                row = IMAGE_HEIGHT - 5;
            else if(row < 0)
                row = 5;

            if(col > IMAGE_WIDTH)
                col = IMAGE_WIDTH - 5;
            else if(col < 0)
                col = 5;

            if(index < 5)
            {
                if(ult_world_coord[index][X] < (front_min_break_distance + 30))
                {
                    float car_collision_y_range = 0;
                    car_collision_y_range = (car_curve_a * (ult_world_coord[index][X] * ult_world_coord[index][X])) + (ult_world_coord[index][X] * car_curve_b);

                    if(fabs(ult_world_coord[index][Y] - car_collision_y_range) < (35/2))
                        circle(outputImage, Point(col,row), 5, Scalar(0,0,255), -1);
                    else
                        circle(outputImage, Point(col,row), 3, Scalar(0,255,0), -1);
                }
                else
                    circle(outputImage, Point(col,row), 3, Scalar(0,255,0), -1);
            }
            else if(index > 6)
            {
                if(ult_world_coord[index][X] > (rear_min_break_distance - 30))
                    circle(outputImage, Point(col,row), 5, Scalar(0,0,255), -1);
                else
                    circle(outputImage, Point(col,row), 3, Scalar(0,255,0), -1);
            }
            else
                circle(outputImage, Point(col,row), 3, Scalar(0,255,0), -1);

        }
        //LOG_INFO(adtf_util::cString::Format(" %g    %g", front_min_break_distance, rear_min_break_distance));
    }




    if (!outputImage.empty())
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

        outputImage.release();
    }

    RETURN_NOERROR;
}

tResult SOP_AutonomousDriving::UpdateInputImageFormat(const tBitmapFormat* pFormat)
{
    if (pFormat != NULL)
    {

    }
    RETURN_NOERROR;
}

tResult SOP_AutonomousDriving::UpdateOutputImageFormat(const cv::Mat& outputImage)
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

tResult SOP_AutonomousDriving::WriteSignalValue(sop_pin_struct *pin, tFloat32 value, tUInt32 timestamp)
{
    //use mutex
    //__synchronized_obj(m_critSecTransmitControl);


    cObjectPtr<IMediaSample> pMediaSample;
    AllocMediaSample((tVoid**)&pMediaSample);

    cObjectPtr<IMediaSerializer> pSerializer;
    pin->m_pDescription->GetMediaSampleSerializer(&pSerializer);
    pMediaSample->AllocBuffer(pSerializer->GetDeserializedSize());

    //cObjectPtr<IMediaTypeDescription> m_pDescription;


    {
        cObjectPtr<IMediaTypeDescription> write_signal_m_pDescription;
        write_signal_m_pDescription = pin->m_pDescription;
        __adtf_sample_write_lock_mediadescription(write_signal_m_pDescription, pMediaSample, pCoderOutput);

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

tResult SOP_AutonomousDriving::WriteControlFlag(sop_pin_struct *pin, int value)
{
    //    if(current_car_state_flag != value)
    {
        cObjectPtr<IMediaSample> ppMediaSample;
        AllocMediaSample((tVoid**)&ppMediaSample);
        cObjectPtr<IMediaSerializer> pSerializer;
        pin->m_pDescription->GetMediaSampleSerializer(&pSerializer);
        ppMediaSample->AllocBuffer(pSerializer->GetDeserializedSize());


        {
            cObjectPtr<IMediaTypeDescription> write_control_m_pDescription;
            write_control_m_pDescription = pin->m_pDescription;

            __adtf_sample_write_lock_mediadescription(write_control_m_pDescription, ppMediaSample, pCoderOutput);

            if(pin->ID_set == tFalse)
            {
                pCoderOutput->GetID("IntValue", pin->ID_value);
                pin->ID_set = tTrue;
            }

            pCoderOutput->Set(pin->ID_value, (tVoid*)&value);

        }

        ppMediaSample->SetTime(_clock->GetStreamTime());
        pin->output.Transmit(ppMediaSample);

        // current_car_state_flag = value;
    }
    RETURN_NOERROR;
}

tResult SOP_AutonomousDriving::SendObstacle(sop_pin_struct *pin, float x, float y)
{

    tBufferID idValue;
    {
        cObjectPtr<IMediaSample> ppMediaSample;
        AllocMediaSample((tVoid**)&ppMediaSample);
        cObjectPtr<IMediaSerializer> pSerializer;
        pin->m_pDescription->GetMediaSampleSerializer(&pSerializer);
        ppMediaSample->AllocBuffer(pSerializer->GetDeserializedSize());


        {
            cObjectPtr<IMediaTypeDescription> send_obstacle_m_pDescription;
            send_obstacle_m_pDescription = pin->m_pDescription;

            __adtf_sample_write_lock_mediadescription(send_obstacle_m_pDescription, ppMediaSample, pCoderOutput);

            if(pin->ID_set == tFalse)
            {
                pCoderOutput->GetID("f32x", idValue);
                pin->ID_value_array.push_back(idValue);
                pCoderOutput->GetID("f32y", idValue);
                pin->ID_value_array.push_back(idValue);
                pin->ID_set = tTrue;
            }
            pCoderOutput->Set(pin->ID_value_array[0], (tVoid*)&x);
            pCoderOutput->Set(pin->ID_value_array[1], (tVoid*)&y);

        }

        ppMediaSample->SetTime(_clock->GetStreamTime());
        pin->output.Transmit(ppMediaSample);
    }
    RETURN_NOERROR;
}

tResult SOP_AutonomousDriving::SendParkingSpace(sop_pin_struct *pin, int parking_id, float x, float y, int parking_status)
{

    tBufferID idValue;
    {
        cObjectPtr<IMediaSample> ppMediaSample;
        AllocMediaSample((tVoid**)&ppMediaSample);
        cObjectPtr<IMediaSerializer> pSerializer;
        pin->m_pDescription->GetMediaSampleSerializer(&pSerializer);
        ppMediaSample->AllocBuffer(pSerializer->GetDeserializedSize());


        {
            cObjectPtr<IMediaTypeDescription> send_ParkingSpace_m_pDescription;
            send_ParkingSpace_m_pDescription = pin->m_pDescription;

            __adtf_sample_write_lock_mediadescription(send_ParkingSpace_m_pDescription, ppMediaSample, pCoderOutput);

            if(pin->ID_set == tFalse)
            {
                pCoderOutput->GetID("i16Identifier", idValue);
                pin->ID_value_array.push_back(idValue);
                pCoderOutput->GetID("f32x", idValue);
                pin->ID_value_array.push_back(idValue);
                pCoderOutput->GetID("f32y", idValue);
                pin->ID_value_array.push_back(idValue);
                pCoderOutput->GetID("ui16Status", idValue);
                pin->ID_value_array.push_back(idValue);
                pin->ID_set = tTrue;
            }
            pCoderOutput->Set(pin->ID_value_array[0], (tVoid*)&parking_id);
            pCoderOutput->Set(pin->ID_value_array[1], (tVoid*)&x);
            pCoderOutput->Set(pin->ID_value_array[2], (tVoid*)&y);
            pCoderOutput->Set(pin->ID_value_array[3], (tVoid*)&parking_status);

        }

        ppMediaSample->SetTime(_clock->GetStreamTime());
        pin->output.Transmit(ppMediaSample);
    }
    RETURN_NOERROR;
}

tResult SOP_AutonomousDriving::WritePinArrayValue(sop_pin_struct *pin, int number_of_array, cString *ID_name , tFloat32 *value)
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


tResult SOP_AutonomousDriving::ReadPinArrayValue(IMediaSample* input_pMediaSample, sop_pin_struct *input_pin, cString *PIN_ID_name, int number_of_array, tFloat32 *output_value)
{
    //use mutex
    //__synchronized_obj(m_critSecGetData);


    int index = 0;
    int id_set_index = 0;
    tFloat32 buf_Value = 0;
    tBufferID idValue;



    {
        cObjectPtr<IMediaTypeDescription> read_pin_m_pDescription;
        read_pin_m_pDescription = input_pin->m_pDescription;
        __adtf_sample_read_lock_mediadescription(read_pin_m_pDescription, input_pMediaSample, pCoderInput);

        for(index = 0; index < number_of_array; index++)
        {
            if(input_pin->ID_set == tFalse)
            {

                input_pin->ID_value_array.clear();

                for(id_set_index = 0; id_set_index < number_of_array; id_set_index++)
                {
                    pCoderInput->GetID(PIN_ID_name[id_set_index], idValue);
                    input_pin->ID_value_array.push_back(idValue);

                }
                input_pin->ID_set = tTrue;
            }
            pCoderInput->Get(input_pin->ID_value_array[index], (tVoid*)&buf_Value);
            output_value[index] = buf_Value;
        }
    }

    RETURN_NOERROR;
}

tResult SOP_AutonomousDriving::ProcessRoadSignStructExt(IMediaSample* pMediaSampleIn)
{
    {
        cObjectPtr<IMediaTypeDescription> Process_Road_m_pDescription;
        Process_Road_m_pDescription = input_road_sign_ext.m_pDescription;
        // focus for sample read lock
        // read-out the incoming Media Sample
        __adtf_sample_read_lock_mediadescription(Process_Road_m_pDescription, pMediaSampleIn, pCoder);

        // get IDs
        if(!input_road_sign_ext.ID_set)
        {
            pCoder->GetID("i16Identifier",m_szIDRoadSignExtI16Identifier);
            pCoder->GetID("f32Imagesize", m_szIDRoadSignExtF32Imagesize);
            pCoder->GetID("af32RVec[0]", m_szIDRoadSignExtAf32RVec);
            pCoder->GetID("af32TVec[0]", m_szIDRoadSignExtAf32TVec);
            input_road_sign_ext.ID_set = tTrue;
        }

        pCoder->Get(m_szIDRoadSignExtI16Identifier, (tVoid*)&road_marker_ID);
        pCoder->Get(m_szIDRoadSignExtF32Imagesize, (tVoid*)&m_f32MarkerSize);
        pCoder->Get("af32TVec", (tVoid*)m_Tvec.data);
        pCoder->Get("af32RVec", (tVoid*)m_Rvec.data);

    }

    marker_distance = short(m_Tvec.at<float>(2) * 100);    //Camera to Marker in cm
    marker_lateral  = -short(m_Tvec.at<float>(0) * 100);    //Camer to Marker center in cm

    RETURN_NOERROR;
}

void SOP_AutonomousDriving::ToggleLights(int buttonId, bool toggle)
{
    short toggle_flag = 0;

    if(toggle == tTrue)
        toggle_flag = 1;
    else
        toggle_flag = 0;

    light_flag[buttonId] = toggle;

    switch (buttonId)
    {
    case 0: // Head
        TransmitBoolValue(&m_oOutputHeadLight, toggle, 0);
//        LOG_INFO(cString::Format("Heads toggled: %d", toggle_flag));
        break;
    case 1: // Brake
        TransmitBoolValue(&m_oOutputBrakeLight, toggle, 0);
//        LOG_INFO(cString::Format("Brake toggled: %d", toggle_flag));
        break;
    case 2: // Reverse
        TransmitBoolValue(&m_oOutputReverseLight, toggle, 0);
//        LOG_INFO(cString::Format("Reverse toggled: %d", toggle_flag));
        break;
    case 3: // Hazard
        TransmitBoolValue(&m_oOutputHazzard, toggle, 0);
//        LOG_INFO(cString::Format("Hazard toggled: %d", toggle_flag));
        break;
    case 4: // Left
        TransmitBoolValue(&m_oOutputTurnLeft, toggle, 0);
//        LOG_INFO(cString::Format("Turn Left toggled: %d", toggle_flag));
        break;
    case 5: // Right
        TransmitBoolValue(&m_oOutputTurnRight, toggle, 0);
//        LOG_INFO(cString::Format("Turn right toggled: %d", toggle_flag));
        break;

    default:
        break;
    }
}

tResult SOP_AutonomousDriving::TransmitBoolValue(cOutputPin* oPin, bool value, tUInt32 timestamp)
{
    //use mutex
    //    __synchronized_obj(m_critSecTransmitBool);

    cObjectPtr<IMediaSample> pMediaSample;
    AllocMediaSample((tVoid**)&pMediaSample);

    cObjectPtr<IMediaSerializer> pSerializer;
    m_pDescriptionBool->GetMediaSampleSerializer(&pSerializer);
    pMediaSample->AllocBuffer(pSerializer->GetDeserializedSize());

    static bool hasID = false;
    static tBufferID szIDBoolValueOutput;
    static tBufferID szIDArduinoTimestampOutput;

    {
        __adtf_sample_write_lock_mediadescription(m_pDescriptionBool, pMediaSample, pCoderOutput);

        if(!hasID)
        {
            pCoderOutput->GetID("bValue", szIDBoolValueOutput);
            pCoderOutput->GetID("ui32ArduinoTimestamp", szIDArduinoTimestampOutput);
            hasID = tTrue;
        }

        pCoderOutput->Set(szIDBoolValueOutput, (tVoid*)&value);
        pCoderOutput->Set(szIDArduinoTimestampOutput, (tVoid*)&timestamp);
    }

    pMediaSample->SetTime(_clock->GetStreamTime());

    oPin->Transmit(pMediaSample);

    RETURN_NOERROR;
}

tResult SOP_AutonomousDriving::LoadManeuverList()
{
    m_sectorList.clear();
    // create dom from string received from pin
    cDOM oDOM;
    oDOM.FromString(m_strManeuverFileString);
    cDOMElementRefList oSectorElems;
    cDOMElementRefList oManeuverElems;


    ManeuverList.id_counter = 0;

    //read first Sector Elem
    if(IS_OK(oDOM.FindNodes("AADC-Maneuver-List/AADC-Sector", oSectorElems)))
    {
        //iterate through sectors
        for (cDOMElementRefList::iterator itSectorElem = oSectorElems.begin(); itSectorElem != oSectorElems.end(); ++itSectorElem)
        {
            //if sector found
            tSector sector;
            sector.id = (*itSectorElem)->GetAttributeUInt32("id");
            if(IS_OK((*itSectorElem)->FindNodes("AADC-Maneuver", oManeuverElems)))
            {
                //iterate through maneuvers
                for(cDOMElementRefList::iterator itManeuverElem = oManeuverElems.begin(); itManeuverElem != oManeuverElems.end(); ++itManeuverElem)
                {
                    tAADC_Maneuver man;
                    man.id = (*itManeuverElem)->GetAttributeUInt32("id");
                    man.action = (*itManeuverElem)->GetAttribute("action");
                    sector.maneuverList.push_back(man);

                    if(man.action == "pull_out_left")
                    {
                        ManeuverList.action[ManeuverList.id_counter][0] = PULL_OUT_LEFT;
                        ManeuverList.action[ManeuverList.id_counter][1] = 0;
                    }
                    else if(man.action == "pull_out_right")
                    {
                        ManeuverList.action[ManeuverList.id_counter][0] = PULL_OUT_RIGHT;
                        ManeuverList.action[ManeuverList.id_counter][1] = 0;
                    }
                    else if(man.action == "left")
                    {
                        ManeuverList.action[ManeuverList.id_counter][0] = TURN_LEFT;
                        ManeuverList.action[ManeuverList.id_counter][1] = 0;
                    }
                    else if(man.action == "right")
                    {
                        ManeuverList.action[ManeuverList.id_counter][0] = TURN_RIGHT;
                        ManeuverList.action[ManeuverList.id_counter][1] = 0;
                    }
                    else if(man.action == "straight")
                    {
                        ManeuverList.action[ManeuverList.id_counter][0] = STRAIGHT;
                        ManeuverList.action[ManeuverList.id_counter][1] = 0;
                    }
                    else if(man.action == "cross_parking 1")
                    {
                        ManeuverList.action[ManeuverList.id_counter][0] = PARKING;
                        ManeuverList.action[ManeuverList.id_counter][1] = 1;
                    }
                    else if(man.action == "cross_parking 2")
                    {
                        ManeuverList.action[ManeuverList.id_counter][0] = PARKING;
                        ManeuverList.action[ManeuverList.id_counter][1] = 2;
                    }
                    else if(man.action == "cross_parking 3")
                    {
                        ManeuverList.action[ManeuverList.id_counter][0] = PARKING;
                        ManeuverList.action[ManeuverList.id_counter][1] = 3;
                    }
                    else if(man.action == "cross_parking 4")
                    {
                        ManeuverList.action[ManeuverList.id_counter][0] = PARKING;
                        ManeuverList.action[ManeuverList.id_counter][1] = 4;
                    }
                    else if(man.action == "cross_parking 5")
                    {
                        ManeuverList.action[ManeuverList.id_counter][0] = PARKING;
                        ManeuverList.action[ManeuverList.id_counter][1] = 5;
                    }
                    else if(man.action == "cross_parking 6")
                    {
                        ManeuverList.action[ManeuverList.id_counter][0] = PARKING;
                        ManeuverList.action[ManeuverList.id_counter][1] = 6;
                    }
                    else if(man.action == "cross_parking 7")
                    {
                        ManeuverList.action[ManeuverList.id_counter][0] = PARKING;
                        ManeuverList.action[ManeuverList.id_counter][1] = 7;
                    }
                    else if(man.action == "cross_parking 8")
                    {
                        ManeuverList.action[ManeuverList.id_counter][0] = PARKING;
                        ManeuverList.action[ManeuverList.id_counter][1] = 8;
                    }
                    ManeuverList.id_counter++;
                }
            }

            m_sectorList.push_back(sector);
        }
    }

    ManeuverList.action[ManeuverList.id_counter][0] = CAR_STOP;
    ManeuverList.action[ManeuverList.id_counter][1] = 0;

    if (oSectorElems.size() > 0)
    {
        LOG_INFO("DriverFilter: Loaded Maneuver file successfully.");
    }
    else
    {
        LOG_ERROR("DriverFilter: no valid Maneuver Data found!");
        RETURN_ERROR(ERR_INVALID_FILE);
    }


    for (int index = 0; index < ManeuverList.id_counter; index++)
    {
        switch (ManeuverList.action[index][0])
        {
            case PULL_OUT_LEFT:
                LOG_INFO(adtf_util::cString::Format("Maneuver id: %d pull_out_left", index));
                break;
            case PULL_OUT_RIGHT:
                LOG_INFO(adtf_util::cString::Format("Maneuver id: %d pull_out_right", index));
                break;
            case TURN_LEFT:
                LOG_INFO(adtf_util::cString::Format("Maneuver id: %d left", index));
                break;
            case TURN_RIGHT:
                LOG_INFO(adtf_util::cString::Format("Maneuver id: %d right", index));
                break;
            case STRAIGHT:
                LOG_INFO(adtf_util::cString::Format("Maneuver id: %d straight", index));
                break;
            case PARKING:
                LOG_INFO(adtf_util::cString::Format("Maneuver id: %d cross_parking %d", index, ManeuverList.action[index][1]));
                break;
        }
    }

    RETURN_NOERROR;
}

tResult SOP_AutonomousDriving::OnSendState(stateCar stateID, tInt16 i16ManeuverEntry)
{
    cObjectPtr<IMediaSample> pMediaSample;
    RETURN_IF_FAILED(AllocMediaSample((tVoid**)&pMediaSample));

    cObjectPtr<IMediaSerializer> pSerializer;
    m_pDescDriverStruct->GetMediaSampleSerializer(&pSerializer);
    tInt nSize = pSerializer->GetDeserializedSize();

    tInt8 value = tInt8(stateID);

    RETURN_IF_FAILED(pMediaSample->AllocBuffer(nSize));
    {
        // focus for sample write lock
        __adtf_sample_write_lock_mediadescription(m_pDescDriverStruct,pMediaSample,pCoder);
        // get the IDs for the items in the media sample
        if(!m_bIDsDriverStructSet)
        {
            pCoder->GetID("i8StateID", m_szIDDriverStructI8StateID);
            pCoder->GetID("i16ManeuverEntry", m_szIDDriverStructI16ManeuverEntry);
            m_bIDsDriverStructSet = tTrue;
        }


        pCoder->Set(m_szIDDriverStructI8StateID, (tVoid*)&value);
        pCoder->Set(m_szIDDriverStructI16ManeuverEntry, (tVoid*)&i16ManeuverEntry);
    }

    pMediaSample->SetTime(_clock->GetStreamTime());
    m_DriverStructOutputPin.Transmit(pMediaSample);



    if(1)
    {
        switch (stateID)
        {
        case stateCar_READY:
            LOG_INFO(cString::Format("Driver Module: Send state: READY, Maneuver ID %d",i16ManeuverEntry));
            break;
        case stateCar_RUNNING:
            LOG_INFO(cString::Format("Driver Module: Send state: RUNNING, Maneuver ID %d",i16ManeuverEntry));
            break;
        case stateCar_COMPLETE:
            LOG_INFO(cString::Format("Driver Module: Send state: COMPLETE, Maneuver ID %d",i16ManeuverEntry));
            break;
        case stateCar_ERROR:
            LOG_INFO(cString::Format("Driver Module: Send state: ERROR, Maneuver ID %d",i16ManeuverEntry));
            break;
        case stateCar_STARTUP:
            LOG_INFO(cString::Format("Driver Module: Send state: STARTUP, Maneuver ID %d",i16ManeuverEntry));
            break;
        }
    }
    RETURN_NOERROR;
}

/*! support function for getting time */
tTimeStamp SOP_AutonomousDriving::GetTime()
{
    return adtf_util::cHighResTimer::GetTime();
}

/*!
 * loads road-sign configuration from a file, and stores into memory
 * */
tResult SOP_AutonomousDriving::LoadConfiguration()
{
    cFilename fileConfig = GetPropertyStr("Configuration");

    // create absolute path for marker configuration file
    ADTF_GET_CONFIG_FILENAME(fileConfig);
    fileConfig = fileConfig.CreateAbsolutePath(".");

    if (fileConfig.IsEmpty())
    {
        RETURN_ERROR(ERR_INVALID_FILE);
    }

    tInt i = 0;
    if (cFileSystem::Exists(fileConfig))
    {
        cDOM oDOM;
        oDOM.Load(fileConfig);
        cDOMElementRefList oElems;

        if(IS_OK(oDOM.FindNodes("configuration/stopLine", oElems)))
        {
            for (cDOMElementRefList::iterator itElem = oElems.begin(); itElem != oElems.end(); ++itElem)
            {
                stopLine item;
                item.f32X = tFloat32((*itElem)->GetAttribute("x","0").AsFloat64());
                item.f32Y = tFloat32((*itElem)->GetAttribute("y","0").AsFloat64());
                item.f32Direction = tFloat32((*itElem)->GetAttribute("direction","0").AsFloat64());

                item.u16Cnt = 0;
                item.u32ticks = GetTime();

//                item.f32Direction *= DEGREES_TO_RADIAN; // convert to radians

               // if (m_bDebugModeEnabled)
//                {
                    LOG_INFO(cString::Format("LoadConfiguration::XY %f %f Direction %f",
                         item.f32X, item.f32Y, item.f32Direction));
//                }

               m_stopLines.push_back(item);

               i++;

            }
        }
     }
    else
    {
        LOG_ERROR("Configuration file does not exist");
        RETURN_ERROR(ERR_INVALID_FILE);
    }


    RETURN_NOERROR;
}


tResult SOP_AutonomousDriving::ResetDigitialMap()
{

    T_section_boundary[0].left  = 8.544;
    T_section_boundary[0].right = 10.644;
    T_section_boundary[0].top   = 16.465;
    T_section_boundary[0].bom   = 14.515;
    T_section_boundary[0].HeadingAngle = 180;

    T_section_boundary[1].left  = 5.515;
    T_section_boundary[1].right = 7.465;
    T_section_boundary[1].top   = 13.456;
    T_section_boundary[1].bom   = 11.356;
    T_section_boundary[1].HeadingAngle = 90;

    T_section_boundary[2].left  = 5.515;
    T_section_boundary[2].right = 7.465;
    T_section_boundary[2].top   = 9.456;
    T_section_boundary[2].bom   = 7.356;
    T_section_boundary[2].HeadingAngle = 90;

    T_section_boundary[3].left  = -0.485;
    T_section_boundary[3].right = 1.465;
    T_section_boundary[3].top   = 9.456;
    T_section_boundary[3].bom   = 7.356;
    T_section_boundary[3].HeadingAngle = -90;

    T_section_boundary[4].left  = -0.485;
    T_section_boundary[4].right = 1.465;
    T_section_boundary[4].top   = 6.456;
    T_section_boundary[4].bom   = 4.356;
    T_section_boundary[4].HeadingAngle = -90;
    T_crossing_nummer = 5;


    avoidance_section_boundary[0].left  = 5.356;
    avoidance_section_boundary[0].right = 9.356;
    avoidance_section_boundary[0].top   = 0.985;
    avoidance_section_boundary[0].bom   = -0.025;

    avoidance_section_boundary[1].left  = 11.715;
    avoidance_section_boundary[1].right = 15.715;
    avoidance_section_boundary[1].top   = 9.644;
    avoidance_section_boundary[1].bom   = 4.356;
    avoidance_nummer = 2;

    pedestrian_section_boundary[0].left  = 2.644;
    pedestrian_section_boundary[0].right = 4.356;
    pedestrian_section_boundary[0].top   = 1.135;
    pedestrian_section_boundary[0].bom   = -0.135;

    pedestrian_section_boundary[1].left  = 2.865;
    pedestrian_section_boundary[1].right = 4.135;
    pedestrian_section_boundary[1].top   = 11.356;
    pedestrian_section_boundary[1].bom   = 9.644;
    pedestrian_nummer = 2;


    child_section_boundary[0].left  = 4.356;
    child_section_boundary[0].right = 9.356;
    child_section_boundary[0].top   = 0.985;
    child_section_boundary[0].bom   = -0.025;

    child_section_boundary[1].left  = 13.715;
    child_section_boundary[1].right = 14.715;
    child_section_boundary[1].top   = 8.644;
    child_section_boundary[1].bom   = 4.356;

    child_section_boundary[2].left  = -0.135;
    child_section_boundary[2].right = 1.135;
    child_section_boundary[2].top   = 10.644;
    child_section_boundary[2].bom   = 2.856;

    child_section_boundary[3].left  = 8.644;
    child_section_boundary[3].right = 12.644;
    child_section_boundary[3].top   = 16.135;
    child_section_boundary[3].bom   = 14.865;



    child_nummer = 4;

    RETURN_NOERROR;
}

