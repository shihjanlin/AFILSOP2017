#ifndef _SOP_AUTONOMOUS_DRIVING_H_
#define _SOP_AUTONOMOUS_DRIVING_H_  

#include "stdafx.h"
#include "ADTF_OpenCV_helper.h"
//#include "audi_q2_nlp.h"
//#include "IpIpoptApplication.hpp"
#include "Nmpc/parameter_settings.h"
#include <time.h>


#define OID_ADTF_FILTER_DEF "adtf.user_sop_autonomous_driving" //unique for a filter
#define ADTF_FILTER_DESC "SOP AutonomousDriving"  //this appears in the Component Tree in ADTF
#define ADTF_FILTER_VERSION_SUB_NAME "SOP_AutonomousDrivingFilter"//must match with accepted_version_...
#define ADTF_FILTER_VERSION_ACCEPT_LABEL "accepted_version"//sets the version entry
#define ADTF_FILTER_VERSION_STRING "1.0.0"//version string
#define ADTF_FILTER_VERSION_Major 1//this values will be compared, major version change - will not work
#define ADTF_FILTER_VERSION_Minor 0//change will work but notice
#define ADTF_FILTER_VERSION_Build 0//change will work but notice
//the detailed description of the filter
#define ADTF_FILTER_VERSION_LABEL "SOP Autonomous Driving Filter \n"


//#define AUTO_A


//#define OUTPUT_BEZIER_CURVE_DEBUG
//#define OUTPUT_TURN_AROUND_COUTER_DEBUG
//#define OUTPUT_REFERENCE_POINT_DEBUG
#define OUTPUT_EKF_DEBUG





#define PI 3.14159265359
#define RADIAN_TO_DEGREES (180.0 / PI)
#define DEGREES_TO_RADIAN (PI / 180.0)
enum COORDINATE {X ,Y, RADIUS, SPEED, HEADING};
enum ULTRASONIC {F_LEFT , F_CENTER_LEFT, F_CENTER, F_CENTER_RIGHT, F_RIGHT, S_LEFT, S_RIGHT, R_LEFT, R_CENTER, R_RIGHT};
enum DETECTIONMODE_t {LSEARCH, LTRACE, SL_SEARCH, SL_TRACE};
enum FindOneSideLane {SL_NotFound, SL_Left, SL_Right};
enum IMAGEP_ROCESSING {IMAGE_STOP, IMAGE_RUN};
enum CROSSING {CROSSING_FLAG_OFF, CROSSING_FLAG_STOP_LINE, CROSSING_FLAG_TRAFFIC_SIGNS};
enum PEDESTRIAN_STATE {NO_PESDESTRIAN, PEDESTRIAN_GOING, PEDESTRIAN_LEAVING, PEDESTRIAN_CHILDREN};
enum CROSSING_VEHICLE_STATE {NO_VEHICLES, VEHICLES_RIGHT, VEHICLES_LEFT, VEHICLES_FRONT, VEHICLES_THERE};
enum STOP_DECISION_STATE {STOP_DECISION, NOSTOP_DECISION};

enum LIGHT {HEAD, BRAKE, REVERSE, HAZARD, LEFT, RIGHT};
enum parkingSlot{slot1, slot2, slot3, slot4};

/*! the enum for the different states of the state machine (i.e. states of the car) */
enum stateCar{stateCar_ERROR = -1, stateCar_READY = 0, stateCar_RUNNING = 1, stateCar_COMPLETE = 2, stateCar_STARTUP = -2};
/*! the enum for the different states of the state machine (i.e. states of the car) */
enum juryActions{action_STOP = -1, action_GETREADY = 0, action_START = 1};




#define PARKING_READY_FLAG_OFF 0
#define PARKING_READY_FLAG_ON 1

#define UNMARKED_INTERSECTION    0
#define STOP_GIVE_WAY            1
#define PARKEN                   2
#define HAVEWAY                  3
#define GIVE_WAY                 5
#define PEDESTRIAN_CROSSING      6
#define TESTSTRECKER_A9          10
#define ROADWORKS                12
#define NO_TRAFFIC_SIGN          50


#define LANE_DETECTION       0x01
#define STOP_LINE_DETECTION  0x02
#define ADULT_DETECTION      0x04
#define CHILD_DETECTION      0x08


#define CAMERA_TO_CENTER        0.08                                 //The distance from the camera to the front in Meter
#define CAMERA_DISTANCE         18                                   //The distance from the camera to the front in cm
#define REAR_CAMERA_DISTANCE    37                                   //The distance from the front camera to the rear camera in cm



typedef struct _sop_pin_struct
{
    cInputPin input;
    cOutputPin output;

    cString   ID_name;
    tBufferID ID_value;
    tBufferID ID_timestamp;

    std::vector<tBufferID> ID_value_array;
    std::vector<tBufferID> ID_value_array2;
    std::vector<tBufferID> ID_Timestamp_array;

    tBool     ID_set;

    cObjectPtr<IMediaTypeDescription> m_pDescription;

} sop_pin_struct;


typedef struct _TURN_AROUND_REFERENCE_COORDINATE
{
    float X[100];
    float Y[100];

}TURN_AROUND_REFERENCE_COORDINATE;

typedef struct _CAR_POSITION_STRUCT
{
    tFloat32 X_Position;
    tFloat32 Y_Position;
    tFloat32 HeadingAngle;
    tFloat32 radius;

}CAR_POSITION_STRUCT;


typedef struct _OBSTACLE
{
    float left_boundary;
    float right_boundary;
    float width;
    float distance;
    float lateral;
    float x;
    float y;
    bool find_flag;
    short counter;

}OBSTACLE;

typedef struct _AVOIDANCE
{
    int step_counter;
    int dodge_distance;
    int dodge_lateral;
    int max_dodge_lateral;
    float last_overall_distance;
    bool flag;
    float last_lane_state;
    float last_SL_stae;
    short comeback_flag;
    int comeback_wait_counter;
}AVOIDANCE_STRUCT;


typedef struct _MANEUVER_LIST
{
    int id;
    int id_counter;
    short action[150][2];
    short state;
    int first_state;
    tBool send_ready_flag;
    tBool stop_flag;

}MANEUVER_LIST;

typedef struct _T_CROSSING_SECTION
{
    tFloat32 X_Position;
    tFloat32 Y_Position;
    tFloat32 HeadingAngle;
}T_CROSSING_SECTION;

typedef struct _SECTION_BOUNDARY
{
    tFloat32 left;
    tFloat32 right;
    tFloat32 top;
    tFloat32 bom;
    tFloat32 HeadingAngle;
}SECTION_BOUNDARY;

/*! struct for a maneuver */
struct tAADC_Maneuver
{
    /*! id of maneuver */
    int id;
    /*! action of maneuver */
    cString action;
};
/*! struct for a sector with maneuvers */
struct tSector
{
    /*! id of sector */
    int id;
    /*! list of maneuvers in sector */
    std::vector<tAADC_Maneuver> maneuverList;
};
/*! struct for a maneuver */
struct tSOP_Maneuver
{
    /*! id of action */
    int action_id;
    /*! action of maneuver */
    cString action;
};



/*! Storage structure for the road sign data */
typedef struct _stopLine
    {

        /*! location */
        tFloat32 f32X;
        tFloat32 f32Y;

        /*! direction (heading) of the road sign */
        tFloat32 f32Direction;

        tInt u16Cnt;

        tTimeStamp u32ticks;/*! measurement ticks*/

    } stopLine;

class SOP_AutonomousDriving : public cTimeTriggeredFilter
{

    /*! This macro does all the plugin setup stuff */
    ADTF_FILTER_VERSION(OID_ADTF_FILTER_DEF, ADTF_FILTER_DESC, adtf::OBJCAT_Auxiliary, ADTF_FILTER_VERSION_SUB_NAME, ADTF_FILTER_VERSION_Major, ADTF_FILTER_VERSION_Minor,ADTF_FILTER_VERSION_Build, ADTF_FILTER_VERSION_LABEL);
public:
    /*! default constructor for template class
        \param __info   [in] This is the name of the filter instance.
    */
    SOP_AutonomousDriving(const tChar* __info);

    /*! default destructor */
    virtual ~SOP_AutonomousDriving();

protected:

    //Input Signals
//    sop_pin_struct  sampling_rate_trigger;

    //Input pin for the ultrasonic front left data
    cInputPin       m_oInputUssStruct;
    cInputPin m_oInputInerMeasUnit;
    cObjectPtr<IMediaTypeDescription> m_pDescriptionUsData;
    cObjectPtr<IMediaTypeDescription> m_pDescriptionInerMeasUnitData;
    std::vector<tBufferID> m_szIDUltrasonicF32Value;
    std::vector<tBufferID>  m_szIDUltrasonicArduinoTimestamp;
    tBool m_bIDsUltrasonicSet;


    tFloat32        ultrasonic_value[20];
    tFloat32        ult_world_coord[10][2];

    sop_pin_struct  image_info_input;
    cString         image_info_ID_name[11];
    sop_pin_struct  position_input;
    cString         position_input_ID_name[5];
    cString         position_output_ID_name[5];


    sop_pin_struct  wheel_speed_input;
    sop_pin_struct  distance_overall_input;

    sop_pin_struct  state_flag;


    sop_pin_struct input_road_sign_ext;
    tBufferID m_szIDRoadSignExtI16Identifier;
    tBufferID m_szIDRoadSignExtF32Imagesize;
    tBufferID m_szIDRoadSignExtAf32TVec;
    tBufferID m_szIDRoadSignExtAf32RVec;

    tBool m_bDebugModeEnabled;
    tBool m_bJuryModelEnabled;
    FILE*m_log; // debug file

    /*! input pin for the run command*/
    cInputPin        m_JuryStructInputPin;
    /*! input pin for the maneuver list*/
    cInputPin        m_ManeuverListInputPin;
    /*! output pin for state from driver*/
    cOutputPin        m_DriverStructOutputPin;
    /*! Coder Descriptor */
    cObjectPtr<IMediaTypeDescription> m_pDescJuryStruct;
    /*! the id for the i8StateID of the media description */
    tBufferID m_szIDJuryStructI8ActionID;
    /*! the id for the i16ManeuverEntry of the media description data */
    tBufferID m_szIDJuryStructI16ManeuverEntry;
    /*! indicates if bufferIDs were set */
    tBool m_bIDsJuryStructSet;

    /*! Coder Descriptor */
    cObjectPtr<IMediaTypeDescription> m_pDescDriverStruct;
    /*! the id for the i8StateID of the media description */
    tBufferID m_szIDDriverStructI8StateID;
    /*! the id for the i16ManeuverEntry of the media description data */
    tBufferID m_szIDDriverStructI16ManeuverEntry;
    /*! indicates if bufferIDs were set */
    tBool m_bIDsDriverStructSet;

    /*! Coder description */
    cObjectPtr<IMediaTypeDescription> m_pDescManeuverList;

    /*! The maneuver file string */
    cString     m_strManeuverFileString;

    /*! this is the list with all the loaded sections from the maneuver list*/
    std::vector<tSector> m_sectorList;

    vector<stopLine> m_stopLines;


    //Output Signals
    sop_pin_struct steering_output;
    sop_pin_struct speed_output;
    cVideoPin      m_oVideoOutputPin;
    sop_pin_struct image_processing_control;
    cString        image_processing_control_ID_name[4];
    tFloat32       image_processing_control_value[4];
    char           image_processing_function_switch;
    sop_pin_struct reference_point;
    sop_pin_struct car_position_pin;
    sop_pin_struct position_initial_pin;
    sop_pin_struct Obstacle_output;
    sop_pin_struct ParkingSpace_output;
    sop_pin_struct EKF_position_output;

    /*! the media description for bool values */
    cObjectPtr<IMediaTypeDescription> m_pDescriptionBool;
    /*! The output pin for head light  */
    cOutputPin     m_oOutputHeadLight;
    /*! The output pin for reverse light */
    cOutputPin     m_oOutputReverseLight;
    /*! The output pin for brake light */
    cOutputPin     m_oOutputBrakeLight;
    /*! The output pin for turn right controller */
    cOutputPin     m_oOutputTurnRight;
    /*! The output pin for turn left controller */
    cOutputPin     m_oOutputTurnLeft;
    /*! The output pin for turn left controller */
    cOutputPin     m_oOutputHazzard;

    /*! EKF variables */
    Mat m_state; /*! filter state {X} */
    Mat m_errorCov; /*! error covariance matrix {P} */
    Mat m_processCov; /*! process covariance matrix {Q} */
    Mat m_transitionMatrix; /*! state transition matrix {F} */

    //Parameter
    tFloat32 reference_value[11];
    tFloat32 rear_camera_ref_value[7];
    tFloat32 position_value[5];
    tFloat32 output_steering;
    tFloat32 last_speed;
    tFloat32 last_steering;
    tFloat32 car_speed;
    tFloat32 distance_overall;
    tFloat32 last_distance_overall;
    tFloat32 last_distance_overall_for_crossing;
    tFloat32 last_distance_overall_for_parking;
    tFloat32 car_curve_a;
    tFloat32 car_curve_b;
    tFloat32 car_curve_c;
    tFloat32 stop_line_distance;
    int crossing_flag;
    int stop_decision_flag;
    int parking_Ready_flag;
    int avoidance_permit_flag;
    int crossing_vehicle_left;
    int crossing_vehicle_right;
    int crossing_vehicle_front;
    int crossing_vehicle;
    double temp_HeadingAngle;

    int lane_change_left;

    int crossing_pedestrian;
    int lane_follow_pedestrian;
    bool position_input_flag;

    int speed_change_counter;

    //Property setting parameters
    double lane_follow_maxSpeed;
    double lane_follow_minSpeed;

    float nostop_crossing_left;
    float nostop_crossing_right;
    float stop_crossing_left;
    float stop_crossing_right;
    float stop_crossing_stopline;

    double weightFact_LaneFollow_HY;
    double weightFact_LaneFollow_LY;
    double weightFact_TurnLeft_X;
    double weightFact_TurnLeft_Y;
    double weightFact_TurnRight_X;
    double weightFact_TurnRight_Y;
    double weightFact_Straight_X;
    double weightFact_Straight_Y;
    double weightFact_PullOutLeft_X;
    double weightFact_PullOutLeft_Y;
    double weightFact_PullOutRight_X;
    double weightFact_PullOutRight_Y;
    double weightFact_Parking_X;
    double weightFact_Parking_Y;
    double weightFact_Avoidance_X;
    double weightFact_Avoidance_Y;
    double crossing_left_distance;
    double crossing_front_distance;
    double crossing_right_distance;
    double crossing_middle_distance;
    double obstacle_detect_distance;
    double avoidance_initial_distance;
    double avoidance_laneChange_speed;
    double carFollowing_detect_distance;
    double carFollowing_preVeh_minSpeed;
    double avoidance_side_distance;
    double avoidance_comeBack_counter;

    float slot0_distance;
    float slot1_distance;
    float slot2_distance;
    float slot3_distance;
    float slot4_distance;

    int parking_marker_distance;
    int crossing_marker_distance;
    int crossing_stop_line_distance;

    tBool crossing_stopLine_mode;
    int pedestrian_stop_counter;
    int adult_flag;
    int child_flag;
    tBool KI_child;
    tBool KI_adult;

    int T_crossing_nummer;
    T_CROSSING_SECTION T_crossing_section[30];
    int avoidance_nummer;
    SECTION_BOUNDARY avoidance_section_boundary[30];
    int pedestrian_nummer;
    SECTION_BOUNDARY pedestrian_section_boundary[30];
    SECTION_BOUNDARY T_section_boundary[30];
    int child_nummer;
    SECTION_BOUNDARY child_section_boundary[30];


    double lane_follow_speed;
    double no_lane_follow_speed;

    int avoidance_straight_flag;
    int avoidance_back_flag;
    int current_car_state_flag;

    tFloat32 front_min_break_time;
    tFloat32 rear_min_break_time;
    tFloat32 min_break_distance;
    float front_min_break_distance;
    float rear_min_break_distance;

    OBSTACLE obstacle_from_Ultrasonic;
    AVOIDANCE_STRUCT avoidance;



    bool light_flag[6];
    int pull_out_light_counter;


    //bezier curve points
    float pt1[2];
    float pt2[2];
    float pt3[2];
    float pt4[2];
    float pt5[2];
    float pt6[2];
    float pt7[2];
    float pt8[2];


    MANEUVER_LIST ManeuverList;





    /*! currently processed road-sign */
    tInt16 road_marker_ID;
    tFloat32 m_f32MarkerSize;
    Mat m_Tvec; /*! translation vector */
    Mat m_Rvec; /*! rotation vector */
    short marker_distance;    //Camera to Marker in cm
    short marker_lateral;     //Camer to Marker center in cm
    short marker_update_counter;

    // ********* variables for calculate TTC ********* //
    short relative_distance[8];
    short last_relative_distance;
    double relative_speed;
    double pre_veh_speed;
    int average_speed_counter;
    short average_distance;
    short last_average_distance;
    tBool car_following_flag;
    // ******************//

    tFloat32 MPC_sampling_rate;
    tFloat32 state_control_sampling_rate;
    tFloat32 MPC_sampling_rate_counter;
    tFloat32 state_control_sampling_rate_counter;

    int turn_around_reference_counter;

    int input_state_flag;

    bool pedestrian_flag;
    int ultra_read_counter;



    tBool car_position_first_flag;
    COORDINATE_STRUCT ref_lane_world_coord;
    COORDINATE_STRUCT ref_lane_coord_in_image;
    COORDINATE_STRUCT rear_ref_lane_coord_in_image;
    COORDINATE_STRUCT ref_coord_in_MCP;

    CAR_POSITION_STRUCT car_cur_position;
    CAR_POSITION_STRUCT car_est_position;

    TURN_AROUND_REFERENCE_COORDINATE turn_left_ref_coord;
    TURN_AROUND_REFERENCE_COORDINATE turn_right_ref_coord;
    TURN_AROUND_REFERENCE_COORDINATE straight_ref_coord;
    TURN_AROUND_REFERENCE_COORDINATE pullout_left_ref_coord;
    TURN_AROUND_REFERENCE_COORDINATE pullout_right_ref_coord;
    TURN_AROUND_REFERENCE_COORDINATE avoidance_ref_coord;
    TURN_AROUND_REFERENCE_COORDINATE parking_ref_coord;

    tResult Init(tInitStage eStage, ucom::IException** __exception_ptr);

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

    tResult Cycle(ucom::IException** __exception_ptr = NULL);


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

    tResult PropertyChanged(const char* strProperty);


    /*!
     * Transmits bool value.
     *
     * \param [in,out]  pin         outputpin to use
     * \param           value       value to transmit
     * \param           timestamp   arduino timestamp to use
     *
     * \return standard adtf error code
     */
    tResult TransmitBoolValue(cOutputPin* pin, tBool value, tUInt32 timestamp);
    /*!
     * Toggle lights to output pin
     *
     * \param   buttonId    Identifier for the button.
     */
    void ToggleLights(int buttonId, bool toggle);



    tResult ProcessVideo(void);
    tResult SendObstacle(sop_pin_struct *pin, float x, float y);
    tResult CreateInputPins(ucom::IException** __exception_ptr = NULL);
    tResult CreateOutputPins(ucom::IException** __exception_ptr = NULL);
    tResult SendParkingSpace(sop_pin_struct *pin, int parking_id, float x, float y, int parking_status);
    tResult ReadPinArrayValue(IMediaSample* input_pMediaSample, sop_pin_struct *input_pin, cString *PIN_ID_name, int number_of_array, tFloat32 *output_value);
    tResult ProcessUssStructValue(IMediaSample* pMediaSample , tFloat32 *output_value);
    tResult ProcessRoadSignStructExt(IMediaSample* pMediaSampleIn);
    tResult CalculateUltrasonicWorldCoordinate(tFloat32 *ult_value);
    int DrivingModeDecision(int driving_mode_flag);
    int DrivingModeDecision_TestModel(void);
    tResult WriteReferencePoint(sop_pin_struct *pin, int number_of_array);
    tResult WriteCarPosition(sop_pin_struct *pin);
    tResult WritePinArrayValue(sop_pin_struct *pin, int number_of_array, cString *ID_name , tFloat32 *value);

    tResult WriteSignalValue(sop_pin_struct *pin, tFloat32 value, tUInt32 timestamp);
    tResult ResetDigitialMap();
    tResult LoadConfiguration();
    tTimeStamp GetTime();

    //Data_Processing.cpp
    tResult CalculateTrackingPoint(void);
    tResult CalculateTurnAroundReferencePoint(char left_or_right, int goal_coord_index);
    float GetDistanceBetweenCoordinates(float x2, float y2, float x1, float y1);


    //State_Control.cpp
    int CrossingDecision(int driving_mode_flag);
    int CrossingWaitTimeDecision(int marker_ID);
    int UltrasensorEmergencyBreak(int driving_mode_flag);
    int ParkingProcess(int driving_mode_flag);
    int PedestrianDecision(int driving_mode_flag);
    tResult CarFollowing(void);
    tResult SpeedDecision(void);
    int AvoidanceProcess(int driving_mode_flag);
    tResult ObstacleDetection(void);
    tResult ChildDetection();


    //NMPC Controller.cpp
    tResult AutoControl(int input_car_state_flag);
    tResult SetIpopt(void);
    tResult CalculateMPC(int input_car_state_flag, float direction);
    tResult ResetIpopt(void);
    tResult CloseIpopt(void);
    tResult ExtendedKF(void);
    tResult curvefitting(void);
    tResult cruiseSpeed(void);
    tResult ResetExtendedKF(void);



    tResult LoadManeuverList();
    tResult OnSendState(stateCar stateID, tInt16 i16ManeuverEntry);


    tResult WriteControlFlag(sop_pin_struct *pin, int value);

    /*! bitmap format of output pin */
    tBitmapFormat m_sOutputFormat;
    cCriticalSection m_oCritSectionInputData;
    cCriticalSection m_critSecTransmitControl;
    cCriticalSection m_critSecGetData;
    cCriticalSection m_critSecGetSpeed;
    cCriticalSection m_critSecMPCControl;
    cCriticalSection m_critSecPositionData;
    cCriticalSection m_critSecUltrasonicData;

    void initialize_bounds();
//    void collocation_matrix();
//    void collocation_matrix_2();
    void CalculateCoefficient(double *answer, double *x, double *y, int size_of_arrays, int degree_of_polynomial);

    //double dotX1(double *sVars, double *uu, double h);
    //double dotX2(double *sVars, double *uu, double h);
    //double dotX3(double *sVars, double *uu, double h);

};

/** @} */ // end of group

#endif  //_OPENCVTEMPLATE_FILTER_HEADER_
