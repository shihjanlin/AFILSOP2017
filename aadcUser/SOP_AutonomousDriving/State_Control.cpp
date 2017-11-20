#include "SOP_AutonomousDriving.h"

float temp_crossing_stop_distance = 0;


float left_nostop_crossing = -2;
float right_nostop_crossing = -11;
float nostop_crossing = 0;
float stop_crossing = 0;
float temp_parking_distance = 0;
float slot_distance[5];

int slot_num = 0;
float temp_ultrasonic_value_0 = 0;

int stopLine_check_flag = 0;


#define NO_BREAK_INDEX_FLAG 55


tInt16 temp_marker_ID;

int break_counter = 0;
int temp_mode_flag = NO_BREAK_INDEX_FLAG;
//short break_index_flag[8] = {0};
int car_stop_counter = 0;
int utral_sensor_counter = 0;

int left_section_counter = 0;
int right_section_counter = 0;
int front_section_counter = 0;
int in_section_counter = 0;
int out_section_counter = 0;
int crossing_stop_wait_counter = 0;

double vcc_max = 0.8;
double vcc_min = 0.5;



/* Function for decision making on vehicle maneuver mode:
 *  {CAR_STOP , LANE_FOLLOW, TURN_LEFT, TURN_RIGHT, STRAIGHT, PARKING, PULL_OUT_LEFT, PULL_OUT_RIGHT}
*/
int SOP_AutonomousDriving::DrivingModeDecision(int driving_mode_flag)
{


    if(marker_update_counter > 99)
    {
        road_marker_ID = NO_TRAFFIC_SIGN;
        marker_distance = 0;
        marker_lateral = 0;
    }

    ObstacleDetection();


    if(driving_mode_flag == LANE_FOLLOW || driving_mode_flag == CAR_STOP)
    {    
        if(car_speed == 0)
            lane_follow_speed = 0.5;
        else
            SpeedDecision();

        if(avoidance.comeback_flag == 0)
            CarFollowing();

        ChildDetection();

        driving_mode_flag = ParkingProcess(driving_mode_flag);

        if(parking_Ready_flag != PARKING_READY_FLAG_ON)
        {
            driving_mode_flag = CrossingDecision(driving_mode_flag);
            driving_mode_flag = PedestrianDecision(driving_mode_flag);
        }

    }





    //     LOG_INFO(adtf_util::cString::Format("AVOIDANCE comeback_flag %d wait flag %d driving_mode_flag %d", avoidance.comeback_flag , avoidance.comeback_wait_counter,  driving_mode_flag));

    if((driving_mode_flag == LANE_FOLLOW || driving_mode_flag == AVOIDANCE) && driving_mode_flag != PARKING)
        driving_mode_flag = AvoidanceProcess(driving_mode_flag);


    if (driving_mode_flag != AVOIDANCE || parking_Ready_flag != PARKING_READY_FLAG_ON /*&&  avoidance.flag != tTrue*/)
        driving_mode_flag = UltrasensorEmergencyBreak(driving_mode_flag);

    return driving_mode_flag;
}

int SOP_AutonomousDriving::AvoidanceProcess(int driving_mode_flag)
{
    /* Avoidance maneuver decision from ultrasensor
     *
     *
    */
    for(int index = 0; index < avoidance_nummer;index++)
    {
        if (car_est_position.X_Position < avoidance_section_boundary[index].right && car_est_position.X_Position > avoidance_section_boundary[index].left &&
            car_est_position.Y_Position < avoidance_section_boundary[index].top && car_est_position.Y_Position > avoidance_section_boundary[index].bom)
        {
            avoidance_permit_flag = 1;
            LOG_INFO(adtf_util::cString::Format("Avoidance permit "));
            break;
        }
        else
            avoidance_permit_flag = 0;
    }

    if (avoidance_permit_flag == 1)
    {
        LOG_INFO(adtf_util::cString::Format("Avoidance start %g",obstacle_from_Ultrasonic.distance));
        if(obstacle_from_Ultrasonic.find_flag == tTrue && obstacle_from_Ultrasonic.distance < 150/*&& obstacle_from_Ultrasonic.counter > 2*/)
        {
            if(avoidance.flag == tFalse)
            {

                avoidance.comeback_flag = 0;
                avoidance.flag = tTrue;
                driving_mode_flag = AVOIDANCE;
                LOG_INFO(adtf_util::cString::Format("Avoidance start %g",obstacle_from_Ultrasonic.distance));
                if(light_flag[LEFT] == tFalse)
                    ToggleLights(LEFT, tTrue);
            }
        }
    }


    if(avoidance.flag == tTrue)
    {
        //        lane_follow_speed = 0.5;
        if(ult_world_coord[S_RIGHT][Y] < (avoidance_side_distance + (15))  && avoidance.comeback_flag == 1)
        {
            avoidance.comeback_wait_counter = avoidance_comeBack_counter;
        }
        else if(ult_world_coord[S_RIGHT][Y] > (avoidance_side_distance + (15)) && avoidance.comeback_flag == 1 && driving_mode_flag == LANE_FOLLOW)
        {
            if(avoidance.comeback_wait_counter > 0)
                avoidance.comeback_wait_counter--;

            if(avoidance.comeback_wait_counter <= 0)
                avoidance.comeback_flag = 2;

            if(light_flag[RIGHT] == tFalse)
                ToggleLights(RIGHT, tTrue);
        }

        if(avoidance.comeback_flag == 2 )
        {

            avoidance.flag = tFalse;
            driving_mode_flag = AVOIDANCE;
        }
    }


    return driving_mode_flag;
}

tResult SOP_AutonomousDriving::ObstacleDetection(void)
{
    int index = 0;
    float temp_Ultrasonic[5][2];
    float car_collision_y_range = 0;

    memset(temp_Ultrasonic, 0, sizeof(temp_Ultrasonic));

    obstacle_from_Ultrasonic.left_boundary = -100;
    obstacle_from_Ultrasonic.right_boundary = 100;
    obstacle_from_Ultrasonic.width = 0;
    obstacle_from_Ultrasonic.distance = 200;
    obstacle_from_Ultrasonic.lateral = 0;
    obstacle_from_Ultrasonic.find_flag = tFalse;

    for(index = 0; index < 5; index++)
    {
        if(ult_world_coord[index][X] < (obstacle_detect_distance + (29))) //29cm is center shift to front//90
        {
            car_collision_y_range = (car_curve_a * (ult_world_coord[index][X] * ult_world_coord[index][X])) + (ult_world_coord[index][X] * car_curve_b);

            //            car_collision_y_range = (reference_value[1] *(ult_world_coord[index][X] * ult_world_coord[index][X])) + (reference_value[2] * ult_world_coord[index][X]) + reference_value[3];

            if(fabs(ult_world_coord[index][Y] - car_collision_y_range) < (35/2))
                //            if(ult_world_coord[index][Y] > (car_collision_y_range - (reference_value[4] / 2)) && ult_world_coord[index][Y] < (car_collision_y_range + (reference_value[4] / 2)))
            {
                temp_Ultrasonic[index][X] = ult_world_coord[index][X];
                temp_Ultrasonic[index][Y] = ult_world_coord[index][Y];
                obstacle_from_Ultrasonic.find_flag = tTrue;
                LOG_INFO(adtf_util::cString::Format("Obstacle find"));

            }


        }
    }

    if(obstacle_from_Ultrasonic.find_flag == tTrue)
    {
        for(index = 0; index < 5; index++)
        {
            if(temp_Ultrasonic[index][X] != 0)
            {
                if(temp_Ultrasonic[index][X] < obstacle_from_Ultrasonic.distance)
                    obstacle_from_Ultrasonic.distance = temp_Ultrasonic[index][X];

            }
        }

        for(index = 0; index < 5; index++)
        {
            if(temp_Ultrasonic[index][X] != 0)
            {
                obstacle_from_Ultrasonic.left_boundary = temp_Ultrasonic[index][Y];
                break;
            }
        }
        for(index = 4; index >= 0; index--)
        {
            if(temp_Ultrasonic[index][X] != 0)
            {
                obstacle_from_Ultrasonic.right_boundary = temp_Ultrasonic[index][Y];
                break;
            }
        }



        if(obstacle_from_Ultrasonic.left_boundary == obstacle_from_Ultrasonic.right_boundary)
        {
            obstacle_from_Ultrasonic.width = 5.0;
            obstacle_from_Ultrasonic.lateral = obstacle_from_Ultrasonic.left_boundary;
        }
        else
        {
            if(obstacle_from_Ultrasonic.left_boundary < 0 && obstacle_from_Ultrasonic.right_boundary < 0)
                obstacle_from_Ultrasonic.width = abs(obstacle_from_Ultrasonic.left_boundary - obstacle_from_Ultrasonic.right_boundary);
            else if(obstacle_from_Ultrasonic.left_boundary < 0 && obstacle_from_Ultrasonic.right_boundary == 0)
                obstacle_from_Ultrasonic.width = -obstacle_from_Ultrasonic.left_boundary;
            else if(obstacle_from_Ultrasonic.left_boundary == 0 && obstacle_from_Ultrasonic.right_boundary > 0)
                obstacle_from_Ultrasonic.width = obstacle_from_Ultrasonic.right_boundary - obstacle_from_Ultrasonic.left_boundary;
            else if(obstacle_from_Ultrasonic.left_boundary > 0 && obstacle_from_Ultrasonic.right_boundary > 0)
                obstacle_from_Ultrasonic.width = obstacle_from_Ultrasonic.right_boundary - obstacle_from_Ultrasonic.left_boundary;

            obstacle_from_Ultrasonic.lateral = obstacle_from_Ultrasonic.left_boundary + (obstacle_from_Ultrasonic.width / 2);
        }

        obstacle_from_Ultrasonic.distance = obstacle_from_Ultrasonic.distance + (29);


        if(obstacle_from_Ultrasonic.counter < 10)
            obstacle_from_Ultrasonic.counter++;



        if(obstacle_from_Ultrasonic.distance < 50 && (current_car_state_flag == LANE_FOLLOW || current_car_state_flag == AVOIDANCE || current_car_state_flag == EMERGENCY_BREAK))
        {
            float temp_x = obstacle_from_Ultrasonic.distance * 0.01;
            float temp_y = obstacle_from_Ultrasonic.lateral  * -0.01;
            obstacle_from_Ultrasonic.x = (cos(car_est_position.HeadingAngle) * temp_x) - (sin(car_est_position.HeadingAngle) * temp_y) + car_est_position.X_Position;
            obstacle_from_Ultrasonic.y = (sin(car_est_position.HeadingAngle) * temp_x) + (cos(car_est_position.HeadingAngle) * temp_y) + car_est_position.Y_Position;
            SendObstacle(&Obstacle_output, obstacle_from_Ultrasonic.x, obstacle_from_Ultrasonic.y);
        }
        else if(obstacle_from_Ultrasonic.x != 0 && obstacle_from_Ultrasonic.y != 0)
        {
            if(current_car_state_flag == LANE_FOLLOW && GetDistanceBetweenCoordinates(car_est_position.X_Position, car_est_position.Y_Position, obstacle_from_Ultrasonic.x, obstacle_from_Ultrasonic.y) < 0.05)
            {
                obstacle_from_Ultrasonic.x = 0;
                obstacle_from_Ultrasonic.y = 0;
                SendObstacle(&Obstacle_output, obstacle_from_Ultrasonic.x, obstacle_from_Ultrasonic.y);
            }
        }


        if(ultra_read_counter < 3)
        {
            relative_distance[ultra_read_counter] = obstacle_from_Ultrasonic.distance;
            ultra_read_counter ++;
        }
        else
        {
            last_average_distance = (relative_distance[0] + relative_distance[1] + relative_distance[2])/3;
            relative_distance[0] = relative_distance[1];
            relative_distance[1] = relative_distance[2];
            relative_distance[2] = obstacle_from_Ultrasonic.distance;
            average_distance = (relative_distance[0] + relative_distance[1] + relative_distance[2])/3;
            relative_speed = (last_average_distance/100.0 - average_distance/100.0) / (state_control_sampling_rate/1000);
            //LOG_INFO(adtf_util::cString::Format("*********************"));
            //            LOG_INFO(adtf_util::cString::Format("Average speed %g",relative_speed));
            //            LOG_INFO(adtf_util::cString::Format("Real speed %g",car_speed));
            //LOG_INFO(adtf_util::cString::Format("Average distance %d",average_distance));
            pre_veh_speed = lane_follow_speed - relative_speed;
            LOG_INFO(adtf_util::cString::Format("Preceding vehicle speed %g",pre_veh_speed));
        }
    }
    else
    {
        obstacle_from_Ultrasonic.counter = 0;
        ultra_read_counter = 0;
        average_distance = 400;
        relative_speed = 0;
        pre_veh_speed = 2;
    }
    RETURN_NOERROR;
}

tResult SOP_AutonomousDriving::SpeedDecision(void)
{

    // correct speed function !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    if(crossing_flag != CROSSING_FLAG_OFF) //in curve of single lane
    {
        if (lane_follow_speed < 0.4)
            lane_follow_speed = 0.4;
        else
            lane_follow_speed = lane_follow_speed - ((lane_follow_speed - 0.4)/5);
    }
    else
    {
        if (lane_follow_speed < lane_follow_minSpeed)
            lane_follow_speed = lane_follow_speed + ( lane_follow_minSpeed - lane_follow_speed)/10 + 0.01;
        else
        {
            if(fabs(reference_value[1])<0.0005 && reference_value[0] == LTRACE && fabs(reference_value[3])<5) // straight and two lanes recoginzed
            {
                if (lane_follow_speed >= lane_follow_maxSpeed - 0.01)
                    lane_follow_speed = lane_follow_maxSpeed;
                else
                    lane_follow_speed = lane_follow_speed + sin((lane_follow_maxSpeed-lane_follow_speed)*PI/(lane_follow_maxSpeed-lane_follow_minSpeed)- 0.01)*0.015;
            }
            else //in curve of single lane
            {
                if (lane_follow_speed <= lane_follow_minSpeed + 0.01)
                    lane_follow_speed = lane_follow_minSpeed;
                else
                    lane_follow_speed = lane_follow_speed - sin((lane_follow_speed-lane_follow_minSpeed)*PI/(lane_follow_maxSpeed-lane_follow_minSpeed)- 0.01)*0.04;
            }
        }
    }
    RETURN_NOERROR;
}

tResult SOP_AutonomousDriving::ChildDetection()
{
    int index = 0;
    tBool  detection_flag = tFalse;

    float temp_HeadingAngle = car_est_position.HeadingAngle;
    if (car_est_position.HeadingAngle >= -PI/4 && car_est_position.HeadingAngle <= PI/4 )
        temp_HeadingAngle = 0;
    else if (car_est_position.HeadingAngle >= PI/4 && car_est_position.HeadingAngle <= 3*PI/4 )
        temp_HeadingAngle = 90;
    else if (car_est_position.HeadingAngle >= -3*PI/4  && car_est_position.HeadingAngle <= -PI/4 )
        temp_HeadingAngle = -90;
    else
        temp_HeadingAngle = 180;

    for(index = 0; index < child_nummer; index++)
    {
        tFloat32 Tx = (car_est_position.X_Position+cos(temp_HeadingAngle*DEGREES_TO_RADIAN)*0.5);
        tFloat32 Ty = (car_est_position.Y_Position+sin(temp_HeadingAngle*DEGREES_TO_RADIAN)*0.5);
        if (Tx < child_section_boundary[index].right && Tx > child_section_boundary[index].left &&
            Ty < child_section_boundary[index].top && Ty > child_section_boundary[index].bom)
        {
            detection_flag = tTrue;
            //bLOG_INFO(adtf_util::cString::Format("Child Detection On"));
            break;
        }
        else
        {
            detection_flag = tFalse;
        }
    }


    if(detection_flag == tTrue)
    {
//        LOG_INFO(adtf_util::cString::Format("Child Detection On"));
        image_processing_function_switch |= CHILD_DETECTION;

        if(child_flag == 1)
            lane_follow_speed = 0.4;
    }
    else
    {
//        LOG_INFO(adtf_util::cString::Format("Child Detection Off"));
        image_processing_function_switch &= ~CHILD_DETECTION;
    }

    RETURN_NOERROR;
}

int SOP_AutonomousDriving::PedestrianDecision(int driving_mode_flag)
{
//    int index = 0;

    if (road_marker_ID == PEDESTRIAN_CROSSING && marker_distance < 70)
    {
        temp_crossing_stop_distance = marker_distance;
        last_distance_overall_for_crossing = distance_overall;
        lane_follow_speed = 0.4;
        pedestrian_flag = tTrue;
    }

    else if (pedestrian_flag == tTrue)
    {
        if(KI_adult)
            image_processing_function_switch |= ADULT_DETECTION;
        if(KI_child)
            image_processing_function_switch |= CHILD_DETECTION;
        lane_follow_speed = 0.4;


        temp_crossing_stop_distance -= ((distance_overall - last_distance_overall_for_crossing) * 100);
        last_distance_overall_for_crossing = distance_overall;

        if(adult_flag == 2 && KI_adult == tTrue)
        {
            if(temp_crossing_stop_distance < 15)
                driving_mode_flag = CAR_STOP;
            pedestrian_stop_counter = 20;
        }
        else if(child_flag == 1 && KI_child == tTrue)
        {
            if(temp_crossing_stop_distance < 15)
                driving_mode_flag = CAR_STOP;
            pedestrian_stop_counter = 20;
        }


        pedestrian_stop_counter--;
        if(pedestrian_stop_counter <= 0 && driving_mode_flag == CAR_STOP)
        {
            driving_mode_flag = LANE_FOLLOW;
            lane_follow_speed = 0.4;
        }

        if(temp_crossing_stop_distance < -50)
        {
            pedestrian_flag = tFalse;
            image_processing_function_switch &= ~ADULT_DETECTION;
            image_processing_function_switch &= ~CHILD_DETECTION;
        }
    }
    return driving_mode_flag;
}



int SOP_AutonomousDriving::ParkingProcess(int driving_mode_flag)
{
    //    LOG_INFO(adtf_util::cString::Format("**** Maneuver ID %d ****", maneuver_ID ));
    //    LOG_INFO(adtf_util::cString::Format("**** Road mark iD %d ****", road_marker_ID ));
    //    LOG_INFO(adtf_util::cString::Format("**** Mark distance %d ****", marker_distance ));
    short parking_id = 0;


    if(parking_Ready_flag == PARKING_READY_FLAG_ON)
    {
        lane_follow_speed = 0.4;

        //Parking Distance
        temp_parking_distance -= ((distance_overall - last_distance_overall_for_parking) * 100);
        last_distance_overall_for_parking = distance_overall;



        //Parking Distance Process
        //LOG_INFO(adtf_util::cString::Format("**** Temp parking distance %g ****", temp_parking_distance));
        //LOG_INFO(adtf_util::cString::Format("**** slot beging distance %g ****", slot_distance[slot_num-1] - 10));
        //LOG_INFO(adtf_util::cString::Format("**** slot end distance %g ****", slot_distance[slot_num] + 10));


        if( ManeuverList.action[ManeuverList.id][1] > 4)
            parking_id =  ManeuverList.action[ManeuverList.id][1] - 4;
        else
            parking_id =  ManeuverList.action[ManeuverList.id][1];

        if( temp_parking_distance <= slot_distance[parking_id] )
        {
            LOG_INFO(adtf_util::cString::Format("**** parking_id %d %d ****", parking_id, ManeuverList.action[ManeuverList.id][1] ));

            driving_mode_flag = PARKING;
            parking_Ready_flag = PARKING_READY_FLAG_OFF;
            temp_parking_distance = 0;
        }

    }

    //parking decision
    else if(ManeuverList.action[ManeuverList.id][0] == PARKING && road_marker_ID == PARKEN && marker_distance < parking_marker_distance)//70
    {
        lane_follow_speed = 0.4;

        if(light_flag[RIGHT] == tFalse)
            ToggleLights(RIGHT, tTrue);

        temp_parking_distance = marker_distance;
        parking_Ready_flag = PARKING_READY_FLAG_ON;
        last_distance_overall_for_parking = distance_overall;

        //parking slot

        slot_distance[0] = slot0_distance;//8/47//57

        slot_distance[1] = slot4_distance;//8/47//57

        slot_distance[2] = slot3_distance;//0

        slot_distance[3] = slot2_distance;//-50

        slot_distance[4] = slot1_distance;//-99


        LOG_INFO(adtf_util::cString::Format("**** Parking recognized ****"));
    }



    return driving_mode_flag;
}

int SOP_AutonomousDriving::UltrasensorEmergencyBreak(int driving_mode_flag)
{
    int index = 0;
    //  LOG_INFO(adtf_util::cString::Format("(EmergencyBreak) Driving Flag Input %d",driving_mode_flag));
    if(car_speed > 0.0)
    {

        front_min_break_distance = (car_speed * front_min_break_time) * 100;  //cm
        if(front_min_break_distance < min_break_distance)
            front_min_break_distance = min_break_distance;
    }
    else if(car_speed < 0.0)
    {
        rear_min_break_distance =  (car_speed * rear_min_break_time) * 100;
        if(rear_min_break_distance > -min_break_distance)
            rear_min_break_distance = -min_break_distance;
    }
    else if(car_speed == 0.0)
    {
        front_min_break_distance =  min_break_distance;
        rear_min_break_distance  = -min_break_distance;
    }

    if(car_speed > 0)
    {
        float car_collision_y_range = 0;

        for(index = 0; index < 5; index++)
        {
            if(ult_world_coord[index][X] < (front_min_break_distance + 30))
            {
                car_collision_y_range = (car_curve_a * (ult_world_coord[index][X] * ult_world_coord[index][X])) + (ult_world_coord[index][X] * car_curve_b);
                if(fabs(ult_world_coord[index][Y] - car_collision_y_range) < (35/2))
                {
                    if(temp_mode_flag == NO_BREAK_INDEX_FLAG)
                        temp_mode_flag = driving_mode_flag;
                    break_counter = 30;
                    //   LOG_INFO(adtf_util::cString::Format("(EmergencyBreak) *************Sensor counter %d",break_counter));
                }
            }
        }

    }

    else if(break_counter != 0)
    {
        //LOG_INFO(adtf_util::cString::Format("(EmergencyBreak) *************Car Stop"));
        float car_collision_y_range = 0;
        car_collision_y_range = (car_curve_a * ((front_min_break_distance + 30) * (front_min_break_distance + 30))) + ((front_min_break_distance + 30) * car_curve_b);
        for(index = 0; index < 5; index++)
        {
            if(ult_world_coord[index][X] < (front_min_break_distance + 30) && fabs(ult_world_coord[index][Y] - car_collision_y_range) < (35/2))
            {
                break_counter = 30;
                //   LOG_INFO(adtf_util::cString::Format("(EmergencyBreak) *************driving Flag Input %d last flag %d",driving_mode_flag, temp_mode_flag));
                //                    break_index_flag = NO_BREAK_INDEX_FLAG;
                //                    driving_mode_flag = temp_mode_flag;
            }
        }
        break_counter--;
    }



    if(break_counter != 0 && break_counter > 0)
    {
        driving_mode_flag = EMERGENCY_BREAK;

        if(light_flag[BRAKE] == tFalse)
            ToggleLights(BRAKE, tTrue);

        image_processing_function_switch &= ~LANE_DETECTION;
        LOG_INFO(adtf_util::cString::Format("(EmergencyBreak) Sensor Flag Input %d",break_counter));
    }
    else if(break_counter <= 0)
    {
        if(temp_mode_flag != NO_BREAK_INDEX_FLAG)
        {
            image_processing_function_switch |= LANE_DETECTION;
            image_processing_function_switch |= STOP_LINE_DETECTION;
            image_processing_function_switch &= ~ADULT_DETECTION;
            image_processing_function_switch &= ~CHILD_DETECTION;

            driving_mode_flag = temp_mode_flag;
            temp_mode_flag = NO_BREAK_INDEX_FLAG;

        }

    }
    // LOG_INFO(adtf_util::cString::Format("(EmergencyBreak) Driving Flag Input after %d",driving_mode_flag));
    return driving_mode_flag;
}


tResult SOP_AutonomousDriving::CarFollowing(void)
{


    if (obstacle_from_Ultrasonic.find_flag == tTrue && average_distance < carFollowing_detect_distance && pre_veh_speed <= lane_follow_speed && pre_veh_speed >= carFollowing_preVeh_minSpeed)
    {
        LOG_INFO(adtf_util::cString::Format("***********Car following start***********"));
        if (lane_follow_speed >= 0.4)
            lane_follow_speed = lane_follow_speed - (lane_follow_speed - pre_veh_speed)/5;
        else
            lane_follow_speed = 0.4;
    }
    else
    {
        pre_veh_speed = 2;
    }

    RETURN_NOERROR;
}

int SOP_AutonomousDriving::CrossingDecision(int driving_mode_flag)
{
    int index = 0;
    int T_crossing_direction = -1;


    //open crossing flag distance in Cm
    if((road_marker_ID == UNMARKED_INTERSECTION || road_marker_ID == STOP_GIVE_WAY || road_marker_ID == HAVEWAY || road_marker_ID == GIVE_WAY ) && marker_distance < crossing_marker_distance)
    {
        if(pedestrian_flag == tTrue)
        {
            pedestrian_flag = tFalse;
            image_processing_function_switch &= ~ADULT_DETECTION;
            image_processing_function_switch &= ~CHILD_DETECTION;
        }

        temp_marker_ID = road_marker_ID;
        temp_crossing_stop_distance = marker_distance;
        last_distance_overall_for_crossing = distance_overall;

        if((road_marker_ID == HAVEWAY && ManeuverList.action[ManeuverList.id][0] != TURN_LEFT)
                || (road_marker_ID == UNMARKED_INTERSECTION && ManeuverList.action[ManeuverList.id][0] == TURN_RIGHT)  )
            stop_decision_flag = NOSTOP_DECISION;
        else
        {
            stop_decision_flag = STOP_DECISION;
            crossing_stop_wait_counter = CrossingWaitTimeDecision(temp_marker_ID);
        }
        crossing_flag = CROSSING_FLAG_TRAFFIC_SIGNS;

        if(marker_distance < 30)
        {
            image_processing_function_switch &= ~LANE_DETECTION;
            image_processing_function_switch &= ~STOP_LINE_DETECTION;
            image_processing_function_switch &= ~ADULT_DETECTION;
            image_processing_function_switch &= ~CHILD_DETECTION;
        }

        if (ManeuverList.action[ManeuverList.id][0] == TURN_LEFT)
        {
            if(light_flag[LEFT] == tFalse)
                ToggleLights(LEFT, tTrue);
        }
        else if (ManeuverList.action[ManeuverList.id][0] == TURN_RIGHT)
        {
            if(light_flag[RIGHT] == tFalse)
                ToggleLights(RIGHT, tTrue);
        }


        LOG_INFO(adtf_util::cString::Format("Crossing traffic sign recognized %d",temp_marker_ID));
    }


    else if(crossing_flag == CROSSING_FLAG_TRAFFIC_SIGNS)
    {
        //        lane_follow_speed = 0.5;
        temp_crossing_stop_distance -= ((distance_overall - last_distance_overall_for_crossing) * 100);
        last_distance_overall_for_crossing = distance_overall;
        //        LOG_INFO(adtf_util::cString::Format("Temp_Stop %g",temp_crossing_stop_distance));
        //        LOG_INFO(adtf_util::cString::Format("Temp_marker %d",temp_marker_ID));

        if (ManeuverList.action[ManeuverList.id][0] == TURN_LEFT)
        {
            nostop_crossing = nostop_crossing_left;
            stop_crossing = stop_crossing_left;
        }
        else
        {
            nostop_crossing = nostop_crossing_right;
            stop_crossing = stop_crossing_right;
        }


        if(stop_decision_flag == STOP_DECISION && temp_crossing_stop_distance <= stop_crossing && driving_mode_flag != CAR_STOP)//-3//15//29 //stop_decision_flag = stop_decision
        {
            driving_mode_flag = CAR_STOP;
            LOG_INFO(adtf_util::cString::Format("Driving Flag Input after %d",driving_mode_flag));
            temp_crossing_stop_distance = 0;
        }

        else if(stop_decision_flag == NOSTOP_DECISION && temp_crossing_stop_distance <= nostop_crossing)  //-27//left is 4, right is -14
        {
            switch (ManeuverList.action[ManeuverList.id][0])
            {
            case TURN_LEFT:

                driving_mode_flag = ManeuverList.action[ManeuverList.id][0];
                crossing_flag = CROSSING_FLAG_OFF;
                stop_decision_flag = STOP_DECISION;

                //                    LOG_INFO(adtf_util::cString::Format("--------------------"));
                //                    LOG_INFO(adtf_util::cString::Format("Start turn left"));
                break;
            case TURN_RIGHT:
                driving_mode_flag = ManeuverList.action[ManeuverList.id][0];
                crossing_flag = CROSSING_FLAG_OFF;
                stop_decision_flag = STOP_DECISION;

                                    LOG_INFO(adtf_util::cString::Format("--------------------"));
                                    LOG_INFO(adtf_util::cString::Format("Start turn right"));
                break;
            case STRAIGHT:
                driving_mode_flag = ManeuverList.action[ManeuverList.id][0];
                crossing_flag = CROSSING_FLAG_OFF;
                stop_decision_flag = STOP_DECISION;

                //                    LOG_INFO(adtf_util::cString::Format("--------------------"));
                //                    LOG_INFO(adtf_util::cString::Format("Start straight"));
                break;
            default:
                break;

            }
            temp_crossing_stop_distance = 0;
        }



    }


    if(driving_mode_flag == CAR_STOP && crossing_flag != CROSSING_FLAG_OFF)
    {

        //        LOG_INFO(adtf_util::cString::Format("Left %g  center left %g  center %g center right %gright %g", ult_world_coord[F_LEFT][X] , ult_world_coord[F_CENTER_LEFT][X] , ult_world_coord[F_CENTER][X] , ult_world_coord[F_CENTER_RIGHT][X] ,ult_world_coord[F_RIGHT][X] ));

        float temp_HeadingAngle = car_est_position.HeadingAngle;
        if (car_est_position.HeadingAngle >= -PI/4 && car_est_position.HeadingAngle <= PI/4 )
            temp_HeadingAngle = 0;
        else if (car_est_position.HeadingAngle >= PI/4 && car_est_position.HeadingAngle <= 3*PI/4 )
            temp_HeadingAngle = 90;
        else if (car_est_position.HeadingAngle >= -3*PI/4  && car_est_position.HeadingAngle <= -PI/4 )
            temp_HeadingAngle = -90;
        else
            temp_HeadingAngle = 180;

        tFloat32 Tx = static_cast<tFloat32>(car_est_position.X_Position+cos(temp_HeadingAngle*DEGREES_TO_RADIAN)*0.7);
        tFloat32 Ty = static_cast<tFloat32>(car_est_position.Y_Position+sin(temp_HeadingAngle*DEGREES_TO_RADIAN)*0.7);
        LOG_INFO(cString::Format("Position x %g Position y %g", Tx, Ty));

        for(index = 0; index < T_crossing_nummer; index++)
        {
            if (Tx < T_section_boundary[index].right && Tx > T_section_boundary[index].left &&
                Ty < T_section_boundary[index].top && Ty > T_section_boundary[index].bom)
            {
                if(temp_HeadingAngle == T_section_boundary[index].HeadingAngle)
                    T_crossing_direction = 0;
                else if(fabs(temp_HeadingAngle - T_section_boundary[index].HeadingAngle) == 180)
                    T_crossing_direction = 1;
                else if(fabs(temp_HeadingAngle - T_section_boundary[index].HeadingAngle) == 90||fabs(temp_HeadingAngle - T_section_boundary[index].HeadingAngle) == 270)
                    T_crossing_direction = 2;

                if (T_crossing_direction!=-1)
                {
                    LOG_INFO(cString::Format("T crossing section id %d Direction %d heading %g", index, T_crossing_section[index].HeadingAngle));
                    break;
                }

            }
        }


        // from left to right: 90 140 100 Middle 80
        if ((T_crossing_direction == -1 && (temp_marker_ID == GIVE_WAY || temp_marker_ID == STOP_GIVE_WAY) && (ultrasonic_value[0] > crossing_left_distance || abs(temp_ultrasonic_value_0 - ultrasonic_value[0]) > 10) && ultrasonic_value[2] > crossing_front_distance &&  ultrasonic_value[6] > crossing_right_distance && ultrasonic_value[4] > crossing_middle_distance) ||
                (T_crossing_direction == -1 && temp_marker_ID == UNMARKED_INTERSECTION && ManeuverList.action[ManeuverList.id][0] == TURN_LEFT && ultrasonic_value[2] > crossing_front_distance &&  ultrasonic_value[6] > crossing_right_distance && ultrasonic_value[4] > crossing_middle_distance ) ||
                (T_crossing_direction == -1 && temp_marker_ID == UNMARKED_INTERSECTION && ManeuverList.action[ManeuverList.id][0] == STRAIGHT &&  ultrasonic_value[6] > crossing_right_distance && ultrasonic_value[4] > crossing_middle_distance ) ||
                (T_crossing_direction == -1 && temp_marker_ID == HAVEWAY && ManeuverList.action[ManeuverList.id][0] == TURN_LEFT && ultrasonic_value[2] > crossing_front_distance  && ultrasonic_value[4] > crossing_middle_distance ) ||

                (T_crossing_direction == 0 && (temp_marker_ID == GIVE_WAY || temp_marker_ID == STOP_GIVE_WAY) && (ultrasonic_value[0] > crossing_left_distance || abs(temp_ultrasonic_value_0 - ultrasonic_value[0]) > 10) && ultrasonic_value[2] > crossing_front_distance  && ultrasonic_value[4] > crossing_middle_distance) ||
                (T_crossing_direction == 0 && temp_marker_ID == UNMARKED_INTERSECTION && ManeuverList.action[ManeuverList.id][0] == TURN_LEFT && ultrasonic_value[2] > crossing_front_distance  && ultrasonic_value[4] > crossing_middle_distance) ||
                (T_crossing_direction == 0 && temp_marker_ID == HAVEWAY && ManeuverList.action[ManeuverList.id][0] == TURN_LEFT && ultrasonic_value[2] > crossing_front_distance  && ultrasonic_value[4] > crossing_middle_distance) ||

                (T_crossing_direction == 1 && (temp_marker_ID == GIVE_WAY || temp_marker_ID == STOP_GIVE_WAY) /*&& ultrasonic_value[2] > 140 */&&  ultrasonic_value[6] > crossing_right_distance && ultrasonic_value[4] > crossing_middle_distance) ||
                (T_crossing_direction == 1 && temp_marker_ID == UNMARKED_INTERSECTION && ManeuverList.action[ManeuverList.id][0] == STRAIGHT &&  ultrasonic_value[6] > crossing_right_distance && ultrasonic_value[4] > crossing_middle_distance ) ||

                (T_crossing_direction == 2 && (temp_marker_ID == GIVE_WAY || temp_marker_ID == STOP_GIVE_WAY) && (ultrasonic_value[0] > crossing_left_distance || abs(temp_ultrasonic_value_0 - ultrasonic_value[0]) > 10) &&  ultrasonic_value[6] > crossing_right_distance && ultrasonic_value[4] > crossing_middle_distance) ||
                (T_crossing_direction == 2 && temp_marker_ID == UNMARKED_INTERSECTION && ManeuverList.action[ManeuverList.id][0] == TURN_LEFT &&  ultrasonic_value[6] > crossing_right_distance && ultrasonic_value[4] > crossing_middle_distance ) ||
                (T_crossing_direction == 2 && temp_marker_ID == HAVEWAY && ManeuverList.action[ManeuverList.id][0] == TURN_LEFT  && ultrasonic_value[4] > crossing_middle_distance ) ||

                (crossing_flag == CROSSING_FLAG_STOP_LINE && T_crossing_direction == -1 && ManeuverList.action[ManeuverList.id][0] == TURN_LEFT && ultrasonic_value[2] > crossing_front_distance &&  ultrasonic_value[6] > crossing_right_distance && ultrasonic_value[4] > crossing_middle_distance ) ||
                (crossing_flag == CROSSING_FLAG_STOP_LINE && T_crossing_direction == -1 && ManeuverList.action[ManeuverList.id][0] == STRAIGHT &&  ultrasonic_value[6] > crossing_right_distance && ultrasonic_value[4] > crossing_middle_distance ) ||
                (crossing_flag == CROSSING_FLAG_STOP_LINE && T_crossing_direction == 2 && (ultrasonic_value[0] > crossing_left_distance || abs(temp_ultrasonic_value_0 - ultrasonic_value[0]) > 10) &&  ultrasonic_value[6] > crossing_right_distance && ultrasonic_value[4] > crossing_middle_distance)
                )
        {
            if (crossing_stop_wait_counter <= 0)
            {
                LOG_INFO(adtf_util::cString::Format("**** Section not Occupied ****"));

                driving_mode_flag = ManeuverList.action[ManeuverList.id][0];
                crossing_flag = CROSSING_FLAG_OFF;
                crossing_stop_wait_counter = 0;
                if (crossing_stopLine_mode == tTrue)
                    stopLine_check_flag = 0;

                if(light_flag[BRAKE] == tTrue)
                {
                    ToggleLights(BRAKE, tFalse);
                }
            }
            if(crossing_stop_wait_counter > 0)
                crossing_stop_wait_counter--;

            //            LOG_INFO(adtf_util::cString::Format("**** crossing_stop_wait_counter %d", crossing_stop_wait_counter));
        }
        else
        {
            crossing_stop_wait_counter = CrossingWaitTimeDecision(temp_marker_ID);
        }

    }

    temp_ultrasonic_value_0 = ultrasonic_value[0];

    return driving_mode_flag;
}

int SOP_AutonomousDriving::CrossingWaitTimeDecision(int marker_ID)
{
    int wait_time_counter = 5;  //60 * 50 mS = 3 S

    if(marker_ID == HAVEWAY && ManeuverList.action[ManeuverList.id][0] != TURN_LEFT)
        wait_time_counter = 0;
    else if(marker_ID == HAVEWAY && ManeuverList.action[ManeuverList.id][0] == TURN_LEFT)
        wait_time_counter = 5;
    else if(marker_ID == STOP_GIVE_WAY)
        wait_time_counter = 30;
    else
    {
        crossing_stop_wait_counter = 10;//
    }

    return wait_time_counter;
}


/*
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 * */
int SOP_AutonomousDriving::DrivingModeDecision_TestModel(void)
{
    //__synchronized_obj(m_oCritSectionInputData);

    int driving_mode_flag = CAR_STOP;

    if(position_input_flag == tFalse)
        return CAR_STOP;


    if(marker_update_counter > 99)
    {
        road_marker_ID = 99;
        marker_distance = 0;
        marker_lateral = 0;
    }
    //    LOG_INFO(adtf_util::cString::Format("MarkerID: %d  Distance: %d   Lateral: %d ",road_marker_ID, marker_distance, marker_lateral));





    switch (input_state_flag)
    {
    case CAR_STOP:
        driving_mode_flag = CAR_STOP;

        break;

        /* Lane follow stop in two conditions:
         * Distance to a stop line smaller than 70 || Distance to a road sign smaller than 50
         * If true, than set crossing_flag = 1, than estimate the distance with car speed
         * If rest distance smaller than 50, then set CAR_STOP
        */
    case LANE_FOLLOW:
        driving_mode_flag = LANE_FOLLOW;


        //************* traffic sign and stop line: stop **************//
        if(crossing_flag != CROSSING_FLAG_OFF)
        {
            //Stop Distance in Cm
            //            temp_crossing_stop_distance -= ((car_speed * 0.05) * 100);
            temp_crossing_stop_distance -= ((distance_overall - last_distance_overall_for_crossing) * 100);
            last_distance_overall_for_crossing = distance_overall;
            //   LOG_INFO(adtf_util::cString::Format("Temp_Stop %g",temp_crossing_stop_distance));
            if(temp_crossing_stop_distance <= 20)//20//17
            {
                driving_mode_flag = CAR_STOP;
                input_state_flag = CAR_STOP;
                crossing_flag = CROSSING_FLAG_OFF;
                LOG_INFO(adtf_util::cString::Format(" Crossing stop flag %d, Distance %g", crossing_flag, temp_crossing_stop_distance));


                temp_crossing_stop_distance = 0;
            }
        }


        //open crossing stop flag distance in Cm
        //        else if(stop_line_distance != 0 && stop_line_distance < 75)
        //        {

        //            temp_crossing_stop_distance = stop_line_distance;
        //            crossing_flag = CROSSING_FLAG_STOP_LINE;
        //        }
        else if(road_marker_ID == STOP_GIVE_WAY && marker_distance < 70)
        {
            temp_crossing_stop_distance = marker_distance;
            crossing_flag = CROSSING_FLAG_TRAFFIC_SIGNS;
        }

        lane_follow_speed = 0.4;


        //        driving_mode_flag = CrossingDecision(driving_mode_flag);
        driving_mode_flag = ParkingProcess(driving_mode_flag);
        //***************************//

      

        break;

        /* Turn_left begins to check conditions before end, i.e. Round > 4*N -5
         * The stop conditions are the same as lane follow
         * If not stopped, check if driving recognized from image processing.
         *      If true, switch to lane follow
         * If round >= 4*N, then stop
        */
    case TURN_LEFT:
        driving_mode_flag = TURN_LEFT;


        if(turn_around_reference_counter >= (3*N))
        {
            driving_mode_flag = CAR_STOP;
            input_state_flag = CAR_STOP;
        }
    

        break;

    case TURN_RIGHT:
        driving_mode_flag = TURN_RIGHT;

        if(turn_around_reference_counter >= (2*N))
        {
            driving_mode_flag = CAR_STOP;
            input_state_flag = CAR_STOP;

            LOG_INFO(adtf_util::cString::Format("stop TURN_RIGHT"));
        }
      
        break;

    case STRAIGHT:
        driving_mode_flag = STRAIGHT;

        break;

    case PARKING:
        driving_mode_flag = PARKING;

        if(turn_around_reference_counter >= (9*N-10))
        {
            driving_mode_flag = CAR_STOP;
            input_state_flag  = CAR_STOP;
        }



        break;

    case PULL_OUT_LEFT:

        driving_mode_flag = PULL_OUT_LEFT;
        if(turn_around_reference_counter >= (3*N))
        {
            driving_mode_flag = CAR_STOP;
            input_state_flag = CAR_STOP;

        }

        break;

    case PULL_OUT_RIGHT:
        driving_mode_flag = PULL_OUT_RIGHT;



        if(turn_around_reference_counter >= (3*N))
        {
            driving_mode_flag = CAR_STOP;
            input_state_flag = CAR_STOP;
            if(light_flag[LEFT] == tTrue)
                ToggleLights(LEFT, tFalse);

            pull_out_light_counter = 0;
        }


        break;


    default:
        driving_mode_flag = CAR_STOP;
        break;
    }
    // LOG_INFO(adtf_util::cString::Format("Driving Flag Input %d",driving_mode_flag));



    driving_mode_flag = UltrasensorEmergencyBreak(driving_mode_flag);

    return driving_mode_flag;
}
