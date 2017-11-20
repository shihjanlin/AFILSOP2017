#include "SOP_AutonomousDriving.h"
#include "IpIpoptApplication.hpp"

#include "Nmpc/audi_q2_nlp.h"
#include "Nmpc/rungekutta.h"







#define MAX_POSITIVE_STEERING_ANGLE           28.2  //Degree
#define MAX_NEGATIVE_STEERING_ANGLE           25.8 //25.8
#define POSITIVE_STEERING_ANGLE_TO_PERCENT    (100.0 / MAX_POSITIVE_STEERING_ANGLE)
#define NEGATIVE_STEERING_ANGLE_TO_PERCENT    (100.0 / MAX_NEGATIVE_STEERING_ANGLE)


double Qx = 20;     // Weighting matrix for X-coordinate
double QxN = 2;  // Weighting matrix for X-coordinate at final point
double Qy = 5; 	 // Weighting matrix for Y-coordinate
double QyN = 5;//5; 	 // Weighting matrix for Y-coordinate at final point
double Qpsi = 2; // Weighting factor for heading
double Rv = 1; 	 // Weighting matrix for velocity
double Rd = 1;//3; 	 // Weighting matrix for steering angle
double CHGsa = 1;
double CHGss = 1;
double Qrd = 1.5;
double vcc = 0.5;
double shift_lane = 0.0;
int shift_lane_counter = 0;


double TTC;

float followspeed_counter = 1;
float v1=0.0;
float v2=0.0;
bool speedchange_finished = tFalse;
float v1_ex=0.0;

double XX[N*NXU + NX];
double LAMBDA[(N+1)*NX + N +1];
double ZL[N*NXU + NX];
double ZU[N*NXU + NX];
MatrixXd dxdu(N*NX,NXU);
double Xini[NX];
double OUTPUT[(N+1)*NX];
double xlower[NXU];
double xupper[NXU];
MatrixXd M(3,4); // Collocation matrix, derivatives of the Lagrange polynomials at collocation points
volatile double ipoptDt;
int mpcIdx = 0;

RungeKutta *rk4;
float xsol[N];
float ysol[N];
float xsol_temp;
float ysol_temp;

float last_heading = 0;

COORDINATE_STRUCT *soll;
MPC_PARAMETER *MPC_parameter;

SmartPtr<TNLP> mynlp;
SmartPtr<IpoptApplication> app;
ApplicationReturnStatus status;



tFloat32 last_mpc_steering = 0;


float dist = 0;
float rdist =0;
float zdist = 0;
int time_counter = 0;

/* Data for Kalman filter*/
Matrix3d P;
double PP = 0.5; // The a posteriori error covariance matrix
Vector3d sensors;
Vector3d estimates;
vector <double (*)(double*, double*, double)> funcs;
double dotX1(double *sVars, double *uu, double h);
double dotX2(double *sVars, double *uu, double h);
double dotX3(double *sVars, double *uu, double h);
Matrix3d F;
Matrix3d H;
Matrix3d Q;
Matrix3d R;
Matrix3d I;

#ifdef AUTO_A
double FF = 1;
double HH = 1;
double QQ = 10;
double RR = 10;
double II = 1.0;
#else
double FF = 1;
double HH = 1;
double QQ = 10; // The covariance matrix of the process noise
double RR = 15;  // The covariance matrix of the observation noise
double II = 1.0;
#endif
/* ********************* */

tResult SOP_AutonomousDriving::AutoControl(int input_car_state_flag)
{
    
    switch (input_car_state_flag)
    {
    case CAR_STOP:
        output_steering = 0;
        last_speed = 0;
        WriteSignalValue(&steering_output, output_steering, 0);
        WriteSignalValue(&speed_output, 0, 0);

        //turn_around_reference_counter = 0;
        if(light_flag[BRAKE] == tFalse)
            ToggleLights(BRAKE, tTrue);
        break;
    case EMERGENCY_BREAK:
        WriteSignalValue(&speed_output, 0, 0);

        //turn_around_reference_counter = 0;
        break;


    case LANE_FOLLOW:

        //        image_processing_function_switch |= LANE_DETECTION;

        if(crossing_flag == 1)
        {
            WriteSignalValue(&steering_output, last_steering, 0);
            WriteSignalValue(&speed_output, last_speed, 0);
        }
        else
        {

            if(reference_value[0] == LTRACE || reference_value[0] == SL_TRACE)
            {
                memcpy(soll->X, ref_lane_world_coord.X, sizeof(ref_lane_world_coord.X));
                memcpy(soll->Y, ref_lane_world_coord.Y, sizeof(ref_lane_world_coord.Y));
                //LOG_INFO(adtf_util::cString::Format("Curve cor = %g, SL_TRACE = %g ", ref_lane_world_coord.X, ref_lane_world_coord.Y));
                CalculateMPC(input_car_state_flag, 1.0);
            }
            else
            {
                WriteSignalValue(&steering_output, last_steering, 0);
                WriteSignalValue(&speed_output, last_speed, 0);
            }
            turn_around_reference_counter = 0;
            last_distance_overall = distance_overall;

        }
        break;

    case AVOIDANCE:


    {
        if(turn_around_reference_counter == 0)
        {

            car_est_position.X_Position =  estimates(0); // X Messwerte
            car_est_position.Y_Position =  estimates(1); // Y Messwerte
            car_est_position.HeadingAngle =  estimates(2); // Psi Messwerte

            CalculateTurnAroundReferencePoint(AVOIDANCE, turn_around_reference_counter);
            memcpy(soll->X, ref_lane_world_coord.X, sizeof(ref_lane_world_coord.X));
            memcpy(soll->Y, ref_lane_world_coord.Y, sizeof(ref_lane_world_coord.Y));

            rdist = distance_overall-last_distance_overall;
            dist = GetDistanceBetweenCoordinates(ref_lane_world_coord.X[1], ref_lane_world_coord.Y[1] , ref_lane_world_coord.X[0], ref_lane_world_coord.Y[0] );

#ifdef OUTPUT_TURN_AROUND_COUTER_DEBUG
            LOG_INFO(adtf_util::cString::Format("AVOIDANCE Ziel%d distance %f currentDist %f lastDist %f reldist %f",turn_around_reference_counter,dist,distance_overall,last_distance_overall,rdist));
#endif
            last_distance_overall = distance_overall;
            turn_around_reference_counter++;

            image_processing_function_switch &= ~LANE_DETECTION;
            image_processing_function_switch &= ~STOP_LINE_DETECTION;
            image_processing_function_switch &= ~ADULT_DETECTION;
            image_processing_function_switch &= ~CHILD_DETECTION;
        }
        else
        {

            if (fabs(distance_overall-last_distance_overall) > dist-0.01)
            {

                CalculateTurnAroundReferencePoint(AVOIDANCE, turn_around_reference_counter);
                memcpy(soll->X, ref_lane_world_coord.X, sizeof(ref_lane_world_coord.X));
                memcpy(soll->Y, ref_lane_world_coord.Y, sizeof(ref_lane_world_coord.Y));

#ifdef OUTPUT_REFERENCE_POINT_DEBUG
                LOG_INFO(adtf_util::cString::Format("AVOIDANCE Ziel %d X Y: %g   %g",turn_around_reference_counter, soll->X[0], soll->Y[0]));
#endif
                rdist = distance_overall-last_distance_overall;
                dist = GetDistanceBetweenCoordinates(ref_lane_world_coord.X[1], ref_lane_world_coord.Y[1] , ref_lane_world_coord.X[0], ref_lane_world_coord.Y[0] );

#ifdef OUTPUT_TURN_AROUND_COUTER_DEBUG
                LOG_INFO(adtf_util::cString::Format("AVOIDANCE Ziel%d distance %f currentDist %f lastDist %f reldist %f",turn_around_reference_counter,dist,distance_overall,last_distance_overall,rdist));
                LOG_INFO(adtf_util::cString::Format("current %d ",turn_around_reference_counter));
#endif

                last_distance_overall = distance_overall;

                if(turn_around_reference_counter < 2*N)
                    turn_around_reference_counter++;
            }
        }


        if(turn_around_reference_counter < 2*N)
            CalculateMPC(input_car_state_flag, 1.0); //0.8
        else
        {
            //            driving_mode_flag = CAR_STOP;
            WriteSignalValue(&speed_output, 0, 0);
            if(light_flag[LEFT] == tTrue)
                ToggleLights(LEFT, tFalse);
            if(light_flag[RIGHT] == tTrue)
                ToggleLights(RIGHT, tFalse);

            image_processing_function_switch |= LANE_DETECTION;
            image_processing_function_switch |= STOP_LINE_DETECTION;


            LOG_INFO(adtf_util::cString::Format("Avoidance stop"));
        }


        if(avoidance.comeback_flag == 0 || avoidance.comeback_flag == 1)
        {
            if(turn_around_reference_counter >= ((2*N) - 10))
            {
                image_processing_function_switch |= LANE_DETECTION;
                image_processing_function_switch |= STOP_LINE_DETECTION;
            }
        }
        else if(avoidance.comeback_flag == 2)
        {
            if(turn_around_reference_counter >= ((2*N) - 10))//12
            {
                image_processing_function_switch |= LANE_DETECTION;
                image_processing_function_switch |= STOP_LINE_DETECTION;
            }
        }


        if(/*turn_around_reference_counter >= (2*N)-7*/((temp_HeadingAngle == 0 || temp_HeadingAngle == PI) && fabs(car_est_position.Y_Position - avoidance_ref_coord.Y[2*N])<0.04) ||
               ((temp_HeadingAngle == PI/2 || temp_HeadingAngle == -PI/2) && fabs(car_est_position.X_Position - avoidance_ref_coord.X[2*N])<0.04) ) //7//10//111
        {
            if(reference_value[0] == LTRACE || reference_value[0] == SL_TRACE)
            {
                if(avoidance.comeback_flag == 0)
                {
                    avoidance.comeback_flag = 1;
                    avoidance.comeback_wait_counter = avoidance_comeBack_counter;
                }
                else if(avoidance.comeback_flag == 2 )
                {
                    avoidance.flag = tFalse;
                    avoidance.comeback_flag = 0;

                    LOG_INFO(adtf_util::cString::Format("**** Avoidance finished ****"));
                }
                current_car_state_flag = LANE_FOLLOW;


                if(light_flag[LEFT] == tTrue)
                    ToggleLights(LEFT, tFalse);
                if(light_flag[RIGHT] == tTrue)
                    ToggleLights(RIGHT, tFalse);

            }
        }
    }

        break;

    case TURN_LEFT:
        // Kalman filter initialization

        if(turn_around_reference_counter == 0)
        {

            car_est_position.X_Position =  estimates(0); // X Messwerte
            car_est_position.Y_Position =  estimates(1); // Y Messwerte
            car_est_position.HeadingAngle =  estimates(2); // Psi Messwerte

            CalculateTurnAroundReferencePoint(TURN_LEFT, turn_around_reference_counter);
            memcpy(soll->X, ref_lane_world_coord.X, sizeof(ref_lane_world_coord.X));
            memcpy(soll->Y, ref_lane_world_coord.Y, sizeof(ref_lane_world_coord.Y));

            rdist = distance_overall-last_distance_overall;
            dist = GetDistanceBetweenCoordinates(ref_lane_world_coord.X[1], ref_lane_world_coord.Y[1] , ref_lane_world_coord.X[0], ref_lane_world_coord.Y[0] );

#ifdef OUTPUT_TURN_AROUND_COUTER_DEBUG
            LOG_INFO(adtf_util::cString::Format("TURN_LEFT Ziel%d distance %f currentDist %f lastDist %f reldist %f",turn_around_reference_counter,dist,distance_overall,last_distance_overall,rdist));
#endif
            last_distance_overall = distance_overall;
            turn_around_reference_counter++;

            image_processing_function_switch &= ~LANE_DETECTION;
            image_processing_function_switch &= ~STOP_LINE_DETECTION;
            image_processing_function_switch &= ~ADULT_DETECTION;
            image_processing_function_switch &= ~CHILD_DETECTION;
        }
        else
        {

            if (fabs(distance_overall-last_distance_overall) > dist-0.01)
            {

                CalculateTurnAroundReferencePoint(TURN_LEFT, turn_around_reference_counter);
                memcpy(soll->X, ref_lane_world_coord.X, sizeof(ref_lane_world_coord.X));
                memcpy(soll->Y, ref_lane_world_coord.Y, sizeof(ref_lane_world_coord.Y));
                //  LOG_INFO(adtf_util::cString::Format("Ziel %d X Y: %g   %g",turn_around_reference_counter, soll->X[0], soll->Y[0]));
                rdist = distance_overall-last_distance_overall;
                dist = GetDistanceBetweenCoordinates(ref_lane_world_coord.X[1], ref_lane_world_coord.Y[1] , ref_lane_world_coord.X[0], ref_lane_world_coord.Y[0] );

#ifdef OUTPUT_TURN_AROUND_COUTER_DEBUG
                LOG_INFO(adtf_util::cString::Format("TURN_LEFT Ziel%d distance %f currentDist %f lastDist %f reldist %f",turn_around_reference_counter,dist,distance_overall,last_distance_overall,rdist));
                LOG_INFO(adtf_util::cString::Format("current %d ",turn_around_reference_counter));
#endif
                last_distance_overall = distance_overall;

                if(turn_around_reference_counter < 3*N)
                    turn_around_reference_counter++;
            }
        }


        if(turn_around_reference_counter < 3*N)
            CalculateMPC(input_car_state_flag, 1.0); //0.8
        else
        {
            //            driving_mode_flag = CAR_STOP;
            WriteSignalValue(&speed_output, 0, 0);
            if(light_flag[LEFT] == tTrue)
                ToggleLights(LEFT, tFalse);

            LOG_INFO(adtf_util::cString::Format("turn left stop"));
        }

        if(turn_around_reference_counter >= ((3*N) - 18))
        {
            image_processing_function_switch |= LANE_DETECTION;
            image_processing_function_switch |= STOP_LINE_DETECTION;
        }


        if(turn_around_reference_counter >= (3*N)-7) //7//10//111
        {
            if(reference_value[0] == LTRACE || reference_value[0] == SL_TRACE)
            {
                current_car_state_flag = LANE_FOLLOW;

                ManeuverList.id++;
                OnSendState(stateCar_RUNNING, ManeuverList.id);

                if(light_flag[LEFT] == tTrue)
                    ToggleLights(LEFT, tFalse);

                LOG_INFO(adtf_util::cString::Format("**** TURN_LEFT finished ****"));
            }
        }

        break;


    case TURN_RIGHT:

        if(turn_around_reference_counter == 0)
        {

            car_est_position.X_Position =  estimates(0); // X Messwerte
            car_est_position.Y_Position =  estimates(1); // Y Messwerte
            car_est_position.HeadingAngle =  estimates(2); // Psi Messwerte

            CalculateTurnAroundReferencePoint(TURN_RIGHT, turn_around_reference_counter);
            memcpy(soll->X, ref_lane_world_coord.X, sizeof(ref_lane_world_coord.X));
            memcpy(soll->Y, ref_lane_world_coord.Y, sizeof(ref_lane_world_coord.Y));


            rdist = distance_overall-last_distance_overall;
            dist = GetDistanceBetweenCoordinates(ref_lane_world_coord.X[1], ref_lane_world_coord.Y[1] , ref_lane_world_coord.X[0], ref_lane_world_coord.Y[0] );

#ifdef OUTPUT_TURN_AROUND_COUTER_DEBUG
            LOG_INFO(adtf_util::cString::Format("TURN_RIGHT Ziel%d distance %f currentDist %f lastDist %f reldist %f",turn_around_reference_counter,dist,distance_overall,last_distance_overall,rdist));
#endif
            last_distance_overall = distance_overall;
            //                LOG_INFO(adtf_util::cString::Format("Ziel %d X Y: %g   %g",turn_around_reference_counter, soll->X[0], soll->Y[0]));
            turn_around_reference_counter++;

            image_processing_function_switch &= ~LANE_DETECTION;
            image_processing_function_switch &= ~STOP_LINE_DETECTION;
            image_processing_function_switch &= ~ADULT_DETECTION;
            image_processing_function_switch &= ~CHILD_DETECTION;
        }
        else
        {
            if (distance_overall-last_distance_overall > dist-0.005)
            {

                CalculateTurnAroundReferencePoint(TURN_RIGHT, turn_around_reference_counter);
                memcpy(soll->X, ref_lane_world_coord.X, sizeof(ref_lane_world_coord.X));
                memcpy(soll->Y, ref_lane_world_coord.Y, sizeof(ref_lane_world_coord.Y));
                // LOG_INFO(adtf_util::cString::Format("Ziel %d X Y: %g   %g",turn_around_reference_counter, soll->X[0], soll->Y[0]));
                //                    LOG_INFO(adtf_util::cString::Format("Ziel %d X Y: %g   %g",turn_around_reference_counter, soll->X[0], soll->Y[0]));
                rdist = distance_overall-last_distance_overall;
                dist = GetDistanceBetweenCoordinates(ref_lane_world_coord.X[1], ref_lane_world_coord.Y[1] , ref_lane_world_coord.X[0], ref_lane_world_coord.Y[0] );

#ifdef OUTPUT_TURN_AROUND_COUTER_DEBUG
                LOG_INFO(adtf_util::cString::Format("TURN_RIGHT Ziel%d distance %f currentDist %f lastDist %f reldist %f",turn_around_reference_counter,dist,distance_overall,last_distance_overall,rdist));
                LOG_INFO(adtf_util::cString::Format("current %d X Y: %g   %g",turn_around_reference_counter, car_est_position.X_Position, car_est_position.Y_Position));
#endif
                last_distance_overall = distance_overall;

                if(turn_around_reference_counter < 2*N)
                    turn_around_reference_counter++;
            }
        }

        if(turn_around_reference_counter < 2*N)
            CalculateMPC(input_car_state_flag, 1.0);
        else
        {
            //            driving_mode_flag = CAR_STOP;
            WriteSignalValue(&speed_output, 0, 0);
            if(light_flag[RIGHT] == tTrue)
                ToggleLights(RIGHT, tFalse);

            LOG_INFO(adtf_util::cString::Format("turn right stop"));
        }

        if(turn_around_reference_counter >= ((2*N)-12))
        {
            image_processing_function_switch |= LANE_DETECTION;
            image_processing_function_switch |= STOP_LINE_DETECTION;
        }

        if(turn_around_reference_counter >= (2*N - 10))  //8
        {
            if(reference_value[0] == LTRACE || reference_value[0] == SL_TRACE)
            {
                current_car_state_flag = LANE_FOLLOW;

                ManeuverList.id++;
                OnSendState(stateCar_RUNNING, ManeuverList.id);

                if(light_flag[RIGHT] == tTrue)
                    ToggleLights(RIGHT, tFalse);

                LOG_INFO(adtf_util::cString::Format("**** TURN_RIGHT finished ****"));

            }
        }
        break;

    case STRAIGHT:

        if(turn_around_reference_counter == 0)
        {
            car_est_position.X_Position =  estimates(0); // X Messwerte
            car_est_position.Y_Position =  estimates(1); // Y Messwerte
            car_est_position.HeadingAngle =  estimates(2); // Psi Messwerte

            CalculateTurnAroundReferencePoint(STRAIGHT, turn_around_reference_counter);
            memcpy(soll->X, ref_lane_world_coord.X, sizeof(ref_lane_world_coord.X));
            memcpy(soll->Y, ref_lane_world_coord.Y, sizeof(ref_lane_world_coord.Y));


            rdist = distance_overall-last_distance_overall;
            dist = GetDistanceBetweenCoordinates(ref_lane_world_coord.X[1], ref_lane_world_coord.Y[1] , ref_lane_world_coord.X[0], ref_lane_world_coord.Y[0] );

#ifdef OUTPUT_TURN_AROUND_COUTER_DEBUG
            LOG_INFO(adtf_util::cString::Format("STRAIGHT Ziel%d distance %f currentDist %f lastDist %f reldist %f",turn_around_reference_counter,dist,distance_overall,last_distance_overall,rdist));
#endif
            last_distance_overall = distance_overall;
            //                LOG_INFO(adtf_util::cString::Format("Ziel %d X Y: %g   %g",turn_around_reference_counter, soll->X[0], soll->Y[0]));
            turn_around_reference_counter++;

            image_processing_function_switch &= ~LANE_DETECTION;
            image_processing_function_switch &= ~STOP_LINE_DETECTION;
            image_processing_function_switch &= ~ADULT_DETECTION;
            image_processing_function_switch &= ~CHILD_DETECTION;
        }
        else
        {
            if (distance_overall-last_distance_overall > dist-0.005)
            {

                CalculateTurnAroundReferencePoint(STRAIGHT, turn_around_reference_counter);
                memcpy(soll->X, ref_lane_world_coord.X, sizeof(ref_lane_world_coord.X));
                memcpy(soll->Y, ref_lane_world_coord.Y, sizeof(ref_lane_world_coord.Y));
                rdist = distance_overall-last_distance_overall;
                dist = GetDistanceBetweenCoordinates(ref_lane_world_coord.X[1], ref_lane_world_coord.Y[1] , ref_lane_world_coord.X[0], ref_lane_world_coord.Y[0] );

#ifdef OUTPUT_TURN_AROUND_COUTER_DEBUG
                LOG_INFO(adtf_util::cString::Format("STRAIGHT current %d X Y: %g   %g",turn_around_reference_counter, car_est_position.X_Position, car_est_position.Y_Position));
#endif
                last_distance_overall = distance_overall;

                if(turn_around_reference_counter < 4*N)
                    turn_around_reference_counter++;
            }
        }

        if(turn_around_reference_counter < 4*N)
            CalculateMPC(input_car_state_flag, 1.0);
        else
        {

            WriteSignalValue(&speed_output, 0, 0);

            LOG_INFO(adtf_util::cString::Format("STRAIGHT stop"));
        }

        if(turn_around_reference_counter >= ((4*N)- 32))
        {
            image_processing_function_switch |= LANE_DETECTION;
            image_processing_function_switch |= STOP_LINE_DETECTION;
        }

        if(turn_around_reference_counter >= (4*N-30))  //1
        {
            if(reference_value[0] == LTRACE || reference_value[0] == SL_TRACE)
            {
                current_car_state_flag = LANE_FOLLOW;

                ManeuverList.id++;
                OnSendState(stateCar_RUNNING, ManeuverList.id);

                LOG_INFO(adtf_util::cString::Format("**** STRAIGHT finished ****"));
            }
        }

        break;

    case PULL_OUT_LEFT:

        if(ManeuverList.id != 0 && ManeuverList.action[ManeuverList.id - 1][0] == PARKING)
        {
            if(pull_out_light_counter > 0 && pull_out_light_counter < 5)
            {
                if(light_flag[HAZARD] == tFalse)
                    ToggleLights(HAZARD, tTrue);
            }
            else if(pull_out_light_counter > 55)
            {
                if(light_flag[HAZARD] == tTrue)
                    ToggleLights(HAZARD, tFalse);
                if(light_flag[LEFT] == tFalse)
                    ToggleLights(LEFT, tTrue);
            }
        }
        else
        {
            if(pull_out_light_counter == 0)
            {
                pull_out_light_counter = 50;
                if(light_flag[LEFT] == tFalse)
                    ToggleLights(LEFT, tTrue);
            }
        }
        if(pull_out_light_counter > 60)
        {
            if(turn_around_reference_counter == 0)
            {

                car_est_position.X_Position =  estimates(0); // X Messwerte
                car_est_position.Y_Position =  estimates(1); // Y Messwerte
                car_est_position.HeadingAngle =  estimates(2); // Psi Messwerte

                CalculateTurnAroundReferencePoint(PULL_OUT_LEFT, turn_around_reference_counter);
                memcpy(soll->X, ref_lane_world_coord.X, sizeof(ref_lane_world_coord.X));
                memcpy(soll->Y, ref_lane_world_coord.Y, sizeof(ref_lane_world_coord.Y));

                rdist = distance_overall-last_distance_overall;
                dist = GetDistanceBetweenCoordinates(ref_lane_world_coord.X[1], ref_lane_world_coord.Y[1] , ref_lane_world_coord.X[0], ref_lane_world_coord.Y[0] );

#ifdef OUTPUT_TURN_AROUND_COUTER_DEBUG
                LOG_INFO(adtf_util::cString::Format("PULL_OUT_LEFT Ziel%d distance %f currentDist %f lastDist %f reldist %f",turn_around_reference_counter,dist,distance_overall,last_distance_overall,rdist));
#endif
                last_distance_overall = distance_overall;
                turn_around_reference_counter++;

                image_processing_function_switch &= ~LANE_DETECTION;
                image_processing_function_switch &= ~STOP_LINE_DETECTION;
                image_processing_function_switch &= ~ADULT_DETECTION;
                image_processing_function_switch &= ~CHILD_DETECTION;

            }
            else
            {

                if (distance_overall-last_distance_overall > dist-0.01)
                {

                    CalculateTurnAroundReferencePoint(PULL_OUT_LEFT, turn_around_reference_counter);
                    memcpy(soll->X, ref_lane_world_coord.X, sizeof(ref_lane_world_coord.X));
                    memcpy(soll->Y, ref_lane_world_coord.Y, sizeof(ref_lane_world_coord.Y));

                    rdist = distance_overall-last_distance_overall;
                    dist = GetDistanceBetweenCoordinates(ref_lane_world_coord.X[1], ref_lane_world_coord.Y[1] , ref_lane_world_coord.X[0], ref_lane_world_coord.Y[0] );

#ifdef OUTPUT_TURN_AROUND_COUTER_DEBUG
                    //LOG_INFO(adtf_util::cString::Format("PULL_OUT_LEFT Ziel%d distance %f currentDist %f lastDist %f reldist %f",turn_around_reference_counter,dist,distance_overall,last_distance_overall,rdist));
#endif
                    last_distance_overall = distance_overall;

                    if(turn_around_reference_counter < 3*N)
                        turn_around_reference_counter++;
                }
            }

            if(turn_around_reference_counter < 3*N)
                CalculateMPC(input_car_state_flag, 1.0);
            else
            {
                WriteSignalValue(&speed_output, 0, 0);

                if(light_flag[LEFT] == tTrue)
                    ToggleLights(LEFT, tFalse);

                LOG_INFO(adtf_util::cString::Format("**** PULL_OUT_LEFT stop ****"));
            }


            if(turn_around_reference_counter >= ((3*N)-18))
            {
                image_processing_function_switch |= LANE_DETECTION;
                image_processing_function_switch |= STOP_LINE_DETECTION;
            }


            if(turn_around_reference_counter >= (3*N - 2))
            {
                if(reference_value[0] == LTRACE || reference_value[0] == SL_TRACE)
                {
                    current_car_state_flag = LANE_FOLLOW;

                    ManeuverList.id++;
                    OnSendState(stateCar_RUNNING, ManeuverList.id);

                    if(light_flag[LEFT] == tTrue)
                        ToggleLights(LEFT, tFalse);

                    pull_out_light_counter = 0;

                    LOG_INFO(adtf_util::cString::Format("**** PULL_OUT_LEFT finished ****"));
                }
            }
        }
        else
        {
            pull_out_light_counter++;
            WriteSignalValue(&steering_output, (0), 0);
            WriteSignalValue(&speed_output, 0, 0);
        }

        break;


    case PULL_OUT_RIGHT:
        if(ManeuverList.id != 0 && ManeuverList.action[ManeuverList.id - 1][0] == PARKING)
        {
            if(pull_out_light_counter > 0 && pull_out_light_counter < 5)
            {
                if(light_flag[HAZARD] == tFalse)
                    ToggleLights(HAZARD, tTrue);
            }
            if(pull_out_light_counter > 55)
            {
                if(light_flag[HAZARD] == tTrue)
                    ToggleLights(HAZARD, tFalse);
                if(light_flag[RIGHT] == tFalse)
                    ToggleLights(RIGHT, tTrue);
            }
        }
        else
        {
            if(pull_out_light_counter == 0)
            {
                pull_out_light_counter = 50;
                if(light_flag[RIGHT] == tFalse)
                    ToggleLights(RIGHT, tTrue);
             }
        }

        if(pull_out_light_counter > 60)
        {
            if(turn_around_reference_counter == 0)
            {

                car_est_position.X_Position =  estimates(0); // X Messwerte
                car_est_position.Y_Position =  estimates(1); // Y Messwerte
                car_est_position.HeadingAngle =  estimates(2); // Psi Messwerte


                CalculateTurnAroundReferencePoint(PULL_OUT_RIGHT, turn_around_reference_counter);
                memcpy(soll->X, ref_lane_world_coord.X, sizeof(ref_lane_world_coord.X));
                memcpy(soll->Y, ref_lane_world_coord.Y, sizeof(ref_lane_world_coord.Y));


                rdist = distance_overall-last_distance_overall;
                dist = GetDistanceBetweenCoordinates(ref_lane_world_coord.X[1], ref_lane_world_coord.Y[1] , ref_lane_world_coord.X[0], ref_lane_world_coord.Y[0] );

                last_distance_overall = distance_overall;
                turn_around_reference_counter++;

                image_processing_function_switch &= ~LANE_DETECTION;
                image_processing_function_switch &= ~STOP_LINE_DETECTION;
                image_processing_function_switch &= ~ADULT_DETECTION;
                image_processing_function_switch &= ~CHILD_DETECTION;
            }
            else
            {
                if (distance_overall-last_distance_overall > dist-0.01)
                {

                    CalculateTurnAroundReferencePoint(PULL_OUT_RIGHT, turn_around_reference_counter);
                    memcpy(soll->X, ref_lane_world_coord.X, sizeof(ref_lane_world_coord.X));
                    memcpy(soll->Y, ref_lane_world_coord.Y, sizeof(ref_lane_world_coord.Y));

                    rdist = distance_overall-last_distance_overall;
                    dist = GetDistanceBetweenCoordinates(ref_lane_world_coord.X[1], ref_lane_world_coord.Y[1] , ref_lane_world_coord.X[0], ref_lane_world_coord.Y[0] );


#ifdef OUTPUT_TURN_AROUND_COUTER_DEBUG
                    //LOG_INFO(adtf_util::cString::Format("PULL_OUT_RIGHT current %d X Y: %g   %g",turn_around_reference_counter, car_est_position.X_Position, car_est_position.Y_Position));
#endif
                    last_distance_overall = distance_overall;

                    if(turn_around_reference_counter < 3*N)
                        turn_around_reference_counter++;
                }
            }

            if(turn_around_reference_counter < 3*N)
                CalculateMPC(input_car_state_flag, 1.0);
            else
            {
                WriteSignalValue(&speed_output, 0, 0);

                if(light_flag[RIGHT] == tTrue)
                    ToggleLights(RIGHT, tFalse);

                LOG_INFO(adtf_util::cString::Format("**** PULL_OUT_RIGHT stop ****"));
            }

            if(turn_around_reference_counter >= ((3*N)-18))
            {
                image_processing_function_switch |= LANE_DETECTION;
                image_processing_function_switch |= STOP_LINE_DETECTION;
            }

            if(turn_around_reference_counter >= (3*N - 5))
            {
                if(reference_value[0] == LTRACE || reference_value[0] == SL_TRACE)
                {
                    current_car_state_flag = LANE_FOLLOW;

                    ManeuverList.id++;
                    OnSendState(stateCar_RUNNING, ManeuverList.id);

                    if(light_flag[RIGHT] == tTrue)
                        ToggleLights(RIGHT, tFalse);

                    pull_out_light_counter = 0;

                    LOG_INFO(adtf_util::cString::Format("**** PULL_OUT_RIGHT finished ****"));
                }
            }
        }
        else
        {
            pull_out_light_counter++;
            WriteSignalValue(&steering_output, (0), 0);
            WriteSignalValue(&speed_output, 0, 0);
        }


        break;

    case PARKING:

        if(turn_around_reference_counter == 0)
        {

            car_est_position.X_Position =  estimates(0); // X Messwerte
            car_est_position.Y_Position =  estimates(1); // Y Messwerte
            car_est_position.HeadingAngle =  estimates(2); // Psi Messwerte

            CalculateTurnAroundReferencePoint(PARKING, turn_around_reference_counter);
            memcpy(soll->X, ref_lane_world_coord.X, sizeof(ref_lane_world_coord.X));
            memcpy(soll->Y, ref_lane_world_coord.Y, sizeof(ref_lane_world_coord.Y));
            rdist = distance_overall-last_distance_overall;
            dist = GetDistanceBetweenCoordinates(ref_lane_world_coord.X[1], ref_lane_world_coord.Y[1] , ref_lane_world_coord.X[0], ref_lane_world_coord.Y[0] );

            last_distance_overall = distance_overall;
            turn_around_reference_counter++;

            image_processing_function_switch &= ~LANE_DETECTION;
            image_processing_function_switch &= ~STOP_LINE_DETECTION;
            image_processing_function_switch &= ~ADULT_DETECTION;
            image_processing_function_switch &= ~CHILD_DETECTION;

            if(light_flag[RIGHT] == tTrue)
                ToggleLights(RIGHT, tFalse);
            if(light_flag[LEFT] == tFalse)
                ToggleLights(LEFT, tTrue);
        }
        else
        {
            if (distance_overall-last_distance_overall >= dist-0.005)
            {

                CalculateTurnAroundReferencePoint(PARKING, turn_around_reference_counter);
                memcpy(soll->X, ref_lane_world_coord.X, sizeof(ref_lane_world_coord.X));
                memcpy(soll->Y, ref_lane_world_coord.Y, sizeof(ref_lane_world_coord.Y));
                rdist = distance_overall-last_distance_overall;
                dist = GetDistanceBetweenCoordinates(ref_lane_world_coord.X[1], ref_lane_world_coord.Y[1] , ref_lane_world_coord.X[0], ref_lane_world_coord.Y[0] );

                last_distance_overall = distance_overall;

                if(turn_around_reference_counter < 6*N)
                    turn_around_reference_counter++;
            }
        }


        if(turn_around_reference_counter <= N )
        {


            if (temp_HeadingAngle == 0 || temp_HeadingAngle == PI)
            {
                zdist = fabs(car_est_position.Y_Position - parking_ref_coord.Y[N]) ;
                //                    zdist = car_est_position.Y_Position - parking_ref_coord.Y[N] ;

            }
            else
            {
                zdist = fabs(car_est_position.X_Position - parking_ref_coord.X[N]) ;

            }
            // when reach goal point, stop
            if (zdist < 0.025)
            {
                WriteSignalValue(&speed_output, 0, 0);
                LOG_INFO(adtf_util::cString::Format("**** Reach terminal point ****"));

                dist = 0;
            }
            else
                CalculateMPC(input_car_state_flag, 1.0);
        }
        else if (turn_around_reference_counter > N  && turn_around_reference_counter <= 2*N )
        {
            WriteSignalValue(&speed_output, 0, 0);
            LOG_INFO(adtf_util::cString::Format("**** Reach Rounds end ****"));
            dist = 0;

            if(light_flag[LEFT] == tTrue)
            {
                ToggleLights(LEFT, tFalse);
            }

            if(light_flag[RIGHT] == tFalse)
            {
                ToggleLights(RIGHT, tTrue);
            }

            if(light_flag[REVERSE] == tFalse)
            {
                ToggleLights(REVERSE, tTrue);
            }
        }
        else if (turn_around_reference_counter > 2*N && turn_around_reference_counter < 5*N )
        {

            if (temp_HeadingAngle == 0)
            {
                zdist = fabs(car_est_position.Y_Position - (parking_ref_coord.Y[0]-0.57)) ;
            }
            else if ( temp_HeadingAngle == PI )
            {
                zdist = fabs(car_est_position.Y_Position - (parking_ref_coord.Y[0]+0.57)) ;
            }
            else if ( temp_HeadingAngle == PI/2 )
            {
                zdist = fabs(car_est_position.X_Position - (parking_ref_coord.X[0]+0.57)) ;

            }
            else if ( temp_HeadingAngle == -PI/2 )
            {
                zdist = fabs(car_est_position.X_Position - (parking_ref_coord.X[0]-0.57)) ;

            }

            // when reach goal point, stop
            if (zdist < 0.03 || turn_around_reference_counter > 4*N+7)
            {
                WriteSignalValue(&speed_output, 0, 0);
                //finish signal
                turn_around_reference_counter =  5*N;
                dist = 0;
            }
            else
                CalculateMPC(input_car_state_flag, -1.0);
        }
        else
        {
            //            driving_mode_flag = CAR_STOP;
            WriteSignalValue(&speed_output, 0, 0);

            turn_around_reference_counter = 0;

            SendParkingSpace(&ParkingSpace_output, ManeuverList.action[ManeuverList.id][1], car_est_position.X_Position, car_est_position.Y_Position, 1);

            if((ManeuverList.id + 1) == ManeuverList.id_counter)
            {
                OnSendState(stateCar_COMPLETE, ManeuverList.id);
                ManeuverList.id++;
                current_car_state_flag = ManeuverList.action[ManeuverList.id][0];
                ToggleLights(HAZARD, tFalse);
            }
            else
            {
                ManeuverList.id++;
                current_car_state_flag = ManeuverList.action[ManeuverList.id][0];
                OnSendState(stateCar_RUNNING, ManeuverList.id);
            }


            if(light_flag[RIGHT] == tTrue)
                ToggleLights(RIGHT, tFalse);
            if(light_flag[REVERSE] == tTrue)
                ToggleLights(REVERSE, tFalse);

            LOG_INFO(adtf_util::cString::Format("**** PARKING finished ****"));
        }
        break;

    default:
        break;
    }

    //    ExtendedKF();

    if(car_speed != 0)
    {
        ExtendedKF();

        if((car_speed - last_speed )> 0.01 && light_flag[BRAKE] == tTrue)
        {
            ToggleLights(BRAKE, tFalse);
        }
        else if((last_speed - car_speed) > 0.1 && light_flag[BRAKE] == tFalse)
        {
            ToggleLights(BRAKE, tTrue);
        }
    }

    curvefitting();
    
    RETURN_NOERROR;
}


/* NMPC controller. For differenct maneuver, different weighting factor, speed should be configured for MPC
 * The positions are updated using Extended Kalman Filter (EKF)
 */
tResult SOP_AutonomousDriving::CalculateMPC(int input_car_state_flag, float direction)
{
    // __synchronized_obj(m_critSecOnPinEvent);
    timeval ts;
    long ipoptStartTime;
    
    // parameter settings
    MPC_parameter->MPC_car_state_flag = current_car_state_flag;
    //    if(current_car_state_flag == AVOIDANCE)
    //        MPC_parameter->MPC_car_state_flag = LANE_FOLLOW;
    if(MPC_parameter->MPC_car_state_flag == LANE_FOLLOW)
    {
        if (lane_follow_speed > (lane_follow_maxSpeed+lane_follow_minSpeed)/2)
            Qy = weightFact_LaneFollow_HY;//8/5; 	 // Weighting matrix for Y-coordinate
        else if (lane_follow_speed <= (lane_follow_maxSpeed+lane_follow_minSpeed)/2 && lane_follow_speed > 0.5)
            Qy = weightFact_LaneFollow_LY;
        else
            Qy = 8;
        QyN = 0; 	 // Weighting matrix for Y-coordinate at final point
        Qpsi = 2; // Weighting factor for heading

        Rv = 1; 	 // Weighting matrix for velocity
        Rd = 2;//2.5//2; 	 // Weighting matrix for steering angle

        vcc = lane_follow_speed;

        CHGsa = 0.5; //0.5
        CHGss = 3.0;
        initialize_bounds();
    }
    else if(MPC_parameter->MPC_car_state_flag == AVOIDANCE)
    {
        if (temp_HeadingAngle == 0 || temp_HeadingAngle == PI)
        {
            Qx = weightFact_Avoidance_X;     // Weighting matrix for X-coordinate
            // Weighting matrix for X-coordinate at final point
            Qy = weightFact_Avoidance_Y; 	 // Weighting matrix for Y-coordinate
            //            LOG_INFO(adtf_util::cString::Format("----------------------------------"));
            //            LOG_INFO(adtf_util::cString::Format("Heading angle %g",temp_HeadingAngle));
        }
        else
        {
            Qx = weightFact_Avoidance_Y;     // Weighting matrix for X-coordinate
            // Weighting matrix for X-coordinate at final point
            Qy = weightFact_Avoidance_X; 	 // Weighting matrix for Y-coordinate
            //            LOG_INFO(adtf_util::cString::Format("----------------------------------"));
            //            LOG_INFO(adtf_util::cString::Format("Heading angle %g",temp_HeadingAngle));
        }


        QxN = 2;  // Weighting matrix for X-coordinate at final point
        QyN = 1;//5; 	 // Weighting matrix for Y-coordinate at final point
        Rv = 1; 	 // Weighting matrix for velocity
        Rd = 1;//3; 	 // Weighting matrix for steering angle
        if(avoidance.comeback_flag == 2 )
        {
            if(lane_follow_speed > 0.8)
                vcc = 0.8;
            else
                vcc = lane_follow_speed;
        }
        else
        {
            vcc = avoidance_laneChange_speed;
            lane_follow_speed = avoidance_laneChange_speed;
        }
        CHGsa = 1.0;
        initialize_bounds();
    }
    else if(MPC_parameter->MPC_car_state_flag == TURN_LEFT)
    {
        if (temp_HeadingAngle == 0 || temp_HeadingAngle == PI)
        {
            Qx = weightFact_TurnLeft_X;     // Weighting matrix for X-coordinate
            // Weighting matrix for X-coordinate at final point
            Qy = weightFact_TurnLeft_Y; 	 // Weighting matrix for Y-coordinate
            //            LOG_INFO(adtf_util::cString::Format("----------------------------------"));
            //            LOG_INFO(adtf_util::cString::Format("Heading angle %g",temp_HeadingAngle));
        }
        else
        {
            Qx = weightFact_TurnLeft_Y;     // Weighting matrix for X-coordinate
            // Weighting matrix for X-coordinate at final point
            Qy = weightFact_TurnLeft_X; 	 // Weighting matrix for Y-coordinate
            //            LOG_INFO(adtf_util::cString::Format("----------------------------------"));
            //            LOG_INFO(adtf_util::cString::Format("Heading angle %g",temp_HeadingAngle));
        }


        QxN = 2;  // Weighting matrix for X-coordinate at final point
        QyN = 1;//5; 	 // Weighting matrix for Y-coordinate at final point
        //Qpsi = 2; // Weighting factor for heading
        Rv = 1; 	 // Weighting matrix for velocity
        Rd = 1;//3; 	 // Weighting matrix for steering angle
        vcc = no_lane_follow_speed * direction;
        CHGsa = 1.0;
        initialize_bounds();
    }
    else if(MPC_parameter->MPC_car_state_flag == TURN_RIGHT)
    {
        if (temp_HeadingAngle == 0 || temp_HeadingAngle == PI)
        {
            Qx = weightFact_TurnRight_X;     // Weighting matrix for X-coordinate
            // Weighting matrix for X-coordinate at final point
            Qy = weightFact_TurnRight_Y; 	 // Weighting matrix for Y-coordinate
            //            LOG_INFO(adtf_util::cString::Format("----------------------------------"));
            //            LOG_INFO(adtf_util::cString::Format("Heading angle %g",temp_HeadingAngle));
        }
        else
        {
            Qx = weightFact_TurnRight_Y;     // Weighting matrix for X-coordinate
            // Weighting matrix for X-coordinate at final point
            Qy = weightFact_TurnRight_X; 	 // Weighting matrix for Y-coordinate
            //            LOG_INFO(adtf_util::cString::Format("----------------------------------"));
            //            LOG_INFO(adtf_util::cString::Format("Heading angle %g",temp_HeadingAngle));
        }
        QyN = 1;//5; 	 // Weighting matrix for Y-coordinate at final point
        //Qpsi = 2; // Weighting factor for heading
        Rv = 1; 	 // Weighting matrix for velocity
        Rd = 1;//3; 	 // Weighting matrix for steering angle
        vcc = no_lane_follow_speed * direction;
        CHGsa = 1.0;
        initialize_bounds();
    }
    else if(MPC_parameter->MPC_car_state_flag == STRAIGHT)
    {
        if (temp_HeadingAngle == 0 || temp_HeadingAngle == PI)
        {
            Qx = weightFact_Straight_X;     // Weighting matrix for X-coordinate
            // Weighting matrix for X-coordinate at final point
            Qy = weightFact_Straight_Y; 	 // Weighting matrix for Y-coordinate
            //            LOG_INFO(adtf_util::cString::Format("----------------------------------"));
            //            LOG_INFO(adtf_util::cString::Format("Heading angle %g",temp_HeadingAngle));
        }
        else
        {
            Qx = weightFact_Straight_Y;     // Weighting matrix for X-coordinate
            // Weighting matrix for X-coordinate at final point
            Qy = weightFact_Straight_X; 	 // Weighting matrix for Y-coordinate
            //            LOG_INFO(adtf_util::cString::Format("----------------------------------"));
            //            LOG_INFO(adtf_util::cString::Format("Heading angle %g",temp_HeadingAngle));
        }
        QyN = 1;//5; 	 // Weighting matrix for Y-coordinate at final point
        //Qpsi = 2; // Weighting factor for heading
        Rv = 1; 	 // Weighting matrix for velocity
        Rd = 1;//3; 	 // Weighting matrix for steering angle
        vcc = no_lane_follow_speed * direction;
        CHGsa = 1.0;
        initialize_bounds();
    }
    else if(MPC_parameter->MPC_car_state_flag == PULL_OUT_LEFT)
    {


        if (temp_HeadingAngle == 0 || temp_HeadingAngle == PI)
        {
            Qx = weightFact_PullOutLeft_X;     // Weighting matrix for X-coordinate
            // Weighting matrix for X-coordinate at final point
            Qy = weightFact_PullOutLeft_Y; 	 // Weighting matrix for Y-coordinate
            //            LOG_INFO(adtf_util::cString::Format("----------------------------------"));
            //            LOG_INFO(adtf_util::cString::Format("Heading angle %g",temp_HeadingAngle));
        }
        else
        {
            Qx = weightFact_PullOutLeft_Y;     // Weighting matrix for X-coordinate
            // Weighting matrix for X-coordinate at final point
            Qy = weightFact_PullOutLeft_X; 	 // Weighting matrix for Y-coordinate
            //            LOG_INFO(adtf_util::cString::Format("----------------------------------"));
            //            LOG_INFO(adtf_util::cString::Format("Heading angle %g",temp_HeadingAngle));
        }

        //        Qx = weightFact_PullOutLeft_X;     // Weighting matrix for X-coordinate
        //        // Weighting matrix for X-coordinate at final point
        //        Qy = weightFact_PullOutLeft_Y; 	 // Weighting matrix for Y-coordinate

        QxN = 2;
        QyN = 5;//5; 	 // Weighting matrix for Y-coordinate at final point
        Qpsi = 2; // Weighting factor for heading
        Rv = 1; 	 // Weighting matrix for velocity
        Rd = 1;//3; 	 // Weighting matrix for steering angle
        vcc = no_lane_follow_speed * direction;
        CHGsa = 1.0;
        initialize_bounds();
    }
    else if(MPC_parameter->MPC_car_state_flag == PULL_OUT_RIGHT)
    {
        if (temp_HeadingAngle == 0 || temp_HeadingAngle == PI)
        {
            Qx = weightFact_PullOutRight_X;     // Weighting matrix for X-coordinate
            // Weighting matrix for X-coordinate at final point
            Qy = weightFact_PullOutRight_Y; 	 // Weighting matrix for Y-coordinate
            //            LOG_INFO(adtf_util::cString::Format("----------------------------------"));
            //            LOG_INFO(adtf_util::cString::Format("Heading angle %g",temp_HeadingAngle));
        }
        else
        {
            Qx = weightFact_PullOutRight_Y;     // Weighting matrix for X-coordinate
            // Weighting matrix for X-coordinate at final point
            Qy = weightFact_PullOutRight_X; 	 // Weighting matrix for Y-coordinate
            //            LOG_INFO(adtf_util::cString::Format("----------------------------------"));
            //            LOG_INFO(adtf_util::cString::Format("Heading angle %g",temp_HeadingAngle));
        }
        //        Qx = weightFact_PullOutRight_X;     // Weighting matrix for X-coordinate
        QxN = 5;  // Weighting matrix for X-coordinate at final point
        /*        Qy = weightFact_PullOutRight_Y;*/ 	 // Weighting matrix for Y-coordinate
        QyN = 5;//5; 	 // Weighting matrix for Y-coordinate at final point
        Qpsi = 2; // Weighting factor for heading
        Rv = 1; 	 // Weighting matrix for velocity
        Rd = 1;//3; 	 // Weighting matrix for steering angle
        vcc = no_lane_follow_speed * direction;
        CHGsa = 1.0;
        initialize_bounds();
    }
    else if(MPC_parameter->MPC_car_state_flag == PARKING)
    {
        if (temp_HeadingAngle == 0 || temp_HeadingAngle == PI)
        {
            Qx = weightFact_Parking_X;     // Weighting matrix for X-coordinate
            // Weighting matrix for X-coordinate at final point
            Qy = weightFact_Parking_Y; 	 // Weighting matrix for Y-coordinate
            if (turn_around_reference_counter>2*N)
            {
                Qx = weightFact_Parking_Y;     // Weighting matrix for X-coordinate
                // Weighting matrix for X-coordinate at final point
                Qy = weightFact_Parking_X; 	 // Weighting matrix for Y-coordinate
            }
            //            LOG_INFO(adtf_util::cString::Format("----------------------------------"));
            //            LOG_INFO(adtf_util::cString::Format("Heading angle %g",temp_HeadingAngle));
        }
        else
        {
            Qx = weightFact_Parking_Y;     // Weighting matrix for X-coordinate
            // Weighting matrix for X-coordinate at final point
            Qy = weightFact_Parking_X; 	 // Weighting matrix for Y-coordinate
            if (turn_around_reference_counter>2*N)
            {
                Qx = weightFact_Parking_X;     // Weighting matrix for X-coordinate
                // Weighting matrix for X-coordinate at final point
                Qy = weightFact_Parking_Y; 	 // Weighting matrix for Y-coordinate
            }
            //            LOG_INFO(adtf_util::cString::Format("----------------------------------"));
            //            LOG_INFO(adtf_util::cString::Format("Heading angle %g",temp_HeadingAngle));
        }

        Rd = 1;//3; 	 // Weighting matrix for steering angle
        vcc = 0.45 * direction;
        CHGsa = 0.0;
        initialize_bounds();
    }
    else

    {
        Qx = 20;     // Weighting matrix for X-coordinate
        QxN = 2;  // Weighting matrix     /* KALMAN FILTER */
        Qy = 5; 	 // Weighting matrix for Y-coordinate
        QyN = 5;//5; 	 // Weighting matrix for Y-coordinate at final point
        Qpsi = 2; // Weighting factor for heading
        Rv = 1; 	 // Weighting matrix for velocity
        Rd = 1;//3; 	 // Weighting matrix for steering angle
        vcc = no_lane_follow_speed * direction;
        initialize_bounds();
        //LOG_INFO(adtf_util::cString::Format("SONDERN MODE----------------------------------"));
        //LOG_INFO(adtf_util::cString::Format("MPC_car_state_flag %d avoidance_flag %d", MPC_parameter->MPC_car_state_flag, avoidance_flag ));

    }

    
    
    //        for(int i =0; i< N; i++)
    //            LOG_INFO(adtf_util::cString::Format("MPC Ziel %d Velocity %g X Y: %g   %g",i, vcc, soll->X[i], soll->Y[i]));
    //        LOG_INFO(adtf_util::cString::Format("----------------------------------"));
    
    gettimeofday(&ts, 0);
    ipoptStartTime = ts.tv_sec * 1000000 + ts.tv_usec;
    
    
    if(MPC_parameter->MPC_car_state_flag == LANE_FOLLOW)
    {
        Xini[0] = 0; // X Messwerte
        Xini[1] = 0; // Y Messwerte
        Xini[2] = 0; // Psi Messwerte
    }
    else
    {

        //car_cur_position.X_Position

        Xini[0] =  estimates(0); // X Messwerte
        Xini[1] =  estimates(1); // Y Messwerte
        Xini[2] =  estimates(2); // Psi Messwerte
    }

#ifdef OUTPUT_EKF_DEBUG
    //LOG_INFO(adtf_util::cString::Format("%g %g",estimates(2), car_cur_position.HeadingAngle));
//    LOG_INFO(adtf_util::cString::Format("****SENSORS Heading %g ****", car_cur_position.HeadingAngle));
#endif

    status = app->OptimizeTNLP(mynlp);

    //LOG_INFO(adtf_util::cString::Format("****Current inputs: V=%g Theta=%g ****",XX[3], XX[4]));
    gettimeofday(&ts, 0);
    long ipoptTime = ts.tv_sec * 1000000 + ts.tv_usec;
    ipoptDt = (ipoptTime - ipoptStartTime) / 1000000.;
    //    std::cout << "Ipopt time measurement: " << ipoptDt << std::endl;

    //if (m_log) fprintf(m_log,"%f %f %f %d\n", ipoptDt, car_speed, XX[4], current_car_state_flag);
    
    mpcIdx += 1;
    
    if(XX[4] == 0)
        output_steering = 0;
    else if(XX[4] > 0)
        output_steering = (XX[4] * RADIAN_TO_DEGREES) * POSITIVE_STEERING_ANGLE_TO_PERCENT;
    else if(XX[4] < 0)
        output_steering = (XX[4] * RADIAN_TO_DEGREES) * NEGATIVE_STEERING_ANGLE_TO_PERCENT;
    WriteSignalValue(&steering_output, -output_steering, 0);
    WriteSignalValue(&speed_output, XX[3], 0);

    last_mpc_steering = XX[4];
    last_speed = XX[3];
    last_steering = -output_steering;

    
    //    curvefitting();

    RETURN_NOERROR;
}

tResult SOP_AutonomousDriving::curvefitting(void)
{
 
    {
        if(output_steering == 0)
        {
            car_curve_a = 0;
            car_curve_b = 0;
            car_curve_c = 0;
        }
        else if(output_steering > 0)
        {
            car_curve_a = -output_steering * 0.0000795;
            car_curve_b = -output_steering * 0.00338;
            car_curve_c = 0;
        }
        else if(output_steering < 0)
        {
            car_curve_a = -output_steering * 0.0001662;
            car_curve_b = -output_steering * -0.004729;
            car_curve_c = 0;
        }
    }


    RETURN_NOERROR;
}

tResult SOP_AutonomousDriving::ExtendedKF(void)
{
    /* KALMAN FILTER */
    funcs.resize(3);
    const double h = 0.00500; // Length of the time intervals in [s]

    funcs[0] = dotX1;
    funcs[1] = dotX2;
    funcs[2] = dotX3;
    rk4 = new RungeKutta(funcs, 3);
    vector <double> res;
//    LOG_INFO(adtf_util::cString::Format("****SENSORS Position in X Y: is equal to %g %g****",car_cur_position.X_Position, car_cur_position.Y_Position));
    //     LOG_INFO(adtf_util::cString::Format("****Estimated Position in X Y: is equal to %g %g****",car_est_position.X_Position, car_est_position.Y_Position));
    //LOG_INFO(adtf_util::cString::Format("SENSORS befor Haading: is equal to %g ",car_est_position.HeadingAngle ));
    for(double t = 0; t < DT; t += h)
    {
        // Control and State arrays for Runge-Kutta method
        //double uu[] = {XX[3], XX[4]};
        //double uu[] = {car_speed, XX[4]};
       double uu[] = {car_speed, last_mpc_steering};
  //       double uu[] = {last_speed, last_mpc_steering};

        double xx[] = {estimates(0), estimates(1), estimates(2)};
        // Calculate next step
        res = rk4->calcState(xx, uu, h);

        estimates(0) += res[0];
        estimates(1) += res[1];
        estimates(2) += res[2];
    }


    if(car_cur_position.radius < 0.5 /*&& road_marker_ID != NO_TRAFFIC_SIGN*/)
    {
        sensors(0) = car_cur_position.X_Position; // Should be the data from sensors X
        sensors(1) = car_cur_position.Y_Position;  // Should be the data from sensors Y
        sensors(2) = car_cur_position.HeadingAngle; // Should be the data from sensors PSI

        PP = 0.5;
    }
    else
    {
        sensors(0) = estimates(0); // Should be the data from sensors X
        sensors(1) = estimates(1);  // Should be the data from sensors Y
        sensors(2) = car_cur_position.HeadingAngle; // Should be the data from sensors PSI
    }


    car_est_position.X_Position =  estimates(0); // X Messwerte
    car_est_position.Y_Position =  estimates(1); // Y Messwerte
    car_est_position.HeadingAngle =  estimates(2); // Psi Messwerte



    // Compute transition matrix
    //
    /*F << 1, 0, -sin(estimates(2) + ((lf/(lf+lr))*XX[4]))*XX[3]*DT,
          0, 1,  cos(estimates(2) + ((lf/(lf+lr))*XX[4]))*XX[3]*DT,
          0, 0, 1;*/
    /*
    F << 1, 0, -sin(estimates(2) + ((lf/(lf+lr))*XX[4]))*car_speed*DT,
          0, 1,  cos(estimates(2) + ((lf/(lf+lr))*XX[4]))*car_speed*DT,
          0, 0, 1;

    H << 1, 0, 0,
         0, 1, 0,
         0, 0, 1;

    Q << 0.00001, 0,  0,
          0, 0.000001, 0,
          0,  0, 0.000001;

    R <<  10, 0,  0,
          0,  10, 0,
          0,  0, 10;
    */

//    if(car_cur_position.radius > 0.5)
    {
        PP = FF * PP * FF + QQ;

        //Matrix3d T = (H * P * H.transpose() + R);
        double TT = (HH * PP * HH + RR);
        /*
        I << 1, 0, 0,
             0, 1, 0,
             0, 0, 1;
        */
        //T = T.inverse().eval();
        TT = 1.0/TT;
        //Matrix3d KG = P * H.transpose() * T;
        double KG = PP * HH * TT;
        if(fabs(last_heading - car_cur_position.HeadingAngle) > 3.0)
            estimates(2) =( estimates(2) * -1.0 );
        //estimates = estimates + KG * (sensors - H * estimates);

            estimates(2) = estimates(2) + KG * (sensors(2) - HH * estimates(2));

        //P = (I - KG * H) * P;
        PP = (1.0 - KG * HH) * PP;
    }

    last_heading = car_cur_position.HeadingAngle;

    /* KALMAN FILTER END */

    tFloat32 EKF_position_value[5];

    EKF_position_value[0] = estimates(0);
    EKF_position_value[1] = estimates(1);
    EKF_position_value[2] = car_cur_position.radius;
    EKF_position_value[3] = car_speed;
    EKF_position_value[4] = estimates(2);

    WritePinArrayValue(&EKF_position_output, 5,position_output_ID_name, EKF_position_value);
//    LOG_INFO(adtf_util::cString::Format("****Estimated Position in X Y: is equal to %g %g****",EKF_position_value[0],EKF_position_value[1]));

    RETURN_NOERROR;
}


tResult SOP_AutonomousDriving::ResetExtendedKF(void)
{
    estimates(0) = car_cur_position.X_Position ;
    estimates(1) = car_cur_position.Y_Position;
    estimates(2) = car_cur_position.HeadingAngle;
    last_heading = car_cur_position.HeadingAngle;
    //    LOG_INFO(adtf_util::cString::Format("ResetExtendedKF position_value[X]    [Y] %f %f  Heading %f", estimates(0), estimates(1), estimates(2)));

    RETURN_NOERROR;
}


tResult SOP_AutonomousDriving::CloseIpopt(void)
{
    app->~IpoptApplication();
    app->~ReferencedObject();
    free(soll);
    free(MPC_parameter);
    
    
    RETURN_NOERROR;
}


void SOP_AutonomousDriving::initialize_bounds(){
    
    xupper[0] = 1e19;    // x-coordinate
    xupper[1] = 1e19;//0.05;     // y-coordinate
    xupper[2] = 2 * PI;    // psi
    xupper[3] = vcc;    //vcc// velocity
    xupper[4] = 0.436;//0.585;   // steering angle [rad]
    xlower[0] = -1e19;//0;
    xlower[1] = -1e19;//-0.05;
    xlower[2] = -2 * PI;
    xlower[3] = vcc;    //vcc// velocity
    xlower[4] = -0.49;//-0.585                                                                                                                                                      ;
}


double dotX1(double *sVars, double *uu, double h)
{
    return h * (uu[0]*cos((sVars[2]+((lf/l)*uu[1]))));
}

double dotX2(double *sVars, double *uu, double h)
{
    return h * (uu[0]*sin((sVars[2]+((lf/l)*uu[1]))));
}

double dotX3(double *sVars, double *uu, double h)
{
    return h * ((uu[0]/(lf + lr))*tan(uu[1]));
}



