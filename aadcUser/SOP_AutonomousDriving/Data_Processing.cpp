#include "SOP_AutonomousDriving.h"



tResult SOP_AutonomousDriving::CalculateUltrasonicWorldCoordinate(tFloat32 *ult_value)
{
    float radian = 0;


    radian = 150 * DEGREES_TO_RADIAN;
    ult_world_coord[F_LEFT][X] = (ult_value[0] * sin(radian)) + 26; //X is
    ult_world_coord[F_LEFT][Y] = (ult_value[0] * cos(radian)) - 10; //Y is ->

    radian = 120 * DEGREES_TO_RADIAN;
    ult_world_coord[F_CENTER_LEFT][X] = (ult_value[2] * sin(radian)) + 28; //X is
    ult_world_coord[F_CENTER_LEFT][Y] = (ult_value[2] * cos(radian)) - 5; //Y is ->

    ult_world_coord[F_CENTER][X] = ult_value[4] + 29 ; //X is
    ult_world_coord[F_CENTER][Y] = 0; //Y is ->

    radian =60 * DEGREES_TO_RADIAN;
    ult_world_coord[F_CENTER_RIGHT][X] = (ult_value[6] * sin(radian)) + 28; //X is
    ult_world_coord[F_CENTER_RIGHT][Y] = (ult_value[6] * cos(radian)) + 5; //Y is ->

    radian = 30 * DEGREES_TO_RADIAN;
    ult_world_coord[F_RIGHT][X] = (ult_value[8] * sin(radian)) + 26; //X is
    ult_world_coord[F_RIGHT][Y] = (ult_value[8] * cos(radian)) + 10; //Y is ->



    ult_world_coord[S_LEFT][X] = 15; //X is
    ult_world_coord[S_LEFT][Y] = -ult_value[10] - 15; //Y is ->

    ult_world_coord[S_RIGHT][X] = 15; //X is
    ult_world_coord[S_RIGHT][Y] = ult_value[12] + 15; //Y is ->



    radian = 210 * DEGREES_TO_RADIAN;
    ult_world_coord[R_LEFT][X] = (ult_value[14] * sin(radian)) - 30; //X is
    ult_world_coord[R_LEFT][Y] = (ult_value[14] * cos(radian)) - 10; //Y is ->

    ult_world_coord[R_CENTER][X] = -ult_value[16] - 30; //X is
    ult_world_coord[R_CENTER][Y] = 0; //Y is ->

    radian = 330 * DEGREES_TO_RADIAN;
    ult_world_coord[R_RIGHT][X] = (ult_value[18] * sin(radian)) - 30; //X is
    ult_world_coord[R_RIGHT][Y] = (ult_value[18] * cos(radian)) + 10; //Y is ->


//    LOG_INFO(adtf_util::cString::Format("Front L to R: x:%.0f , y:%.0f; x:%.0f , y:%.0f; x:%.0f , y:%.0f; x:%.0f , y:%.0f; x:%.0f , y:%.0f;", ult_world_coord[F_LEFT][X],         ult_world_coord[F_LEFT][Y],
//                                        ult_world_coord[F_CENTER_LEFT][X],  ult_world_coord[F_CENTER_LEFT][Y],
//                                        ult_world_coord[F_CENTER][X],       ult_world_coord[F_CENTER][Y],
//                                        ult_world_coord[F_CENTER_RIGHT][X], ult_world_coord[F_CENTER_RIGHT][Y],
//                                        ult_world_coord[F_RIGHT][X],        ult_world_coord[F_RIGHT][Y]));

    //    LOG_INFO(adtf_util::cString::Format("Side  L to R: %.0f , %.0f; %.0f , %.0f;", ult_world_coord[S_LEFT][X], ult_world_coord[S_LEFT][Y],ult_world_coord[S_RIGHT][X], ult_world_coord[S_RIGHT][Y]));

    //    LOG_INFO(adtf_util::cString::Format("Rear  L to R: %.0f , %.0f; %.0f , %.0f; %.0f , %.0f;", ult_world_coord[R_LEFT][X],   ult_world_coord[R_LEFT][Y],
    //                                                                                                ult_world_coord[R_CENTER][X], ult_world_coord[R_CENTER][Y],
    //                                                                                                ult_world_coord[R_RIGHT][X],  ult_world_coord[R_RIGHT][Y]));


    RETURN_NOERROR;
}

tResult SOP_AutonomousDriving::CalculateTrackingPoint(void)
{
#define BREAK_ERROR 0.3
    //Draw Tracking Point
    int index = 0;

    float distance = 0;
    float lane_ref_distance = 0;
    float error = 0;
    char loop_flag = 1;


    if(reference_value[0] == LTRACE || reference_value[0] == SL_TRACE)
    {
        lane_ref_distance =  ((0.5 * 100) * DT);   // m/s -> cm/s * DT

#if(CAMERA_DISTANCE == 0)
        ref_lane_coord_in_image.X[0] = 1;
#else
        ref_lane_coord_in_image.X[0] = CAMERA_DISTANCE;
#endif
        ref_lane_coord_in_image.Y[0] = (reference_value[1] * (ref_lane_coord_in_image.X[index] * ref_lane_coord_in_image.X[index])) + (ref_lane_coord_in_image.X[index] * reference_value[2]) + reference_value[3];
        index = 1;
        ref_lane_coord_in_image.X[index] = ref_lane_coord_in_image.X[0];

        while(loop_flag == 1)
        {
            ref_lane_coord_in_image.Y[index] = (reference_value[1] * (ref_lane_coord_in_image.X[index] * ref_lane_coord_in_image.X[index])) + (ref_lane_coord_in_image.X[index] * reference_value[2]) + reference_value[3];


            distance = GetDistanceBetweenCoordinates(ref_lane_coord_in_image.X[index], ref_lane_coord_in_image.Y[index] , ref_lane_coord_in_image.X[index - 1], ref_lane_coord_in_image.Y[index - 1]);

            ref_lane_coord_in_image.X[index] += 0.3;

            error = distance -  lane_ref_distance;

            if(error > BREAK_ERROR)
            {
                index++;
                if(index > N+1)
                    loop_flag = 0;
                else
                    ref_lane_coord_in_image.X[index] = ref_lane_coord_in_image.X[index - 1];
            }

        }

        for(index = 0; index <= N; index++)
        {
            ref_lane_world_coord.X[index] = ref_lane_coord_in_image.X[index] * 0.01;  //cm to meter
            ref_lane_world_coord.Y[index] = -ref_lane_coord_in_image.Y[index] * 0.01;
        }

    }

    RETURN_NOERROR;
}


float SOP_AutonomousDriving::GetDistanceBetweenCoordinates(float x2, float y2, float x1, float y1)
{
    return fabs((sqrt(pow((x2 - x1), 2)+ pow((y2 - y1),2))));

}

/* Calculate reference point for trajectory tracking in MPC:
Mauneuver includes: turn_left, turn_right, parking, turn_out_left, turn_out_right
Global coordinates are used here. The origin point comes from estimated position: car_est_position.X_Position & car_est_position.Y_Position
Reference trajectories are calculated using Bezier curve with four points （https://en.wikipedia.org/wiki/B%C3%A9zier_curve）:
B(t) = P1*(1-t)^3 + P2*(1-t)^2*t + P3*(1-t)*t^2 + P4*t^3， where 0<=t<=1
The steps to construct reference trajectories are stated as follows:
Step 1: In the first round (i.e., (turn_around_reference_counter == 0) ), defined the entire reference trajectories depending on difference cases
Step 2: In each round (roundIdx), give N points to ref_lane_world_coord.X & ref_lane_world_coord.Y
*/
tResult SOP_AutonomousDriving::CalculateTurnAroundReferencePoint(char status_flag, int roundIdx)
{
    int index = 0;
    double tt = 0;
    int indexout = 0;
    float temp_x = 0;
    float temp_y = 0;


    switch (status_flag)
    {

    case AVOIDANCE:

#ifdef AUTO_A
        if (avoidance.comeback_flag == 0)
        {
            pt1[X] = 0   ; pt1[Y]=0;
            pt2[X] = 0.5 ; pt2[Y]=0;
            pt3[X] = 0.5; pt3[Y]=0.46;
            pt4[X] = 1.0; pt4[Y]=0.46;
        }
        else if (avoidance.comeback_flag == 2)
        {
            pt1[X] = 0   ; pt1[Y]=0;
            pt2[X] = 0.75 ; pt2[Y]=0;
            pt3[X] = 0.75; pt3[Y]=-0.44;
            pt4[X] = 1.5; pt4[Y]=-0.44;
        }
#else
        if (avoidance.comeback_flag == 0)
        {
            pt1[X] = 0   ; pt1[Y]=0;
            pt2[X] = 0.5 ; pt2[Y]=0;
            pt3[X] = 0.5; pt3[Y]=0.46;
            pt4[X] = 1.0; pt4[Y]=0.46;
        }
        else if (avoidance.comeback_flag == 2)
        {
            pt1[X] = 0   ; pt1[Y]=0;
            pt2[X] = 0.25 ; pt2[Y]=0;
            pt3[X] = 0.75; pt3[Y]=-0.42;
            pt4[X] = 1; pt4[Y]=-0.42;
        }
#endif



        // bezier curve Calculate
        if(turn_around_reference_counter == 0) //turn_around_reference_counter: round counter
        {
            temp_HeadingAngle = car_est_position.HeadingAngle;
            if (car_est_position.HeadingAngle >= -PI/4 && car_est_position.HeadingAngle <= PI/4 )
                temp_HeadingAngle = 0;
            else if (car_est_position.HeadingAngle >= PI/4 && car_est_position.HeadingAngle <= 3*PI/4 )
                temp_HeadingAngle = PI/2;
            else if (car_est_position.HeadingAngle >= -3*PI/4  && car_est_position.HeadingAngle <= -PI/4 )
                temp_HeadingAngle = -PI/2;
            else
                temp_HeadingAngle = PI;

            for(index = 0; index < 2*N; index++)
            {
                tt = (index+1)/(double)(2*N);

                avoidance_ref_coord.X[index] = pow((1-tt), 3)*pt1[X] + 3*pow((1-tt), 2)*tt*pt2[X] + 3*pow(tt, 2)*(1-tt)*pt3[X]+pow(tt, 3)*pt4[X];
                avoidance_ref_coord.Y[index] = pow((1-tt), 3)*pt1[Y] + 3*pow((1-tt), 2)*tt*pt2[Y] + 3*pow(tt, 2)*(1-tt)*pt3[Y]+pow(tt, 3)*pt4[Y];

#ifdef OUTPUT_BEZIER_CURVE_DEBUG
                LOG_INFO(adtf_util::cString::Format("Coordinate 1 %d  X=%f  Y=%f  ",index, avoidance_ref_coord.X[index], avoidance_ref_coord.Y[index]));
#endif

            }
            for(index = 2*N; index < 3*N; index++)
            {
                avoidance_ref_coord.X[index] = avoidance_ref_coord.X[index-1]+0.5;
                avoidance_ref_coord.Y[index] = pt4[Y];
            }

            // rotate and shift the reference trajectory to new coordinates
            for(index = 0; index < 3*N; index++)
            {
                temp_x = (cos(temp_HeadingAngle) * avoidance_ref_coord.X[index]) - (sin(temp_HeadingAngle) * avoidance_ref_coord.Y[index]) + car_est_position.X_Position;
                temp_y = (sin(temp_HeadingAngle) * avoidance_ref_coord.X[index]) + (cos(temp_HeadingAngle) * avoidance_ref_coord.Y[index]) + car_est_position.Y_Position;
                avoidance_ref_coord.X[index] = temp_x;
                avoidance_ref_coord.Y[index] = temp_y;

#ifdef OUTPUT_BEZIER_CURVE_DEBUG
                LOG_INFO(adtf_util::cString::Format("Coordinate%d  X=%f  Y=%f  ",index, avoidance_ref_coord.X[index], avoidance_ref_coord.Y[index]));
#endif

            }

        }

        // write reference point
        for(index = 0; index < N; index++)
        {

            indexout = roundIdx+index ;

            ref_lane_world_coord.X[index] = avoidance_ref_coord.X[indexout];
            ref_lane_world_coord.Y[index] = avoidance_ref_coord.Y[indexout];


        }

        break;
        /* Turn left includes
     * 3*N points with Bezier curve
     * 1*N points with prolonged straight line
    */
    case TURN_LEFT:

#ifdef AUTO_A
        pt1[X] = 0   ; pt1[Y]=0;
        pt2[X] = 0.8 ; pt2[Y]=0;
        pt3[X] = 1.20; pt3[Y]=0.3;
        pt4[X] = 1.20; pt4[Y]=1.1;
#else
        pt1[X] = 0   ; pt1[Y]=0;
        pt2[X] = 0.95 ; pt2[Y]=0;
        pt3[X] = 1.20; pt3[Y]=0.3;
        pt4[X] = 1.20; pt4[Y]=1.1;
#endif

        // bezier curve Calculate
        if(turn_around_reference_counter == 0) //turn_around_reference_counter: round counter
        {
            temp_HeadingAngle = car_est_position.HeadingAngle;
            if (car_est_position.HeadingAngle >= -PI/4 && car_est_position.HeadingAngle <= PI/4 )
                temp_HeadingAngle = 0;
            else if (car_est_position.HeadingAngle >= PI/4 && car_est_position.HeadingAngle <= 3*PI/4 )
                temp_HeadingAngle = PI/2;
            else if (car_est_position.HeadingAngle >= -3*PI/4  && car_est_position.HeadingAngle <= -PI/4 )
                temp_HeadingAngle = -PI/2;
            else
                temp_HeadingAngle = PI;

            for(index = 0; index < 3*N; index++)
            {
                tt = (index+1)/(double)(3*N);

                turn_left_ref_coord.X[index] = pow((1-tt), 3)*pt1[X] + 3*pow((1-tt), 2)*tt*pt2[X] + 3*pow(tt, 2)*(1-tt)*pt3[X]+pow(tt, 3)*pt4[X];
                turn_left_ref_coord.Y[index] = pow((1-tt), 3)*pt1[Y] + 3*pow((1-tt), 2)*tt*pt2[Y] + 3*pow(tt, 2)*(1-tt)*pt3[Y]+pow(tt, 3)*pt4[Y];

#ifdef OUTPUT_BEZIER_CURVE_DEBUG
                LOG_INFO(adtf_util::cString::Format("Coordinate 1 %d  X=%f  Y=%f ",index, turn_left_ref_coord.X[index], turn_left_ref_coord.Y[index]));
#endif

            }
            for(index = 3*N; index < 4*N; index++)
            {
                turn_left_ref_coord.X[index] = pt4[X];
                turn_left_ref_coord.Y[index] = turn_left_ref_coord.Y[index-1] + 0.01;
                //LOG_INFO(adtf_util::cString::Format("Index %d Coordinate  X= %f  Y= %f  ",index, turn_left_ref_coord.X[index], turn_left_ref_coord.Y[index]));
            }

            for(index = 0; index < 4*N; index++)
            {
                temp_x = (cos(temp_HeadingAngle) * turn_left_ref_coord.X[index]) - (sin(temp_HeadingAngle) * turn_left_ref_coord.Y[index]) + car_est_position.X_Position;
                temp_y = (sin(temp_HeadingAngle) * turn_left_ref_coord.X[index]) + (cos(temp_HeadingAngle) * turn_left_ref_coord.Y[index]) + car_est_position.Y_Position;
                turn_left_ref_coord.X[index] = temp_x;
                turn_left_ref_coord.Y[index] = temp_y;

#ifdef OUTPUT_BEZIER_CURVE_DEBUG
                LOG_INFO(adtf_util::cString::Format("Coordinate 2 %d  X=%f  Y=%f ",index, turn_left_ref_coord.X[index], turn_left_ref_coord.Y[index]));
#endif

            }

        }

        // write reference point
        for(index = 0; index < N; index++)
        {

            indexout = roundIdx+index ;

            ref_lane_world_coord.X[index] = turn_left_ref_coord.X[indexout];
            ref_lane_world_coord.Y[index] = turn_left_ref_coord.Y[indexout];


        }
        //            for(int i =0; i< N; i++)
        //                LOG_INFO(adtf_util::cString::Format("Ref %d X Y: %g   %g",i, ref_lane_world_coord.X[i], ref_lane_world_coord.Y[i]));
        //            LOG_INFO(adtf_util::cString::Format("----------------------------------"));
        break;

        /* Turn right includes
         * N points with Bezier curve
         * N points with prolonged straight line
        */
    case TURN_RIGHT:


#ifdef AUTO_A
        pt1[X] = 0  ; pt1[Y]=0;
        pt2[X] = 0.4; pt2[Y]=0;
        pt3[X] = 0.64; pt3[Y]=-0.3;
        pt4[X] = 0.64; pt4[Y]=-0.8;
#else
        pt1[X] = 0  ; pt1[Y]=0;
        pt2[X] = 0.5; pt2[Y]=0;
        pt3[X] = 0.65; pt3[Y]=-0.3;
        pt4[X] = 0.65; pt4[Y]=-0.8;
#endif

        // bezier curve Calculate
        if(turn_around_reference_counter == 0)
        {
            temp_HeadingAngle = car_est_position.HeadingAngle;
            if (car_est_position.HeadingAngle >= -PI/4 && car_est_position.HeadingAngle <= PI/4 )
                temp_HeadingAngle = 0;
            else if (car_est_position.HeadingAngle >= PI/4 && car_est_position.HeadingAngle <= 3*PI/4 )
                temp_HeadingAngle = PI/2;
            else if (car_est_position.HeadingAngle >= -3*PI/4  && car_est_position.HeadingAngle <= -PI/4 )
                temp_HeadingAngle = -PI/2;
            else
                temp_HeadingAngle = PI;

            for(index = 0; index < 2*N; index++)
            {
                tt = (index+1)/(double)(2*N);
                turn_right_ref_coord.X[index] = pow((1-tt), 3)*pt1[X] + 3*pow((1-tt), 2)*tt*pt2[X] + 3*pow(tt, 2)*(1-tt)*pt3[X]+pow(tt, 3)*pt4[X];
                turn_right_ref_coord.Y[index] = pow((1-tt), 3)*pt1[Y] + 3*pow((1-tt), 2)*tt*pt2[Y] + 3*pow(tt, 2)*(1-tt)*pt3[Y]+pow(tt, 3)*pt4[Y];


            }
            for(index = 2*N; index < 3*N; index++)
            {
                //turn_right_ref_coord.X[index] = 0.80+car_est_position.X_Position;
                turn_right_ref_coord.X[index] = pt4[X];
                turn_right_ref_coord.Y[index] = turn_right_ref_coord.Y[index-1] - 0.01;

            };

            // rotate and shift the reference trajectory to new coordinates
            for(index = 0; index < 3*N; index++)
            {
                temp_x = (cos(temp_HeadingAngle) * turn_right_ref_coord.X[index]) - (sin(temp_HeadingAngle) * turn_right_ref_coord.Y[index]) + car_est_position.X_Position;
                temp_y = (sin(temp_HeadingAngle) * turn_right_ref_coord.X[index]) + (cos(temp_HeadingAngle) * turn_right_ref_coord.Y[index]) + car_est_position.Y_Position;
                turn_right_ref_coord.X[index] = temp_x;
                turn_right_ref_coord.Y[index] = temp_y;
#ifdef OUTPUT_BEZIER_CURVE_DEBUG
                LOG_INFO(adtf_util::cString::Format("Coordinate%d  X=%f  Y=%f  ",index, turn_right_ref_coord.X[index], turn_right_ref_coord.Y[index]));
#endif
            }
        }

        // write reference point
        for(index = 0; index < N; index++)
        {
            indexout = roundIdx+index ;

            ref_lane_world_coord.X[index] = turn_right_ref_coord.X[indexout];  //cm to meter
            ref_lane_world_coord.Y[index] = turn_right_ref_coord.Y[indexout];
        }
        break;

    case STRAIGHT:

#ifdef AUTO_A
        pt1[X] = 0  ; pt1[Y]=0;
        pt2[X] = 0.3; pt2[Y]=0;
        pt3[X] = 0.65; pt3[Y]=0;
        pt4[X] = 2; pt4[Y]=0;
#else
        pt1[X] = 0  ; pt1[Y]=0;
        pt2[X] = 0.3; pt2[Y]=0;
        pt3[X] = 0.65; pt3[Y]=0;
        pt4[X] = 2; pt4[Y]=0;
#endif

        // bezier curve Calculate
        if(turn_around_reference_counter == 0)
        {
            temp_HeadingAngle = car_est_position.HeadingAngle;
            if (car_est_position.HeadingAngle >= -PI/4 && car_est_position.HeadingAngle <= PI/4 )
                temp_HeadingAngle = 0;
            else if (car_est_position.HeadingAngle >= PI/4 && car_est_position.HeadingAngle <= 3*PI/4 )
                temp_HeadingAngle = PI/2;
            else if (car_est_position.HeadingAngle >= -3*PI/4  && car_est_position.HeadingAngle <= -PI/4 )
                temp_HeadingAngle = -PI/2;
            else
                temp_HeadingAngle = PI;

                        LOG_INFO(adtf_util::cString::Format("----------------------------------"));
                        LOG_INFO(adtf_util::cString::Format("Heading angle %g",temp_HeadingAngle));
            for(index = 0; index < 5*N; index++)
            {
                tt = (index+1)/(double)(5*N);
                straight_ref_coord.X[index] = pow((1-tt), 3)*pt1[X] + 3*pow((1-tt), 2)*tt*pt2[X] + 3*pow(tt, 2)*(1-tt)*pt3[X]+pow(tt, 3)*pt4[X];
                straight_ref_coord.Y[index] = pow((1-tt), 3)*pt1[Y] + 3*pow((1-tt), 2)*tt*pt2[Y] + 3*pow(tt, 2)*(1-tt)*pt3[Y]+pow(tt, 3)*pt4[Y];


            }


            // rotate and shift the reference trajectory to new coordinates
            for(index = 0; index < 5*N; index++)
            {
                temp_x = (cos(temp_HeadingAngle) * straight_ref_coord.X[index]) - (sin(temp_HeadingAngle) * straight_ref_coord.Y[index]) + car_est_position.X_Position;
                temp_y = (sin(temp_HeadingAngle) * straight_ref_coord.X[index]) + (cos(temp_HeadingAngle) * straight_ref_coord.Y[index]) + car_est_position.Y_Position;
                straight_ref_coord.X[index] = temp_x;
                straight_ref_coord.Y[index] = temp_y;
//#ifdef OUTPUT_BEZIER_CURVE_DEBUG
                LOG_INFO(adtf_util::cString::Format("Coordinate%d  X=%f  Y=%f  ",index, straight_ref_coord.X[index], straight_ref_coord.Y[index]));
//#endif
            }
        }

        // write reference point
        for(index = 0; index < N; index++)
        {
            indexout = roundIdx+index ;

            ref_lane_world_coord.X[index] = straight_ref_coord.X[indexout];  //cm to meter
            ref_lane_world_coord.Y[index] = straight_ref_coord.Y[indexout];
        }
        break;

        /* Pullout left includes
         * 3*N points with Bezier curve
         * 2*N points with prolonged straight line
        */
    case PULL_OUT_LEFT:

#ifdef AUTO_A
        pt1[X] = 0   ; pt1[Y]=0;
        pt2[X] = 0.7 ; pt2[Y]=0;
        pt3[X] = 1.12; pt3[Y]=0.42;
        pt4[X] = 1.12; pt4[Y]=0.96;
#else
        pt1[X] = 0   ; pt1[Y]=0;
        pt2[X] = 0.7 ; pt2[Y]=0;
        pt3[X] = 1.12; pt3[Y]=0.42;
        pt4[X] = 1.12; pt4[Y]=0.96;
#endif

        // bezier curve Calculate
        if(turn_around_reference_counter == 0)
        {
            temp_HeadingAngle = car_est_position.HeadingAngle;
            if (car_est_position.HeadingAngle >= -PI/4 && car_est_position.HeadingAngle <= PI/4 )
                temp_HeadingAngle = 0;
            else if (car_est_position.HeadingAngle >= PI/4 && car_est_position.HeadingAngle <= 3*PI/4 )
                temp_HeadingAngle = PI/2;
            else if (car_est_position.HeadingAngle >= -3*PI/4  && car_est_position.HeadingAngle <= -PI/4 )
                temp_HeadingAngle = -PI/2;
            else
                temp_HeadingAngle = PI;


            for(index = 0; index < 3*N; index++)
            {
                tt = (index+1)/(double)(3*N);
                pullout_left_ref_coord.X[index] = pow((1-tt), 3)*pt1[X] + 3*pow((1-tt), 2)*tt*pt2[X] + 3*pow(tt, 2)*(1-tt)*pt3[X]+pow(tt, 3)*pt4[X];
                pullout_left_ref_coord.Y[index] = pow((1-tt), 3)*pt1[Y] + 3*pow((1-tt), 2)*tt*pt2[Y] + 3*pow(tt, 2)*(1-tt)*pt3[Y]+pow(tt, 3)*pt4[Y];


            }
            for(index = 3*N; index < 4*N; index++)
            {
                pullout_left_ref_coord.X[index] = pt4[X];
                pullout_left_ref_coord.Y[index] = pullout_left_ref_coord.Y[index-1] + 0.02;
            }

            // rotate and shift the reference trajectory to new coordinates
            for(index = 0; index < 4*N; index++)
            {
                temp_x = (cos(temp_HeadingAngle) * pullout_left_ref_coord.X[index]) - (sin(temp_HeadingAngle) * pullout_left_ref_coord.Y[index]) + car_est_position.X_Position;
                temp_y = (sin(temp_HeadingAngle) * pullout_left_ref_coord.X[index]) + (cos(temp_HeadingAngle) * pullout_left_ref_coord.Y[index]) + car_est_position.Y_Position;
                pullout_left_ref_coord.X[index] = temp_x;
                pullout_left_ref_coord.Y[index] = temp_y;
#ifdef OUTPUT_BEZIER_CURVE_DEBUG
                LOG_INFO(adtf_util::cString::Format("Coordinate%d  X=%f  Y=%f  ",index, pullout_left_ref_coord.X[index], pullout_left_ref_coord.Y[index]));
#endif
            }

        }

        // write reference point
        for(index = 0; index < N; index++)
        {

            indexout = roundIdx+index ;

            ref_lane_world_coord.X[index] = pullout_left_ref_coord.X[indexout];
            ref_lane_world_coord.Y[index] = pullout_left_ref_coord.Y[indexout];


        }
        //            for(int i =0; i< N; i++)
        //                LOG_INFO(adtf_util::cString::Format("Ref %d X Y: %g   %g",i, turn_left_ref_coord.X[i], turn_left_ref_coord.Y[i]));
        //            LOG_INFO(adtf_util::cString::Format("----------------------------------"));
        break;

        /* Pullout right includes
         * N points with Bezier curve
         * N points with prolonged straight line
        */
    case PULL_OUT_RIGHT:

#ifdef AUTO_A
        pt1[X] = 0  ; pt1[Y]=0;
        pt2[X] = 0.5; pt2[Y]=0;
        pt3[X] = 0.75; pt3[Y]=-0.5;
        pt4[X] = 0.75; pt4[Y]=-0.8;
#else
        pt1[X] = 0  ; pt1[Y]=0;
        pt2[X] = 0.5; pt2[Y]=0;
        pt3[X] = 0.75; pt3[Y]=-0.5;
        pt4[X] = 0.75; pt4[Y]=-0.8;
#endif

        // bezier curve Calculate
        if(turn_around_reference_counter == 0)
        {

            temp_HeadingAngle = car_est_position.HeadingAngle;
            if (car_est_position.HeadingAngle >= -PI/4 && car_est_position.HeadingAngle <= PI/4 )
                temp_HeadingAngle = 0;
            else if (car_est_position.HeadingAngle >= PI/4 && car_est_position.HeadingAngle <= 3*PI/4 )
                temp_HeadingAngle = PI/2;
            else if (car_est_position.HeadingAngle >= -3*PI/4  && car_est_position.HeadingAngle <= -PI/4 )
                temp_HeadingAngle = -PI/2;
            else
                temp_HeadingAngle = PI;


            for(index = 0; index < 3*N; index++)
            {
                tt = (index+1)/(double)(3*N);
                pullout_right_ref_coord.X[index] = pow((1-tt), 3)*pt1[X] + 3*pow((1-tt), 2)*tt*pt2[X] + 3*pow(tt, 2)*(1-tt)*pt3[X]+pow(tt, 3)*pt4[X];
                pullout_right_ref_coord.Y[index] = pow((1-tt), 3)*pt1[Y] + 3*pow((1-tt), 2)*tt*pt2[Y] + 3*pow(tt, 2)*(1-tt)*pt3[Y]+pow(tt, 3)*pt4[Y];
            }
            for(index = 3*N; index < 4*N; index++)
            {
                pullout_right_ref_coord.X[index] = pt4[X];
                pullout_right_ref_coord.Y[index] = pullout_right_ref_coord.Y[index-1] - 0.01;
            }

            // rotate and shift the reference trajectory to new coordinates
            for(index = 0; index < 4*N; index++)
            {
                temp_x = (cos(temp_HeadingAngle) * pullout_right_ref_coord.X[index]) - (sin(temp_HeadingAngle) * pullout_right_ref_coord.Y[index]) + car_est_position.X_Position;
                temp_y = (sin(temp_HeadingAngle) * pullout_right_ref_coord.X[index]) + (cos(temp_HeadingAngle) * pullout_right_ref_coord.Y[index]) + car_est_position.Y_Position;
                pullout_right_ref_coord.X[index] = temp_x;
                pullout_right_ref_coord.Y[index] = temp_y;
#ifdef OUTPUT_BEZIER_CURVE_DEBUG
                LOG_INFO(adtf_util::cString::Format("Coordinate%d  X=%f  Y=%f  ",index, pullout_right_ref_coord.X[index], pullout_right_ref_coord.Y[index]));
#endif
            }

        }

        // write reference point
        for(index = 0; index < N; index++)
        {
            indexout = roundIdx+index ;

            ref_lane_world_coord.X[index] = pullout_right_ref_coord.X[indexout];  //cm to meter
            ref_lane_world_coord.Y[index] = pullout_right_ref_coord.Y[indexout];
        }
        break;

        /* Parking includes
         * 4*N points with Bezier curve (forward part)
         * N points with connecting point
         * 4*N points with Bezier curve (backward part)
         * N points with terminate point
        */
    case PARKING:

#ifdef AUTO_A
        pt1[X] = 0  ; pt1[Y]=0;
        pt2[X] = 0; pt2[Y]=0;
        pt3[X] = 0.1; pt3[Y]=0;
        pt4[X] = 0.5; pt4[Y]=0.36;

        pt5[X] = 0    ; pt5[Y]=0   ;
        pt6[X] = -0.60; pt6[Y]=0;
        pt7[X] = -0.68; pt7[Y]=-0.30;
        pt8[X] = -0.68; pt8[Y]=-1.50;
#else

                    pt1[0] = 0; pt1[1]=0;
                    pt2[0] = 0.2; pt2[1]=0;
                    pt3[0] = 0.4; pt3[1]=0.15;
                    pt4[0] = 0.4; pt4[1]=0.33;

                    pt5[0] = 0; pt5[1]=0;
                    pt6[0] = -0.15; pt6[1]=0;
                    pt7[0] = -0.26; pt7[1]=-0.4;
                    pt8[0] = -0.30; pt8[1]=-1.2;
#endif

        // bezier curve Calculate
        if(turn_around_reference_counter == 0)
        {
            temp_HeadingAngle = car_est_position.HeadingAngle;
            if (car_est_position.HeadingAngle >= -PI/4 && car_est_position.HeadingAngle <= PI/4 )
                temp_HeadingAngle = 0;
            else if (car_est_position.HeadingAngle >= PI/4 && car_est_position.HeadingAngle <= 3*PI/4 )
                temp_HeadingAngle = PI/2;
            else if (car_est_position.HeadingAngle >= -3*PI/4  && car_est_position.HeadingAngle <= -PI/4 )
                temp_HeadingAngle = -PI/2;
            else
                temp_HeadingAngle = PI;

            for(index = 0; index < N; index++)
            {
                tt = (index+1)/(double)(N);
                parking_ref_coord.X[index] = pow((1-tt), 3)*pt1[X] + 3*pow((1-tt), 2)*tt*pt2[X] + 3*pow(tt, 2)*(1-tt)*pt3[X]+pow(tt, 3)*pt4[X];//car_est_position.X_Position;
                parking_ref_coord.Y[index] = pow((1-tt), 3)*pt1[Y] + 3*pow((1-tt), 2)*tt*pt2[Y] + 3*pow(tt, 2)*(1-tt)*pt3[Y]+pow(tt, 3)*pt4[Y];//car_est_position.Y_Position;
            }

            for(index = N; index < 2*N; index++)
            {
                parking_ref_coord.X[index] = pt4[X];//pt4[X];//+car_est_position.X_Position;
                parking_ref_coord.Y[index] = pt4[Y];//pt4[Y];//+car_est_position.Y_Position;
                // LOG_INFO(adtf_util::cString::Format("ORiginal Coordinate 1 %d  X=%f  Y=%f  ",index, parking_ref_coord.X[index], parking_ref_coord.Y[index]));

            }

            //            // rotate and shift the reference trajectory to new coordinates
            //            for (index=0;index<4*N;index++)
            //            {
            //                //LOG_INFO(adtf_util::cString::Format("Ref %d X Y: %g   %g",index, parking_ref_coord.X[index]+car_est_position.X_Position, parking_ref_coord.Y[index]+car_est_position.Y_Position));
            //            }
            // rotate and shift the reference trajectory to new coordinates
            for(index = 0; index < 2*N; index++)
            {
                //LOG_INFO(adtf_util::cString::Format("ORiginal Coordinate 2 %d  X=%f  Y=%f  ",index, parking_ref_coord.X[index], parking_ref_coord.Y[index]));
                temp_x = (cos(temp_HeadingAngle) * parking_ref_coord.X[index]) - (sin(temp_HeadingAngle) * parking_ref_coord.Y[index]) + car_est_position.X_Position;
                temp_y = (sin(temp_HeadingAngle) * parking_ref_coord.X[index]) + (cos(temp_HeadingAngle) * parking_ref_coord.Y[index]) + car_est_position.Y_Position;
                parking_ref_coord.X[index] = temp_x;
                parking_ref_coord.Y[index] = temp_y;
#ifdef OUTPUT_BEZIER_CURVE_DEBUG
                LOG_INFO(adtf_util::cString::Format("Coordinate%d  X=%f  Y=%f  ",index, parking_ref_coord.X[index], parking_ref_coord.Y[index]));
#endif
            }


        }

        //Update the backward curve with current position
        if(turn_around_reference_counter == 2*N)
        {
//            LOG_INFO(adtf_util::cString::Format("Temp heading %g  ",temp_HeadingAngle));

            for(index = 2*N; index < 5*N; index++)
            {
                tt = (index-2*N+1)/(double)(2*N);
                parking_ref_coord.X[index] = pow((1-tt), 3)*pt5[X] + 3*pow((1-tt), 2)*tt*pt6[X] + 3*pow(tt, 2)*(1-tt)*pt7[X]+pow(tt, 3)*pt8[X];//+car_est_position.X_Position;
                parking_ref_coord.Y[index] = pow((1-tt), 3)*pt5[Y] + 3*pow((1-tt), 2)*tt*pt6[Y] + 3*pow(tt, 2)*(1-tt)*pt7[Y]+pow(tt, 3)*pt8[Y];//+car_est_position.Y_Position;
            }

            for(index = 5*N; index < 6*N; index++)
            {
                parking_ref_coord.X[index] = pt8[X];//+car_est_position.X_Position;
                parking_ref_coord.Y[index] = pt8[Y];//+car_est_position.Y_Position;

            }
            for(index = 2*N; index < 6*N; index++)
            {
                temp_x = (cos(temp_HeadingAngle) * parking_ref_coord.X[index]) - (sin(temp_HeadingAngle) * parking_ref_coord.Y[index]) + car_est_position.X_Position;
                temp_y = (sin(temp_HeadingAngle) * parking_ref_coord.X[index]) + (cos(temp_HeadingAngle) * parking_ref_coord.Y[index]) + car_est_position.Y_Position;
                parking_ref_coord.X[index] = temp_x;
                parking_ref_coord.Y[index] = temp_y;
//                LOG_INFO(adtf_util::cString::Format("Coordinate%d  X=%f  Y=%f  ",index, parking_ref_coord.X[index], parking_ref_coord.Y[index]));
            }

        }




        // write reference point
        for(index = 0; index < N; index++)
        {
            indexout = roundIdx+index ;
            ref_lane_world_coord.X[index] = parking_ref_coord.X[indexout];
            ref_lane_world_coord.Y[index] = parking_ref_coord.Y[indexout];

        }
        //            for(int i =0; i< N; i++)
        //                LOG_INFO(adtf_util::cString::Format("Ref %d X Y: %g   %g",i, turn_left_ref_coord.X[i], turn_left_ref_coord.Y[i]));
        //            LOG_INFO(adtf_util::cString::Format("----------------------------------"));
        break;

    default:
        break;
    }
    RETURN_NOERROR;
}

void SOP_AutonomousDriving::CalculateCoefficient(double *answer, double *x, double *y, int size_of_arrays, int degree_of_polynomial)
{
    int i,j,k;
    double X[2 * degree_of_polynomial + 1];                        //Array that will store the values of sigma(xi),sigma(xi^2),sigma(xi^3)....sigma(xi^2n)
    double B[degree_of_polynomial + 1][degree_of_polynomial + 2];            //B is the Normal matrix(augmented) that will store the equations, 'a' is for value of the final coefficients
    double Y[degree_of_polynomial + 1];                    //Array to store the values of sigma(yi),sigma(xi*yi),sigma(xi^2*yi)...sigma(xi^n*yi)

    for (i = 0;i < 2 * degree_of_polynomial + 1; i++)
    {
        X[i]=0;
        for (j = 0; j < size_of_arrays;j++)
            X[i]=X[i]+pow(x[j],i);        //consecutive positions of the array will store N,sigma(xi),sigma(xi^2),sigma(xi^3)....sigma(xi^2n)
    }

    for (i = 0; i <= degree_of_polynomial;i++)
        for (j = 0;j <= degree_of_polynomial;j++)
            B[i][j]=X[i+j];            //Build the Normal matrix by storing the corresponding coefficients at the right positions except the last column of the matrix


    for (i = 0;i < degree_of_polynomial+1; i++)
    {
        Y[i]=0;
        for (j = 0; j < size_of_arrays; j++)
            Y[i]=Y[i]+pow(x[j],i)*y[j];        //consecutive positions will store sigma(yi),sigma(xi*yi),sigma(xi^2*yi)...sigma(xi^n*yi)
    }
    for (i = 0 ;i <= degree_of_polynomial;i++)
        B[i][degree_of_polynomial + 1]=Y[i];                //load the values of Y as the last column of B(Normal Matrix but augmented)
    degree_of_polynomial = degree_of_polynomial + 1;                //n is made n+1 because the Gaussian Elimination part below was for n equations, but here n is the degree of polynomial and for n degree we get n+1 equations


    /*    cout<<"\nThe Normal(Augmented Matrix) is as follows:\n";
    for (i = 0; i < degree_of_polynomial; i++)            //print the Normal-augmented matrix
    {
        for (j = 0;j <= degree_of_polynomial;j++)
            cout<<B[i][j]<<setw(16);
        cout<<"\n";
    }*/


    for (i = 0; i < degree_of_polynomial; i++)                    //From now Gaussian Elimination starts(can be ignored) to solve the set of linear equations (Pivotisation)
        for (k = i + 1; k < degree_of_polynomial; k++)
            if (B[i][i]<B[k][i])
                for (j = 0;j <= degree_of_polynomial; j++)
                {
                    double temp = B[i][j];
                    B[i][j]=B[k][j];
                    B[k][j]=temp;
                }

    for (i = 0;i < degree_of_polynomial - 1; i++)            //loop to perform the gauss elimination
        for (k = i + 1; k < degree_of_polynomial; k++)
        {
            double t=B[k][i]/B[i][i];
            for (j = 0; j <= degree_of_polynomial; j++)
                B[k][j]=B[k][j]-t*B[i][j];    //make the elements below the pivot elements equal to zero or elimnate the variables
        }
    for (i = degree_of_polynomial - 1; i >= 0;i--)                //back-substitution
    {                        //x is an array whose values correspond to the values of x,y,z..
        answer[i] = B[i][degree_of_polynomial];                //make the variable to be calculated equal to the rhs of the last equation
        for (j = 0; j < degree_of_polynomial; j++)
            if (j != i)            //then subtract all the lhs values except the coefficient of the variable whose value                                   is being calculated
                answer[i] = answer[i]-B[i][j] * answer[j];
        answer[i] = answer[i] / B[i][i];            //now finally divide the rhs by the coefficient of the variable to be calculated
    }


}
