/****************************************************************************
 *
 *   Copyright (c) 2012, 2013 PX4 Development Team. All rights reserved.
 *   Author: Marco Tranzatto <marco.tranzatto@gmail.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/
/**
 * @file guidance_module.c
 *
 * Guidance module.
 * Implementation of controller(s) used to make the boat sailing autonomously.
 *
 * @author Marco Tranzatto <marco.tranzatto@gmail.com>
 */

#include "guidance_module.h"

#define M_PI_F 3.14159265358979323846f

static float sum_error_pi = 0.0f; ///error sum from iterations of guidance_module, used in PI

//static data for tack action
static struct{
    bool boat_is_tacking;
    float tack_rudder_command;
    float roll_before_tack[2];
    float yaw_before_tack[2];
    float roll_stop_tack;
    float yaw_stop_tack;
}tack_data =    {
                    .boat_is_tacking = false,//true if boat is performing a tack maneuver
                    .tack_rudder_command = 0.0f,
                    .roll_before_tack[0] = 0.0f,
                    .roll_before_tack[1] = 0.0f,
                    .yaw_before_tack[0] = 0.0f,
                    .yaw_before_tack[1] = 0.0f,
                    .roll_stop_tack = 2.0f,
                    .yaw_stop_tack = 1.04f //more or less 60 degress
                };

//static data for sails controller
static struct{
    float position_quantum;
    float command_quantum;
}sail_controller_data = {
                            .position_quantum = M_PI_F/4.0f, //initial guess: 4 available positions
                            .command_quantum = SAIL_SATURATION/4.0f //initial guess: 4 available positions
                        };


/** @brief PI controller with conditional integration*/
float pi_controller(const float *ref_p, const float *meas_p,
                    const struct parameters_qgc *param_qgc_p);

/** @brief if the boat should tack, perform tack maneuver*/
float tack_action(struct reference_actions_s *ref_act_p,
                  struct structs_topics_s *strs_p);

/** @brief determine if the tack maneuver is finished*/
bool is_tack_completed(const struct structs_topics_s *strs_p);

/** Check first condition for stopping tack, @see is_tack_completed*/
bool roll_stop_tack(float angle, uint8_t index_roll);

/** Check second condition for stopping tack, @see is_tack_completed*/
bool yaw_stop_tack(float angle, uint8_t index_yaw);

/** @brief set the stop_tack value used in stop_tack_action()*/
void set_stop_tack(float roll_stop, float yaw_stop);

/** @brief simple controller for sails*/
float sail_controller();

/**
 * set the stop_tack value used in stop_tack_action()
*/
void set_stop_tack(float roll_stop, float yaw_stop){
    tack_data.roll_stop_tack = roll_stop;
    //convert value from deg to rad
    tack_data.yaw_stop_tack = yaw_stop * M_PI_F / 180.0f;
}

/** PI controller to compute the input for the rudder servo motor.
 *
 * Perform ret(t) = P * error(t) + I_c * sum_{k = 0}^{t}(error(k)) where
 * the constant I_c = I / (1 + error(t)^2). P and I are qcg parameters.
 *
 * @param ref_p         pointer to reference value
 * @param meas_p        pointer to measurements value
 * @param param_qgc_p   pointer to struct containing parameters from qgc
 *
 * @return input value for actuator of rudder
*/
float pi_controller(const float *ref_p, const float *meas_p,
                    const struct parameters_qgc *param_qgc_p){

    float error;
    float i_conditioned; //integral condition constant
    float action;

    error = *ref_p - *meas_p;

    //integral constant for conditional integration, this is for anti-wind up!
    i_conditioned = param_qgc_p->rudder_i_gain / (1.0f + error * error);

    //update sum error
    sum_error_pi += error;

    //TODO check if sum_error_pi has to be set to 0 once in a while

    //command = P * error + I * sum{error}
    action = param_qgc_p->rudder_p_gain * error + i_conditioned * sum_error_pi;

    return action;
}

/** Perform tack maneuver.
 *
 * Change rudder command to full saturation, opposite in sign of rudder command before tack.
 * Keep on this rudder command until stop_tack_action() is false.
 * When the tack maneuver is completed, updated reference param by setting should_tack to false.
 *
 * @param ref_act_p     pointer to reference action
 * @param strs_p        pointer to topics data
 * @return              rudder command
*/
float tack_action(struct reference_actions_s *ref_act_p,
                  struct structs_topics_s *strs_p){

    float command = 0.0f;

    //we are here beacuse ref_act_p->should_tack is true, so boat should tack
    if(tack_data.boat_is_tacking){
        //we have started the tack maneuver, check if stop it or keep it on
        if(is_tack_completed(strs_p)){
            //we have just completed the tack maneuver
            ref_act_p->should_tack = false;//so PI controller can compute new rudder commnad
            tack_data.boat_is_tacking = false;

            //notify to path_planning that we've completed the tack action
            notify_tack_completed();

        }
        else{
            //keep on tack maneuver
            command = tack_data.tack_rudder_command;
        }
    }
    else{
        //we must start tack maneuver
        tack_data.boat_is_tacking = true;
        //invert rudder command and set it to the maximum value
        tack_data.tack_rudder_command = (strs_p->actuators.control[0] > 0) ? (-RUDDER_SATURATION):
                                                                              RUDDER_SATURATION;
        //save actual roll and yaw angles
        tack_data.roll_before_tack[0] = strs_p->att.roll;
        tack_data.roll_before_tack[1] = strs_p->boat_weather_station.roll_r;
        tack_data.yaw_before_tack[0] = strs_p->att.yaw;
        tack_data.yaw_before_tack[1] = strs_p->boat_weather_station.heading_tn;

        //set command for rudder
        command = tack_data.tack_rudder_command;
    }

    return command;
}

/** Determine when a tack maneuver is completed
 *
 * Tack is completed if two conditions are both true.
 * 1)If roll (either from Kalman filter or weather station) angle has changed in sign
 * from the roll angle before tack, and if
 * this roll angle is greater/less than previous roll angle / roll_stop_tack, changed in sign,
 * then first condition is met. Roll_stop_tack value is set by set_stop_tack function.
 * 2)If yaw (either from Kalman filter or weather station) angle has decreased/increased by
 * of yaw_stop_tack rads, then second condition is met.
 * Yaw_stop_tack value is set by set_stop_tack function.
 *
 * @param strs_p        pointer to topics values
 * @return              true if the tack action is completed
 */
bool is_tack_completed(const struct structs_topics_s *strs_p){

    bool first_cond = false;
    bool second_cond = false;

    //check first condition on roll angles (by Kalman filter app and Weather station)
    first_cond = roll_stop_tack(strs_p->att.roll, 0) ||
                     roll_stop_tack(strs_p->boat_weather_station.roll_r, 1);

    //check second condition on yaw angles (by Kalman filter app and Weather station)
    second_cond = yaw_stop_tack(strs_p->att.yaw, 0) ||
                     yaw_stop_tack(strs_p->boat_weather_station.heading_tn, 1);

    #if SIMULATION_FLAG == 1

//    float pos_p[] = {0.1f,
//                     0.2f,
//                     0.3f,
//                     0.4f};

//    float val_p[] = {roll_stop_tack(strs_p->boat_weather_station.roll_r, 1),
//                    (float)yaw_stop_tack(strs_p->att.yaw, 0),
//                     strs_p->att.yaw,
//                    tack_data.yaw_before_tack[0]};

//    print_debug_mode(pos_p, val_p, sizeof(pos_p) / sizeof(float), strs_p);
    #endif

    //return logic "and" between these two conditions
    return (first_cond && second_cond);
}

/**
 * Check first condition for stopping tack, @see is_tack_completed.
 *
 * @param angle         actual roll angle to check
 * @param index_roll    index to access to tack_data.roll_before_tack vector
 * @return              true if second condition on this angle is met
*/
bool roll_stop_tack(float angle, uint8_t index_roll){
    bool stop = false;

    if(tack_data.roll_before_tack[index_roll] > 0){
        if(angle <= (-tack_data.roll_before_tack[index_roll] / tack_data.roll_stop_tack))
            stop = true;
    }
    else if(tack_data.roll_before_tack[index_roll] < 0){
        if(angle >= (-tack_data.roll_before_tack[index_roll] / tack_data.roll_stop_tack))
            stop = true;
    }

    return stop;
}

/**
 * Check second condition for stopping tack, @see is_tack_completed.
 *
 * @param angle         actual yaw angle to check, positive from North to South passing through East
 * @param index_yaw     index to access to tack_data.yaw_before_tack vector
 * @return              true if second condition on this angle is met
 *
*/
bool yaw_stop_tack(float angle, uint8_t index_yaw){
    bool stop = false;

    /**
     * angle can be either the taw angle provided by Kalman filter (implemented in ekf_att_pos_estimator app),
     * or the heading angle provided by the weather station magnetic sensor (measurements published in
     * parser_200WX app). We will refer to these two angle as a general yaw angle called "angle".
     * This angle provided by sensors is assumed to be positive on the East side (that is, going from
     * North to South, passing through East), and it's negative on the West side (going from
     * North to South, passing through West).
     * Angle is 0 on the true North, -pi/2 on West, -pi on South, pi/2 on East, etc.
     *
     * If the boat during the tack maneuver is steering on the left, we must pay attention if the
     * stern passes through South coming from the West side and going to the East side.
     * If the boat is steering on the right, we must pay attention if the stern passes through South
     * coming from the East side and going to the West side.
     * In these special cases, we must "extend" the angle, ranging from -2pi to 2pi to avoid
     * possibile errors due to yaw missing measurements by sensors.
    */

    if(tack_data.tack_rudder_command > 0){
        /* we're steering on the left, if yaw/heading angle before tacking was negative
         * (West side) we must "extend" angle if it has swtiched from a negative
         * value to a positive one.
        */
        if(tack_data.yaw_before_tack[index_yaw] < 0)
            angle = (angle < 0) ? angle : -2 * M_PI_F + angle;

        /* see if the difference between final and initial angle is at least yaw_stop_tac,
         * remeber that angles have a sign!!!
        */
        stop = ((angle - tack_data.yaw_before_tack[index_yaw]) <= -tack_data.yaw_stop_tack);
    }
    else{
        /* we're steering on the right, if yaw/heading angle before tacking was positive
         * (East side) we must "extend" angle if it has swtiched from a positive
         * value to a negative one.
        */
        if(tack_data.yaw_before_tack[index_yaw] > 0)
            angle = (angle > 0) ? angle : 2 * M_PI_F + angle;

        /* see if the difference between final and initial angle is at least yaw_stop_tac,
         * remeber that angles have a sign!!!
        */
        stop = ((angle - tack_data.yaw_before_tack[index_yaw]) >= tack_data.yaw_stop_tack);
    }


    return stop;
}

/**
 * Simple controller for sails.
*/
float sail_controller(){

    float abs_angle;
    float command;
    int32_t sector;
    float mean_apparent;

    mean_apparent = get_app_wind();

    abs_angle = (mean_apparent > 0) ? mean_apparent :
                                    -(mean_apparent);

    /*
     * See in which sector (from 0 to num set by set_sail_positions()) the absolute value of
     * apparent wind direction is.
    */
    sector = (int32_t)(abs_angle / sail_controller_data.position_quantum);

    /*
     * If it is in sector 0, then tighten the sail (giving SAIL_SATURATION as command).
     * If it is in the last sector, ease off the sail (giving 0 as command).
     * Use a linear value in a middle sector.
    */
    command = SAIL_SATURATION - (sector * sail_controller_data.command_quantum);

    return command;
}

/**
 * Set a new value for the numbers of positions at which the sail can be at.
 * There are "num" positions available for the sails.
*/
void set_sail_positions(int32_t num){

    /*There are "num" available positions for the sail. Each of the
     * is large pi / num. In this way the absolute value of apparent wind is in a sector
     * numbered from 0 to num-1.
    */

    sail_controller_data.position_quantum = M_PI_F / (float)num;
    sail_controller_data.command_quantum = SAIL_SATURATION / (float)num;
}

/** Implement reference actions provided by optimal path planning*/
void guidance_module(struct reference_actions_s *ref_act_p,
                     const struct parameters_qgc *param_qgc_p,
                     struct structs_topics_s *strs_p){

    float alpha;
    float rudder_command = 0.0f;
    float sail_command = 0.0f;

    //rudder control

    //get alpha from the moving average value of the last k values of instant alpha
    alpha = get_alpha();

    if(ref_act_p->should_tack){

        rudder_command = tack_action(ref_act_p, strs_p);
    }
    if(!(ref_act_p->should_tack)){
        //if the boat should not tack, compute rudder action to follow reference alpha

        //PI controller for rudder
        rudder_command = pi_controller(&(ref_act_p->alpha_star), &alpha, param_qgc_p);
    }

    //sails control only if AS_SAIL param from QGC is negative
    if(param_qgc_p->sail_servo < 0.0f)
        sail_command = sail_controller();
    else
        sail_command = param_qgc_p->sail_servo;

    //saturation for safety reason
    if(rudder_command > RUDDER_SATURATION)
        rudder_command = RUDDER_SATURATION;
    else if(rudder_command < -RUDDER_SATURATION)
        rudder_command = -RUDDER_SATURATION;

    if(sail_command < 0.0f)
        sail_command = 0.0f;
    else if(sail_command > SAIL_SATURATION)
        sail_command = SAIL_SATURATION;

    //update actuator value
    strs_p->actuators.control[0] = rudder_command;
    strs_p->actuators.control[3] = sail_command;



    //save first debug values for post-processing, other values set in path_planning()
    strs_p->boat_guidance_debug.timestamp = hrt_absolute_time();
    strs_p->boat_guidance_debug.alpha = alpha;
    strs_p->boat_guidance_debug.rudder_action = rudder_command;
    strs_p->boat_guidance_debug.sail_action = sail_command;
}
