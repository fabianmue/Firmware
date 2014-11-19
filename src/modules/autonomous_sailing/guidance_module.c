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

static float sum_error_pi = 0.0f; ///error sum from iterations of guidance_module, used in PI

static bool boat_is_tacking = false;///true if boat is performing a tack maneuver

static float tack_rudder_command = 0.0f;

static float roll_before_tack[2] = {0.0f, 0.0f}; ///roll angle (from ekf and weather station) before starting tack maneuver

/** @brief PI controller with conditional integration*/
float pi_controller(const float *ref_p, const float *meas_p,
                    const struct parameters_qgc *param_qgc_p);

/** @brief if the boat should tack, perform tack maneuver*/
float tack_action(struct reference_actions_s *ref_act_p,
                  struct structs_topics_s *strs_p);

/** @brief determine if the tack maneuver is finished*/
bool stop_tack_action(float angle, uint8_t index_roll);

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
    i_conditioned = param_qgc_p->i_gain / (1 + error * error);

    //update sum error
    sum_error_pi += error;

    //TODO check if sum_error_pi has to be set to 0 once in a while

    //with Dumas angle convention, we have to make a negative feedback law control
    action = -(param_qgc_p->p_gain * error + i_conditioned * sum_error_pi);

    return action;
}

/** Perform tack maneuver.
 *
 * Change rudder command to full saturation, opposite in sign of rudder command before tack.
 * Keep on this rudder command until stop_tack_action() is false.
 * When the tack maneuver is completed, updated reference param by setting should_tack to false.
 *
 * @param ref_act_p pointer to reference action
 * @param strs_p    pointer to topics data
 * @return          rudder command
*/
float tack_action(struct reference_actions_s *ref_act_p, struct structs_topics_s *strs_p){

    float command = 0.0f;

    //we are here beacuse ref_act_p->should_tack is true, so boat should tack
    if(boat_is_tacking){
        //we have started the tack maneuver, check if stop it or keep it on
        if(stop_tack_action(strs_p->att.roll, 0) || stop_tack_action(strs_p->boat_weather_station.roll_r, 1)){

            //we have just completed the tack maneuver
            ref_act_p->should_tack = false;//so PI controller can compute new rudder commnad
            boat_is_tacking = false;

        }
        else{
            //keep on tack maneuver
            command = tack_rudder_command;
        }
    }
    else{
        //we must start tack maneuver
        boat_is_tacking = true;
        //invert rudder command and set it to the maximum value
        tack_rudder_command = (strs_p->actuators.control[0] > 0)? -RUDDER_SATURATION:
                                                                  RUDDER_SATURATION;
        //save actual roll angles
        roll_before_tack[0] = strs_p->att.roll;
        roll_before_tack[1] = strs_p->boat_weather_station.roll_r;

        //set command for rudder
        command = tack_rudder_command;
    }

    return command;
}

/** Determine when a tack maneuver is completed
 *
 * If roll angle has changed in sign from the roll angle before tack, and if
 * this roll angle is greater/less than half of previous roll angle changed in sign,
 * then tack action is completed
 *
 * @param angle         actual roll angle(from ekf or weather station)
 * @param index_roll    0 if roll angle from ekf, 1 if roll angle from weather station
 * @return              true if the tack action is completed
 */
bool stop_tack_action(float angle, uint8_t index_roll){
    bool stop = false;

    if(roll_before_tack[index_roll] > 0){
        if(angle <= (-roll_before_tack[index_roll] / 2.0f))
            stop = true;
    }
    else if(roll_before_tack[index_roll] < 0){
        if(angle >= (-roll_before_tack[index_roll] / 2.0f))
            stop = true;
    }

    return stop;
}

/** Implement reference actions provided by optimal path planning*/
void guidance_module(struct reference_actions_s *ref_act_p,
                     const struct parameters_qgc *param_qgc_p,
                     struct structs_topics_s *strs_p,
                     const struct published_fd_s *pubs_p){

    float alpha;
    float command = 0.0f;

    //get alpha from the moving average value of the last k values of instant alpha
    alpha = get_alpha();

    if(ref_act_p->should_tack){

        command = tack_action(ref_act_p, strs_p);
    }
    if(!(ref_act_p->should_tack)){
        //if the boat should not tack, compute rudder action to follow reference alpha

        //PI controller for rudder
        command = pi_controller(&(ref_act_p->alpha_star), &alpha, param_qgc_p);
    }

    //saturation for safety reason
    if(command > RUDDER_SATURATION)
        command = RUDDER_SATURATION;
    else if(command < -RUDDER_SATURATION)
        command = -RUDDER_SATURATION;

    //TODO sailing control



    //update actuator value
    strs_p->actuators.control[0] = command;
    // actuators.control[0] -> rudder
    // actuators.control[3] -> sail

    //publish debug value for post-processing
    strs_p->boat_guidance_debug.timestamp = hrt_absolute_time();
    strs_p->boat_guidance_debug.alpha_star = ref_act_p->alpha_star;
    strs_p->boat_guidance_debug.alpha = alpha;
    strs_p->boat_guidance_debug.rudder_action = command;

    orb_publish(ORB_ID(boat_guidance_debug), pubs_p->boat_guidance_debug_pub, &(strs_p->boat_guidance_debug));

}
