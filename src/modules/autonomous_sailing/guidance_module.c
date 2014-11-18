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

/** @brief PI controller with conditional integration*/
float pi_controller(const float *ref_p, const float *meas_p,
                    const struct parameters_qgc *param_qgc_p);

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

/** Implement reference actions provided by optimal path planning*/
void guidance_module(const struct reference_actions_s *ref_act_p,
                     const struct parameters_qgc *param_qgc_p,
                     struct structs_topics_s *strs_p,
                     const struct published_fd_s *pubs_p){

    float alpha;
    float command = 0.0f;

    //TODO check tack action

    //get alpha from the moving average value of the last k values of instant alpha
    alpha = get_alpha();

    //PI controller for rudder
    command = pi_controller(&(ref_act_p->alpha_star), &alpha, param_qgc_p);

    //saturation for safety reason
    if(command > RUDDER_SATURATION)
        command = RUDDER_SATURATION;
    else if(command < -RUDDER_SATURATION)
        command = -RUDDER_SATURATION;

    //TODO sailing control

    //debug
//    printf("alpha_star %2.3f \t alpha %2.3f \t command %2.3f \n",
//          (double)(ref_act_p->alpha_star), (double)alpha, (double)command);

//    //cancella
//    strs_p->debug_att.timestamp = hrt_absolute_time();
    strs_p->debug_att.roll = ref_act_p->alpha_star;
    strs_p->debug_att.pitch = alpha;
//    strs_p->debug_att.yaw = command;

//    orb_publish(ORB_ID(vehicle_attitude), pubs_p->debug_att, &(strs_p->debug_att));

//    //fine cancella


    //update actuator value
    strs_p->actuators.control[0] = command;
    // actuators.control[0] -> output channel 1
    // actuators.control[2] -> output channel 3
    // actuators.control[3] -> output channel 4

    //publish debug value for post-processing
    strs_p->boat_guidance_debug.timestamp = hrt_absolute_time();
    strs_p->boat_guidance_debug.alpha_star = ref_act_p->alpha_star;
    strs_p->boat_guidance_debug.alpha = alpha;
    strs_p->boat_guidance_debug.rudder_action = command;

    orb_publish(ORB_ID(boat_guidance_debug), pubs_p->boat_guidance_debug_pub, &(strs_p->boat_guidance_debug));

}
