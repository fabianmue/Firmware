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
#define TWO_M_PI_F 6.28318530717959f

static char txt_msg[250]; ///used to send messages to QGC
static const float deg2rad = 0.0174532925199433f; // pi / 180

//static float sum_error_pi = 0.0f; ///error sum from iterations of guidance_module, used in PI

//static data for tack action
static struct{
    bool boat_is_tacking;       ///true if boat is performing a tack maneuver
    uint16_t tack_type;         ///type of tack action to perform
    bool sailing_at_port_haul;///true if the boat was sailing on port haul before tacking
    float alpha_min_stop_tack_r;///minimum value of alpha in the new haul in order to stop tacking
}tack_data =    {
                    .boat_is_tacking = false,//true if boat is performing a tack maneuver
                    .tack_type = 0, //tack as helmsam would do
                    .sailing_at_port_haul = true,//dummy initial guess
                    .alpha_min_stop_tack_r = 0.610865238f //35.0f * deg2rad
                };

 //static data for sails controller
static struct{
    float sail_closed_cmd;      ///sail command when sails should be considered closed
    float alpha_sail_closed_r;  ///absolute alpha angle[rad] where sails should start opening
    float alpha_sail_opened_r;  ///absolute alpha angle[rad] where sails should be fully opend
    float positive_slope;       ///positive linear slope to compute sails cmd when alpha in [alpha_sail_closed_r, alpha_sail_opened_r]
}sail_controller_data = {
                            .sail_closed_cmd = 0.56f, //SAIL_SATURATION,
                            .alpha_sail_closed_r =  0.785398163f,//45.0f * deg2rad,
                            .alpha_sail_opened_r = 2.617993878f,//150.0f * deg2rad,
                            .positive_slope = 0.56f /  1.832595714f
                            //SAIL_SATURATION / (150.0f * deg2rad - 45.0f * deg2rad)
                        };

//static data for rudder control
static struct{
    float p;                ///proportional gain
    float i;                ///integral gain
    float kaw;              ///constant fo anti-wind up in normal digital PI
    float cp;                ///constant for condition integral, in proportional action
    float ci;                ///constant for condition integral, in integral action
    uint8_t rudder_controller_type; /// 0 = standard PI 1 = conditional PI 2 = LQR
    float last_command;     ///last command provided by the PI
    float sum_error_pi;     ///error sum from iterations of guidance_module, used in PI
    float rud_cmd_45_left;  ///rudder command to set the rudder at 45 deg and make boat steer left
    float alpha_rudder_x1_r;   ///alpha angle[rad] called x1 in rudder command plots
    float alpha_rudder_x2_r;   ///alpha angle[rad] called x2 in rudder command plots
} rudder_controller_data =  {
                                .p = 0.0f,
                                .i = 0.0f,
                                .kaw = 0.5f,
                                .cp = 1.0f,
                                .ci = 1.0f,
                                .rudder_controller_type = 0,//start with standard PI
                                .last_command = 0.0f,
                                .sum_error_pi = 0.0f,
                                .rud_cmd_45_left = 0.85f,//RUDDER_45_LEFT,
                                .alpha_rudder_x2_r =  0.0f,
                                .alpha_rudder_x2_r =  0.3490658503f//20.0f * deg2rad
                            };

//static data for LQR and MPC controllers
static struct {
    float state_extended_model[3];///state of the model used by LQR and MPC
    float rudder_latest; ///latest rudder command gave by either LQR or MPC
    //static data for MPC
    mpc_boatTack_params mpc_boatTack_params_s; ///params to fill before calling the solver
    mpc_boatTack_output mpc_boatTack_output_s; ///optimal rudder control
    mpc_boatTack_info mpc_boatTack_info_s; ///solver output
    uint64_t time_last_mpc; ///time when last MPC was computed
    //static data for LQR
    float lqr_gain[3];
    uint64_t time_last_lqr; ///time when last LQR was computed
} optimal_control_data =    {
                                .rudder_latest = 0.0f,
                                .time_last_mpc = 0,
                                .time_last_lqr = 0
                            };

/** @brief PI controller with conditional integration*/
float pi_controller(const float *ref_p, const float *meas_p);

/** @brief if the boat should tack, perform tack maneuver*/
void tack_action(struct reference_actions_s *ref_act_p,
                 float *p_rudder_cmd, float *p_sails_cmd,
                 struct structs_topics_s *strs_p);

/** @brief determine if the tack maneuver is finished*/
bool is_tack_completed(void);

/** @brief rule based control system for sails during upwind sailing*/
float sail_controller(float alpha);

/** @brief saturation on command to the rudder servo motor*/
float rudder_saturation(float command);

/** @brief saturation on command to the sails servo motor*/
float sail_saturation(float command);

/** @brief compute rudder and sails command to perform 'helmsman0' 's tack manuever port to starboard*/
void helmsman0_tack_p2s(float alpha, float *p_rud, float* p_sails);

/** @brief compute rudder and sails command to perform 'helmsman0' 's tack manuever starobard to por*/
void helmsman0_tack_s2p(float alpha, float *p_rud, float* p_sails);

/** @brief compute rudder and sails command to perform 'helmsman1' 's tack manuever port to starboard*/
void helmsman1_tack_p2s(float alpha, float *p_rud, float* p_sails);

/** @brief compute rudder and sails command to perform 'helmsman1' 's tack manuever starobard to por*/
void helmsman1_tack_s2p(float alpha, float *p_rud, float* p_sails);

/** @brief tack maneuver as helmsman would do*/
void helmsman_tack(struct reference_actions_s *ref_act_p,
                   float *p_rudder_cmd, float *p_sails_cmd);

/** @brief action to perform when tack maneuver is completed*/
void tack_completed(struct reference_actions_s *ref_act_p);

/** @brief control rudder using LQR controller*/
void lqr_control_rudder(float *p_rudder_cmd,
                        const struct reference_actions_s *ref_act_p,
                        struct structs_topics_s *strs_p);

/** @brief control rudder using MPC controller*/
void mpc_control_rudder(float *p_rudder_cmd,
                        const struct reference_actions_s *ref_act_p,
                        struct structs_topics_s *strs_p);

/** @brief compute actual state of the extended model used by both LQR and MPC*/
void compute_state_extended_model(const struct reference_actions_s *ref_act_p);

/** @brief dummy abs function for float value*/
float my_fabs(float x){
    x = (x > 0.0f) ? x : -x;
    return x;
}

/**
 * Set LQR gain.
 *
*/
void set_lqr_gain(float lqr_k1, float lqr_k2, float lqr_k3){
    optimal_control_data.lqr_gain[0] = lqr_k1;
    optimal_control_data.lqr_gain[1] = lqr_k2;
    optimal_control_data.lqr_gain[2] = lqr_k3;
}

/**
 * Set data of the sail controller.
 *
 * The sail controller is in charge to regulate the sails angle while sailing.
 * This controller is valid in every zone we are sailing at.
 * Moreover it is in charge to set the sails angle even during the tack maneuver.
 *
 * @param sail_closed_cmd     command to sails when they should be considered closed
 * @param alpha_sail_closed_r absolute angle[rad] where sails should star opening
 * @param aplha_sail_opend_r  absolute angle[rad] where sails should be fully opened
 *
*/
void set_sail_data(float sail_closed_cmd, float alpha_sail_closed_r, float alpha_sail_opened_r){

    //make sure sail_closed_cmd is in the saturation sails limits
    sail_closed_cmd = sail_saturation(sail_closed_cmd);

    //test that both alpha_sail_closed_r and alpha_sail_opened_r are positive
    alpha_sail_closed_r = my_fabs(alpha_sail_closed_r);
    alpha_sail_opened_r = my_fabs(alpha_sail_opened_r);

    //save data into sail_controller_data structure
    sail_controller_data.sail_closed_cmd = sail_closed_cmd;
    sail_controller_data.alpha_sail_closed_r = alpha_sail_closed_r;
    sail_controller_data.alpha_sail_opened_r = alpha_sail_opened_r;

    /*update positive_slope. Remeber that 0.0f as a command to sails means that
     * the sails are fully opened. So, to compute the positive slope to apply at the sail controller
     * when alpha is [alpha_sail_closed_r, alpha_sail_opened_r] we should use sail_closed_cmd as
     * increment on the Y axis to compute the slope.
    */
    sail_controller_data.positive_slope = sail_closed_cmd / (alpha_sail_opened_r - alpha_sail_closed_r);

}


/**
 * Tack maneuver can be performed in five different ways.
 * The first one (type is equal to 0) is performed by using a "standard" input sequence
 * as a real helmsman would do. @see helmsman0_tack_p2s and @see helmsman0_tack_s2p.
 *
 * The second one (type is equal to 1) is slightly different from the "standard" helmsman maneuver
 * but it is still reasonable to think of it as a maneuver that helmsman would do.
 * @see helmsman1_tack_p2s and @see helmsman1_tack_s2p.
 *
 * The Third one (type is equal to 2) is performed by changing only the reference
 * angle with respect to the wind (alpha) and then "wait" for the PI controller
 * of the rudder to follow this changing.
 *
 * The fourth one (type equal to 3) is performed by a LQR controller.
 *
 * The fifth one (type equal to 4) is performed by a MPC controller.
 *
 * @param tack_type               type of tack to perform
 * @param alpha_min_stop_tack_r   minimum absolute value of alpha[rad] in the haul to stop tack
*/
void set_tack_data(uint16_t tack_type, float alpha_min_stop_tack_r){
    uint16_t old_tack_type = tack_data.tack_type;
    //save new value
    tack_data.tack_type = tack_type;
    /*make sure that alpha_min is positive! Since alpha_min is the minimum value
     * at wich the tack from port haul to starboard haul can be considered finished,
     * it has to be a positive value.
    */
    tack_data.alpha_min_stop_tack_r = my_fabs(alpha_min_stop_tack_r);

    //notify the changing to QGroundControl what kind of tack the boat will do
    if(old_tack_type != tack_type){
        if(tack_type == 0)
            sprintf(txt_msg, "Tack as helmsman0.");
        else if(tack_type == 1)
            sprintf(txt_msg, "Tack as helmsman1.");
        else if(tack_type == 2)
            sprintf(txt_msg, "Implicit (PI) Tack.");
        else if(tack_type == 3)
            sprintf(txt_msg, "LQR Tack.");
        else if(tack_type == 4)
            sprintf(txt_msg, "MPC Tack.");

        send_log_info(txt_msg);
    }
}


/** PI controller to compute the input for the rudder servo motor.
 *
 * If a conditional PI is in use, perform ret(t) = P * error(t) + I_c * sum_{k = 0}^{t}(error(k))
 * where the constant I_c = I / (1 + c * error(t)^2).
 * Otherwise use a normal digital PI: ret(t) = P * error(t) + I * sum_{k = 0}^{t}(error(k))
 * P and I are qcg parameters.
 *
 * @param ref_p         pointer to reference value
 * @param meas_p        pointer to measurements value
 * @param param_qgc_p   pointer to struct containing parameters from qgc
 *
 * @return input value for actuator of rudder
*/
float pi_controller(const float *ref_p, const float *meas_p){

    float error;
    float integral_part = 0.0f;
    float proportional_part = 0.0f;
    float action;

    error = *ref_p - *meas_p;

    if(rudder_controller_data.rudder_controller_type == 1){
        //Conditional Integration
        float abs_error;

        abs_error = (error > 0) ? error : -error;
        //update sum error
        rudder_controller_data.sum_error_pi += error;

        //integral constant for conditional integration, this is for anti-wind up!
        float i_gain = rudder_controller_data.i / (1.0f + rudder_controller_data.ci * error * error);
        integral_part = i_gain * rudder_controller_data.sum_error_pi;

        //proportional action
        float p_gain = rudder_controller_data.p / (1.0f + rudder_controller_data.cp * abs_error);
        proportional_part = p_gain * error;
    }
    else{
        //use normal digital PI, with anti wind-up constant

        //compute input for anti wind-up component
        float input_kaw = rudder_saturation(rudder_controller_data.last_command) - rudder_controller_data.last_command;

        //update sum error using anti wind-up component
        rudder_controller_data.sum_error_pi += (error + rudder_controller_data.kaw * input_kaw);

        integral_part = rudder_controller_data.i * rudder_controller_data.sum_error_pi;

        proportional_part = rudder_controller_data.p * error;
    }

    //command = P * error + I * sum{error}
    action = proportional_part + integral_part;

    //update last_command
    rudder_controller_data.last_command = action;

    return action;
}

/**
 * Set rudder controller data.
 *
 * @param p                         proportional gain
 * @param i                         integral gain
 * @param c                         used in conditional integral
 * @param rudder_controller_type    0 = standard PI, 1 = conditional PI, 2 = LQR
 * @param kaw                       constant used for anti-wind up in normal PI
*/
void set_rudder_data(float p, float i, float cp,
                     float ci, int32_t rudder_controller_type, float kaw,
                     float alpha_rudder_x1_r, float alpha_rudder_x2_r, float rud_cmd_45_left){

    //convert rudder_controller_type from int32_t to uint8_t
    uint8_t rudder_type = (uint8_t)rudder_controller_type;

    //save PI data
    rudder_controller_data.p = p;
    rudder_controller_data.i = i;
    rudder_controller_data.kaw = kaw;
    rudder_controller_data.cp = cp;
    rudder_controller_data.ci = ci;

    //check if we have changed the type of rudder controller
    if(rudder_controller_data.rudder_controller_type != rudder_type){
        //save new type of controller
        rudder_controller_data.rudder_controller_type = rudder_type;

        //reset PI internal data
        rudder_controller_data.last_command = 0.0f;
        rudder_controller_data.sum_error_pi = 0.0f;

        //send message to QGC
        if(rudder_type == 0)
            sprintf(txt_msg, "Switched to normal PI with anti wind-up gain.");
        else if(rudder_type == 1)
            sprintf(txt_msg, "Switched to conditional PI.");
        else
            sprintf(txt_msg, "Switched to LQR controller.");

        send_log_info(txt_msg);
    }

    //make sure rud_cmd_45_left does not exceed rud limits
    rud_cmd_45_left = rudder_saturation(rud_cmd_45_left);
    /* Since rud_cmd_45 is the command to give to the rudder when we want to
     * tack from port haul to starboard haul, it has to be positive in
     * order to make the boat steer on the left.
    */
    rud_cmd_45_left = my_fabs(rud_cmd_45_left);

    rudder_controller_data.alpha_rudder_x1_r = alpha_rudder_x1_r;
    rudder_controller_data.alpha_rudder_x2_r = alpha_rudder_x2_r;
    rudder_controller_data.rud_cmd_45_left = rud_cmd_45_left;
}

/** Perform tack maneuver.
 *
 * Perform tack maneuver based on the type of tack set by QGroundControl.
 * There are two possibles type of tack, @see set_tack_data() .
 *
 * @param ref_act_p     pointer to reference action
 * @param strs_p        pointer to topics data
 * @param p_rudder_cmd  pointer where rudder command will be returned
 * @param p_sails_cmd   pointer where sails command will be returned
 * @param strs_p        pointer to structs_topics_s
*/
void tack_action(struct reference_actions_s *ref_act_p,
                 float *p_rudder_cmd, float *p_sails_cmd,
                 struct structs_topics_s *strs_p){

    //check if we've already started tacking, or if this is the first time
    if(tack_data.boat_is_tacking){
        //we have already started the tack maneuver, check if it's completed
        if(is_tack_completed()){

            //we have just completed the tack maneuver
            tack_completed(ref_act_p);
            /* return to guidance guidance_module function so the controller for keeping
             * the new haul can start working
            */
            return;
        }
    }
    else{
        //we must start tack maneuver now
        tack_data.boat_is_tacking = true;
        /*
         * Pay attention: when path_planning module set should_tack = true, it even
         * changed the alpha_star, so to see at which haul we were sailing at before tacking,
         * we must change the sign of alpha_star!
         * If we are starting tacking now, sailing_at_port_haul variable must be updated.
         * If we were sailing on port (starboard) haul, alpha_star, before path planning
         * changed it, was < (>) 0.
         */
         tack_data.sailing_at_port_haul = (-ref_act_p->alpha_star < 0) ? true : false;
    }

    /*we are here beacuse ref_act_p->should_tack is true, so boat should tack.
     * Check which type of tack maneuver we should perform by checking
     * tack_data.tack_type
    */
    if(tack_data.tack_type == 0 || tack_data.tack_type == 1){
        //perform tack maneuver as helmsman would do
        helmsman_tack(ref_act_p, p_rudder_cmd, p_sails_cmd);
    }
    else if(tack_data.tack_type == 2){
        /*perform tack maneuver by simply returning the control of rudder
         * and sails to normal controller.
         *
         * This action of returning the control should bring the boat to tack.
         * Since when path_planning set should_tack = true, it even changed alpha_star
         * (the reference of rudder PI controller). Since alpha star has been changed,
         * the rudder control should move the rudder in order to follow the new
         * alpha star, and this should bring the boat to tack.
         *
         * Calling tack_completed() we say (even if for now it's false) that we have
         * completed the tack maneuver and we want the rudder controller and the
         * sails controller to take care of tracking the new alpha_star set by path_planning.
        */

        tack_completed(ref_act_p);
    }
    else if(tack_data.tack_type == 3){
        //LQR controller
        lqr_control_rudder(p_rudder_cmd, ref_act_p, strs_p);
        //use standard sail controller
        *p_sails_cmd = sail_controller(get_alpha_dumas());
    }
    else{
        //MPC controller
        mpc_control_rudder(p_rudder_cmd, ref_act_p, strs_p);
        //use standard sail controller
        *p_sails_cmd = sail_controller(get_alpha_dumas());
    }

}

/**
 * Perform tack maneuver as real helmsman would do, type of tack taken into account
 * are 0 and 1.
 *
 * Helmsman tack maneuver uses the alpha angle value as input in the rule
 * based control system.
*/
void helmsman_tack(struct reference_actions_s *ref_act_p,
                   float *p_rudder_cmd, float *p_sails_cmd){


    /* Helmsman tack maneuver uses the alpha angle value as input in the rule
     * based control system.
    */
    float alpha = get_alpha_dumas();

    // compute rudder and sails commad in any case. If the tack is completed, the guidance module

    /* use variable sailing_at_port_haul to see which tack maneuver has to be used.
     * Use tack_data.tack_type to select between helmsman0 and helmsman1 maneuver
    */
    if(tack_data.sailing_at_port_haul){
        if(tack_data.tack_type == 0)
            helmsman0_tack_p2s(alpha, p_rudder_cmd, p_sails_cmd);
        else
            helmsman1_tack_p2s(alpha, p_rudder_cmd, p_sails_cmd);
    }
    else{
        if(tack_data.tack_type == 0)
            helmsman0_tack_s2p(alpha, p_rudder_cmd, p_sails_cmd);
        else
            helmsman1_tack_s2p(alpha, p_rudder_cmd, p_sails_cmd);
    }
}

/**
 * Common action to perform when tack maneuver is completed.
*/
void tack_completed(struct reference_actions_s *ref_act_p){
    //we have just completed the tack maneuver

    /* Set should_tack to false so normal controllers
     * can compute new values for rudder and sails.
    */
    ref_act_p->should_tack = false;
    tack_data.boat_is_tacking = false;

    //notify to path_planning that we've completed the tack action
    notify_tack_completed();

    //notify to QGroundControl that we've completed the tack action
    sprintf(txt_msg, "Tack completed.");
    send_log_info(txt_msg);
}

/**
 * Determine when a tack maneuver is completed
 *
 * If the boat was sailing on port haul before tacking, the tack maneuver is considered
 * completed if alpha is greater or equals to alpha_min_stop_tack_r, @see set_tack_data() .
 * If the boat was sailing on starboard haul before tacking, the tack maneuver is considered
 * cmpleted if the alpha is less or equals to -alpha_min_stop_tack_r, @see set_tack_data() .
 *
 */
bool is_tack_completed(void){

    float alpha = get_alpha_dumas();

    bool completed = false;

    //check if we were sailing on port haul before starting the tack maneuver
    if(tack_data.sailing_at_port_haul){
        //we were sailing on port haul before tacking
        if(alpha >= tack_data.alpha_min_stop_tack_r)
            completed = true;
    }
    else{
        //we were sailing on starboard haul before tacking
        if(alpha <= -tack_data.alpha_min_stop_tack_r)
            completed = true;
    }
    return completed;
}

/**
 * Compute rudder and sails command to perform a tack maneuver that changes port haul
 * to starboard haul, use this function if tack_data.tack_type is 0.
 *
 * The output values are computed using a rule based system implemented as a real
 * helmsman would do the tack maneuver.
 *
 * @param alpha     angle with respect to the wind. Should be computed using yaw angle
 * @param p_rud     pointer where rudder cmd will be returned
 * @param p_sails   pointer where sails commad will be returned
*/
void helmsman0_tack_p2s(float alpha, float *p_rud, float *p_sails){

    float rud_45_left;
    float rud_x1_alpha;
    float rud_x2_alpha;

    //take the value of the rudder command to set it at 45deg and make the boat steer on the left
    rud_45_left = rudder_controller_data.rud_cmd_45_left;

    //take x1 and x2 value
    rud_x1_alpha = rudder_controller_data.alpha_rudder_x1_r;
    rud_x2_alpha = rudder_controller_data.alpha_rudder_x2_r;

    //rule based controller for the rudder
    if(alpha <= -0.523598f)//alpha <= -30deg
        *p_rud = rud_45_left;

    else if(alpha <= rud_x1_alpha)//-30deg < alpha <= rud_x1_alpha
        *p_rud =  (-rud_45_left /  (rud_x1_alpha + 0.523598f))
                  * (alpha - rud_x1_alpha);//linear slope from -rud_45_left to x1 as rud cmd

    else if(alpha <= rud_x2_alpha)//rud_x1_alpha < alpha <= rud_x2_alpha
        *p_rud =  (rud_45_left /  (rud_x2_alpha - rud_x1_alpha))
                  * (alpha - rud_x1_alpha);

    else if(alpha <= 0.69813f)//rud_x2_alpha < alpha <= 40deg
        *p_rud = (-rud_45_left / (0.69813f - rud_x2_alpha)) * (alpha - 0.69813f);

    else//alpha > 40deg
        *p_rud = 0.0f;

    /* use the same sail controller as in the normal sailig, byt now use the same alpha as
     * the one passed to helmsman_tack_p2s. (It should be alpha_yaw, that is, the alpha
     * updated using yaw angle and not cog angle. This alpha_yaw angle has a much faster
     * dynamic than the alpha computed with cog.
    */
    *p_sails = sail_controller(alpha);
}

/**
 * Compute rudder and sails command to perform a tack maneuver that changes starboard haul
 * to port haul, use this function if tack_data.tack_type is 0.
 *
 * The output values are computed using a rule based system implemented as a real
 * helmsman would do the tack maneuver.
*/
void helmsman0_tack_s2p(float alpha, float *p_rud, float *p_sails){

    float alpha_port;
    float rudder_port;

    //use simmetry in helmsman_tack_p2s and take care of changing only the sign of alpha and rudder
    alpha_port = -alpha;

    helmsman0_tack_p2s(alpha_port, &rudder_port, p_sails);

    /* convert rudder command computed to tack from port to starboard, into rudder command
     * to tack from starboard to port */
    *p_rud = -rudder_port;
}


/**
 * Compute rudder and sails command to perform a tack maneuver that changes port haul
 * to starboard haul, use this function if tack_data.tack_type is 1.
 *
 * The output values are computed using a rule based system implemented as a real
 * helmsman would do the tack maneuver, this is slightly different from @see helmsman0_tack_p2s.
 *
 * @param alpha     angle with respect to the wind. Should be computed using yaw angle
 * @param p_rud     pointer where rudder cmd will be returned
 * @param p_sails   pointer where sails commad will be returned
*/
void helmsman1_tack_p2s(float alpha, float *p_rud, float *p_sails){

    float rud_45_left;
    float rud_x1_alpha;
    float rud_x2_alpha;

    //take the value of the rudder command to set it at 45deg and make the boat steer on the left
    rud_45_left = rudder_controller_data.rud_cmd_45_left;

    //take x1 and x2 value
    rud_x1_alpha = rudder_controller_data.alpha_rudder_x1_r;
    rud_x2_alpha = rudder_controller_data.alpha_rudder_x2_r;

    //rule based controller for the rudder
    if(alpha <= rud_x1_alpha)//alpha <= rud_x1_alpha
        *p_rud = rud_45_left;
    else if(alpha <= rud_x2_alpha) //use linear slope until x2
        *p_rud = (- rud_45_left / (rud_x2_alpha - rud_x1_alpha))
                 * (alpha - rud_x2_alpha);
    else //alpha > rud_x2_alpha
        *p_rud = 0.0f;

    /* use the same sail controller as in the normal sailig, byt now use the same alpha as
     * the one passed to helmsman_tack_p2s. (It should be alpha_yaw, that is, the alpha
     * updated using yaw angle and not cog angle. This alpha_yaw angle has a much faster
     * dynamic than the alpha computed with cog.
    */
    *p_sails = sail_controller(alpha);
}

/**
 * Compute rudder and sails command to perform a tack maneuver that changes starboard haul
 * to port haul, use this function if tack_data.tack_type is 1.
 *
 * The output values are computed using a rule based system implemented as a real
 * helmsman would do the tack maneuver.
*/
void helmsman1_tack_s2p(float alpha, float *p_rud, float *p_sails){

    float alpha_port;
    float rudder_port;

    //use simmetry in helmsman_tack_p2s and take care of changing only the sign of alpha and rudder
    alpha_port = -alpha;

    helmsman1_tack_p2s(alpha_port, &rudder_port, p_sails);

    /* convert rudder command computed to tack from port to starboard, into rudder command
     * to tack from starboard to port */
    *p_rud = -rudder_port;
}

/**
 * Compute optimal rudder command based on the LQR controller.
 *
 * Based on the extended state model (@see compute_state_extended_model), the rudder command
 * at step k is given by rudder_k = rudder_{k-1} + K_LQR * state_extended_model.
 * Compute a new command only if the time elapsed since time_last_lqr is greater or equal to LQR_MODEL_TS.
*/
void lqr_control_rudder(float *p_rudder_cmd,
                        const struct reference_actions_s *ref_act_p,
                        struct structs_topics_s *strs_p){
    float u_k; //optimal input of the extended state model
    uint64_t now_us = hrt_absolute_time(); //absolute time in micro seconds

    //compute a new command only if the time elapsed since time_last_lqr is greater or equal to LQR_MODEL_TS.
    if((now_us - optimal_control_data.time_last_lqr) >= LQR_MODEL_TS){

        //compute the new state of the extended model based on the latest measurements
        compute_state_extended_model(ref_act_p);

        /*apply LQR gain matrix to the actual state of the extended model,
         * this will give you the u_k of the extended state model
        */
        u_k = 0.0f;
        //u_k = K_LQR * extendedState
        for(uint8_t i = 0; i < 3; i++){
            u_k = u_k +
                  optimal_control_data.lqr_gain[i] * optimal_control_data.state_extended_model[i];
        }

        //compute rudder command at step k to give to the real system: u_k + rudder_{k-1}
        *p_rudder_cmd = u_k + optimal_control_data.state_extended_model[2];

        //update time_last_lqr
        optimal_control_data.time_last_lqr = now_us;

        //save optimal control data
        strs_p->boat_opt_control.timestamp = hrt_absolute_time();
        strs_p->boat_opt_control.x1 = optimal_control_data.state_extended_model[0];
        strs_p->boat_opt_control.x2 = optimal_control_data.state_extended_model[1];
        strs_p->boat_opt_control.x3 = optimal_control_data.state_extended_model[2];
        strs_p->boat_opt_control.opt_rud = *p_rudder_cmd; // optimal rudder command computed
        strs_p->boat_opt_control.type_controller = 0;     // I am the LQR control, set 0 on type

        //signal that boat_opt_control has been updated
        strs_p->boat_opt_control_updated = true;
    }
    else{
        //provide last LQR command computed
        *p_rudder_cmd = optimal_control_data.state_extended_model[2];
    }
}

/**
 * Set MPC cost matrix, lower bound and upper bound.
 *
 * The MPC cost function is: sum_{i=1}^{prediction_horizon}(z_i^T * h * z_i) +
 * + z_{prediction_horizon+1}^T * h_final * z_{prediction_horizon+1}.
 * Where z_i = [u_i; xExtended_{i+1}]; where xExtended_i is the extended
 * state of the linear model estimated offline.
 * @see compute_state_extended_model.
 *
 * @param h        diagonal element of H matrix
 * @param lb       lower bounds
 * @param ub       upper bounds
 * @param h_final  final cost, only for the extended state
*/
void set_mpc_data(float h[4], float lb[2], float ub[2], float h_final[3][3]){

    //set Hessina matrix
    for(uint8_t i = 0; i < 4; i++)
        optimal_control_data.mpc_boatTack_params_s.Hessians[i] = h[i];

    //set lower bound values
    for(uint8_t i = 0; i < 2; i++)
        optimal_control_data.mpc_boatTack_params_s.lowerBound[i] = lb[i];

    //set upper bound values
    for(uint8_t i = 0; i < 2; i++)
        optimal_control_data.mpc_boatTack_params_s.upperBound[i] = ub[i];

    /* The real matrix HessinaFinal is about the input and the state of the system,
     * since we want a final cost only for the state at the end of the horizon, we
     * set the first row and coulumn of HessinaFinal to 0.
     * HessinaFinal is matrix of size [4 x 4] (column major format)
    */
    uint8_t index_hf;

    //first go by the whole row i, and then increment column j
    for(uint8_t j = 0; j < 4; j++){
        for(uint8_t i = 0; i < 4; i++){
            /*remember that HessianFinal is 4x4 (column major format), so from i and j we must
             * have a new index to access into the vector HessiansFinal
            */
            index_hf = j * 4 + i;

            if(i == 0 || j == 0){
                //first row or first coulumn of HessiansFinal, set it to 0
                optimal_control_data.mpc_boatTack_params_s.HessiansFinal[index_hf] = 0.0f;
            }
            else{
                //remember that HessianFinal is 4x4 (column major format) but h_final is 3x3
                optimal_control_data.mpc_boatTack_params_s.HessiansFinal[index_hf] = h_final[i-1][j-1];
            }
        }
    }
}

/**
 * Compute optimal rudder command based on the MPC controller.
 *
 * Based on the extended state model (@see compute_state_extended_model), the rudder command
 * at step k is given by rudder_k = rudder_{k-1} + uStar, where uStar is the output
 * of the MPC controller.
 * Compute a new command only if the time elapsed since time_last_lqr is greater or equal to MPC_MODEL_TS.
*/
void mpc_control_rudder(float *p_rudder_cmd,
                        const struct reference_actions_s *ref_act_p,
                        struct structs_topics_s *strs_p){

    int solver_ret;
    uint64_t now_us = hrt_absolute_time(); //absolute time in micro seconds

    //compute a new command only if the time elapsed since time_last_lqr is greater or equal to LQR_MODEL_TS.
    if((now_us - optimal_control_data.time_last_mpc) >= MPC_MODEL_TS){

        //compute the new state of the extended model based on the latest measurements
        compute_state_extended_model(ref_act_p);

        //TODO initialize optimal_control_data.mpc_boatTack_params_s!!!

        //call the solver
        solver_ret = mpc_boatTack_solve(&(optimal_control_data.mpc_boatTack_params_s),
                                        &(optimal_control_data.mpc_boatTack_output_s),
                                        &(optimal_control_data.mpc_boatTack_info_s));

        //TODO CHECK solver_ret before using result as rudder command!!!
        if(solver_ret == 1){//CHE VALORE CI METTO?
            //compute rudder command at step k to give to the real system: optimalU + rudder_{k-1}
            *p_rudder_cmd = optimal_control_data.mpc_boatTack_output_s.u0[0] +
                            optimal_control_data.state_extended_model[2];
        }
        else{
            //something went wrong in the solver! Give the last command that has been given
            *p_rudder_cmd = optimal_control_data.state_extended_model[2];
        }

        //update time_last_lqr
        optimal_control_data.time_last_mpc = now_us;

        //save optimal control data
        strs_p->boat_opt_control.timestamp = hrt_absolute_time();
        strs_p->boat_opt_control.x1 = optimal_control_data.state_extended_model[0];
        strs_p->boat_opt_control.x2 = optimal_control_data.state_extended_model[1];
        strs_p->boat_opt_control.x3 = optimal_control_data.state_extended_model[2];
        strs_p->boat_opt_control.opt_rud = *p_rudder_cmd;
        strs_p->boat_opt_control.type_controller = 1; //I am the MPC controller
        strs_p->boat_opt_control.it = optimal_control_data.mpc_boatTack_info_s.it;
        strs_p->boat_opt_control.solvetime = optimal_control_data.mpc_boatTack_info_s.solvetime;
        strs_p->boat_opt_control.res_eq = optimal_control_data.mpc_boatTack_info_s.res_eq;
        strs_p->boat_opt_control.pobj = optimal_control_data.mpc_boatTack_info_s.pobj;
        strs_p->boat_opt_control.dobj = optimal_control_data.mpc_boatTack_info_s.dobj;
        strs_p->boat_opt_control.dgap = optimal_control_data.mpc_boatTack_info_s.dgap;
        strs_p->boat_opt_control.rdgap = optimal_control_data.mpc_boatTack_info_s.rdgap;

        //signal that boat_opt_control has been updated
        strs_p->boat_opt_control_updated = true;
    }
    else{
        //provide last LQR command computed
        *p_rudder_cmd = optimal_control_data.state_extended_model[2];
    }
}

/**
 * Compute the actual state of the extended model used by both LQR and MPC.
 *
 * The state of the extended model is: x_k = [yawRate_k; yaw_k; rudder_{k-1}],
 * where yawRate and yaw are expressed in our sensor frame.
 *
 * The input of the extended model is u_k = rudder_k - rudder_{k-1}.
 *
 * The updated state value is stored in optimal_control_data.state_extended_model.
*/
void compute_state_extended_model(const struct reference_actions_s *ref_act_p){

    //yaw rate from measurements
    optimal_control_data.state_extended_model[0] = get_yaw_rate_sns();

    /* difference from the actual yaw and the desired yaw, assuming true wind
     * will be constant and there will be no drift (in general these
     * two assumptions are not true, but it is the best we can do).
    */
    optimal_control_data.state_extended_model[1] = ref_act_p->alpha_star - get_alpha_dumas();

    //latest command given to the rudder
    optimal_control_data.state_extended_model[2] = optimal_control_data.rudder_latest;
}

/**
 * Simple rule based control system for the sails.
 * Look at the alpha angle to set how much the sails should be opened or closed.
 *
 * The output should be used to control the sails both while the boat is sailing
 * upwind and when it is tacking.
 *
 * @param alpha angle with respect to the wind
*/
float sail_controller(float alpha){

    float sail = 0.0f;

    //rule based controller for the sails
    if(alpha <= -sail_controller_data.alpha_sail_opened_r)
        sail = SAIL_FULLY_OPENED;

    else if(alpha <= - sail_controller_data.alpha_sail_closed_r)
        sail = sail_controller_data.positive_slope *
                (alpha + sail_controller_data.alpha_sail_opened_r);

    else if(alpha <= sail_controller_data.alpha_sail_closed_r)
        sail = sail_controller_data.sail_closed_cmd;

    else if(alpha <= sail_controller_data.alpha_sail_opened_r)
        sail = -sail_controller_data.positive_slope *
                (alpha - sail_controller_data.alpha_sail_opened_r);

    else //alpha > sail_controller_data.alpha_sail_opened_r
        sail = SAIL_FULLY_OPENED;

    return sail;
}


/**
 * Saturation of rudder command, according to rudder servo motor limits
*/
float rudder_saturation(float command){

    if(command > RUDDER_SATURATION)
        command = RUDDER_SATURATION;
    else if(command < -RUDDER_SATURATION)
        command = -RUDDER_SATURATION;

    return command;
}

/**
 * Saturation of sails command, according to sail servo motor limits
*/
float sail_saturation(float command){

    if(command > SAIL_SATURATION)
        command = SAIL_SATURATION;
    else if(command < 0.0f)
        command = 0.0f;

    return command;
}

/** Implement reference actions provided by optimal path planning*/
void guidance_module(struct reference_actions_s *ref_act_p,
                     const struct parameters_qgc *param_qgc_p,
                     struct structs_topics_s *strs_p){

    float alpha;//angle with respect to the wind
    float rudder_command = 0.0f;
    float sail_command = 0.0f;

    //get alpha from the moving average value of the last k values of instant alpha
    alpha = get_alpha_dumas();

    //check if path_planning() told us to tack
    if(ref_act_p->should_tack){
        //perform tack maneuver
        tack_action(ref_act_p, &rudder_command, &sail_command, strs_p);
    }
    if(!(ref_act_p->should_tack)){
        //if the boat should not tack, compute rudder and sails actions to follow reference alpha
        if(rudder_controller_data.rudder_controller_type != 2){
            //PI controller for rudder
            rudder_command = pi_controller(&(ref_act_p->alpha_star), &alpha);
        }
        else{
            //LQR controller
            lqr_control_rudder(&rudder_command, ref_act_p, strs_p);
        }

        //sails control only if AS_SAIL param from QGC is negative
        if(param_qgc_p->sail_servo < 0.0f)
            sail_command = sail_controller(alpha);
        else
            sail_command = param_qgc_p->sail_servo;
    }

    #if SIMULATION_FLAG == 1
    //debug
    //rudder_command = param_qgc_p->deva1;
    #endif


    //saturation for safety reason
    rudder_command = rudder_saturation(rudder_command);
    sail_command = sail_saturation(sail_command);

    //update actuator value
    strs_p->actuators.control[0] = rudder_command;
    strs_p->actuators.control[3] =  sail_command;

    //update rudder_latest in any case, even if we are not tacking
    optimal_control_data.rudder_latest = rudder_command;

    //save first debug values for post-processing, other values set in path_planning()
    strs_p->boat_guidance_debug.timestamp = hrt_absolute_time();
    strs_p->boat_guidance_debug.alpha = alpha;
    strs_p->boat_guidance_debug.rudder_action = rudder_command;
    strs_p->boat_guidance_debug.sail_action = sail_command;
    strs_p->boat_guidance_debug.twd_mean = get_twd_sns();
    strs_p->boat_guidance_debug.app_mean = get_app_wind_sns();

    #if SIMULATION_FLAG == 1
    strs_p->airspeed.true_airspeed_m_s = ref_act_p->alpha_star - get_alpha_dumas();
    #endif
}

