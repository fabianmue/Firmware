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

//constant for tack_type
#define TACK_IMPLICIT     0
#define TACK_LQR    1
#define TACK_MPC    2
#define TACK_P      3

//const for type of rudder controller during normal sailing
#define SAILING_RUD_STD_PI 0 ///standard PI with anti-wind up action
#define SAILING_RUD_COND_PI 1 ///conditional PI

// errors legend in is_tack_completed()
#define EVERYTHING_OK 0
#define FORCED_TACK_STOP 1 //tack had to be stopped for safety reason
#define NO_MPC_SOLVE_FNC 2 //no available solve function for the set prediction horizon

#define M_PI_F 3.14159265358979323846f
#define TWO_M_PI_F 6.28318530717959f

static char txt_msg[150]; ///used to send messages to QGC
static const float deg2rad = 0.0174532925199433f; // pi / 180

/** @brief PI controller with conditional integration*/
float pi_controller(const float *ref_p, const float *meas_p);

/** @brief if the boat should tack, perform tack maneuver*/
void tack_action(struct reference_actions_s *ref_act_p,
                 float *p_rudder_cmd, float *p_sails_cmd,
                 struct structs_topics_s *strs_p);

/** @brief determine if the tack maneuver is completed*/
bool is_tack_completed(struct reference_actions_s *ref_act_p, int8_t *error_p);

/** @brief rule based control system for sails during upwind sailing*/
float sail_controller(float alpha);

/** @brief saturation on command to the rudder servo motor*/
float rudder_saturation(float command);

/** @brief saturation on command to the sails servo motor*/
float sail_saturation(float command);

/** @brief action to perform when tack maneuver is completed*/
void tack_completed(struct reference_actions_s *ref_act_p, int8_t error_code);

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

/** @brief compute -A * x_0, used by Forces as initil parameter*/
void compute_minusAExt_times_x0(float* minusAExt_times_x0);

/** @brief compute rudder action for P tack*/
float p_tack_rudder(const struct reference_actions_s *ref_act_p);

/// Forces parameters
struct forces_params_s{
    /* vector of size 3 */
    float minusAExt_times_x0[3];

    /* diagonal matrix of size [4 x 4] (only the diagonal is stored) */
    float Hessians[4];

    /* matrix of size [4 x 4] (column major format) */
    float HessiansFinal[16];

    /* vector of size 2 */
    float lowerBound[2];

    /* vector of size 2 */
    float upperBound[2];

    /* matrix of size [3 x 4] (column major format) */
    float C[12];

    /* matrix of size [3 x 4] (column major format) */
    float D[12];
};

///Forces output
struct forces_output_s{
    /* vector of size 1 */
    float u0[1];
};

///Forces info
struct forces_info_s{
    /* iteration number */
    int it;

    /* inf-norm of equality constraint residuals */
    float res_eq;

    /* inf-norm of inequality constraint residuals */
    float res_ineq;

    /* primal objective */
    float pobj;

    /* dual objective */
    float dobj;

    /* duality gap := pobj - dobj */
    float dgap;

    /* relative duality gap := |dgap / pobj | */
    float rdgap;

    /* duality measure */
    float mu;

    /* duality measure (after affine step) */
    float mu_aff;

    /* centering parameter */
    float sigma;

    /* number of backtracking line search steps (affine direction) */
    int lsit_aff;

    /* number of backtracking line search steps (combined direction) */
    int lsit_cc;

    /* step size (affine direction) */
    float step_aff;

    /* step size (combined direction) */
    float step_cc;

    /* solvertime */
    float solvetime;
};

//static data for tack action
static struct{
    bool boat_is_tacking;       ///true if boat is performing a tack maneuver
    uint16_t tack_type;         ///type of tack action to perform
}tack_data =    {
                    .boat_is_tacking = false,//true if boat is performing a tack maneuver
                    .tack_type = 0,
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
    uint8_t rudder_controller_type; /// 0 = standard PI 1 = conditional PI
    float last_command;     ///last command provided by the PI
    float sum_error_pi;     ///error sum from iterations of guidance_module, used in PI
    float abs_rud_saturation;  ///rudder saturation set by QGC
    float p_tack_kp;           ///kp for P controller during P_TACK maneuver
    float p_tack_cp;           ///cp for P controller during P_TACK maneuver
} rudder_controller_data =  {
                                .p = 0.35f,
                                .i = 0.0f,
                                .kaw = 0.5f,
                                .cp = 0.35f,
                                .ci = 0.0f,
                                .rudder_controller_type = 0,//start with conditional PI
                                .last_command = 0.0f,
                                .sum_error_pi = 0.0f,
                                .abs_rud_saturation = MAX_RUDDER_SATURATION,
                                .p_tack_kp = 0.73661977f,
                                .p_tack_cp = 0.1f
                            };

//static data for LQR and MPC controllers
static struct {
    float state_extended_model[3];///state of the model used by LQR and MPC
    float rudder_latest; ///latest rudder command gave by either LQR or MPC
    //static data for MPC
    uint64_t time_last_mpc; ///time when last MPC was computed
    uint64_t mpc_sampling_time_us;
    struct forces_params_s forces_params; ///data to full fill before calling forces solver
    struct forces_output_s forces_output; ///output from forces
    struct forces_info_s forces_info; ///info after calling forces
    int32_t pred_horz_steps;///number of steps for the prediction horizon
    //static data for LQR
    float lqr_gain[3]; ///lqr gain vector
    uint64_t lqr_sampling_time_us;///sampling type of the LQR, in uSec
    uint64_t time_last_lqr; ///time when last LQR was computed
    float delta_values[2]; ///delta values to specify the tube around the origin
    uint64_t min_time_in_band_us; ///min microseconds the system should be in the band to complete tacking
    uint64_t last_time_in_band_us;///last time (in this tack) the systm was detected into the band
    uint64_t time_started_tack_us;///first time (in this tack) the systm was detected into the band
    bool time_started_valid; ///true if time_started_tack_us is valid
    bool last_time_valid; ///true if last_time_in_band_us is valid
    uint64_t safety_time_stop_tack; ///Max time for completing a tack
} opc_data =    {
                                .rudder_latest = 0.0f,
                                .time_last_mpc = (uint64_t) 0,
                                .time_last_lqr = (uint64_t) 0,
                                .lqr_sampling_time_us = (uint64_t) 100000,//0.1 sec
                                .mpc_sampling_time_us = (uint64_t) 100000,//0.1 sec
                                .min_time_in_band_us = (uint64_t) 500000,//0.5 sec
                                .time_started_valid = false,//time_started_tack_us not valid now
                                .last_time_valid = false, //last_time_in_band_us not valid now
                                .safety_time_stop_tack = MIN_SAFETY_TIME_STOP_TCK,
                                .pred_horz_steps = 20
                            };

/** @brief dummy abs function for float value*/
float my_fabs(float x){
    x = (x > 0.0f) ? x : -x;
    return x;
}

/**
 * Set LQR gain.
 *
*/
void gm_set_lqr_gain(float lqr_k1, float lqr_k2, float lqr_k3, int32_t lqr_samp_time_us){
    opc_data.lqr_gain[0] = lqr_k1;
    opc_data.lqr_gain[1] = lqr_k2;
    opc_data.lqr_gain[2] = lqr_k3;
    opc_data.lqr_sampling_time_us = (uint64_t) lqr_samp_time_us;
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
void gm_set_sail_data(float sail_closed_cmd, float alpha_sail_closed_r, float alpha_sail_opened_r){

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
 * Tack maneuver can be performed in four different ways.
 * The first one (type is equal to 0) is performed by changing only the reference
 * angle with respect to the wind (alpha) and then "wait" for the PI controller
 * of the rudder to follow this changing, this is called implicit tack.
 *
 * The second one (type equal to 1) is performed by a LQR controller.
 *
 * The third one (type equal to 2) is performed by a MPC controller.
 *
 * The fourth one (type equal to 3) is performed by a P controller.
 *
 * @param tack_type               type of tack to perform
*/
void gm_set_tack_data(uint16_t tack_type){
    uint16_t old_tack_type = tack_data.tack_type;

    //notify the changing to QGroundControl what kind of tack the boat will do
    if(old_tack_type != tack_type){
        if(tack_type == TACK_IMPLICIT){
            smq_send_log_info("Implicit (PI) Tack.");
            //save new value
            tack_data.tack_type = tack_type;
        }
        else if(tack_type == TACK_LQR){
            smq_send_log_info("LQR Tack.");
            //save new value
            tack_data.tack_type = tack_type;
        }
        else if(tack_type == TACK_MPC){
            smq_send_log_info("MPC Tack.");
            //save new value
            tack_data.tack_type = tack_type;
        }
        else if(tack_type == TACK_P){
            smq_send_log_info("P Tack.");
            //save new value
            tack_data.tack_type = tack_type;
        }
        else{
            //error: no valid type of tack selected
            smq_send_log_critical("Error: no valid type of tack selected.");
            //use the last valid type of tack set
        }
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

    if(rudder_controller_data.rudder_controller_type == SAILING_RUD_COND_PI){
        //Conditional PI
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
 * @param rudder_controller_type    0 = standard PI, 1 = conditional PI
 * @param kaw                       constant used for anti-wind up in normal PI
 * @param abs_rudder_saturation     max rudder command, must be <= 1.0
*/
void gm_set_rudder_data(float p, float i, float cp,
                     float ci, int32_t rudder_controller_type, float kaw,
                     float abs_rudder_saturation){

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

        //reset PI internal data
        rudder_controller_data.last_command = 0.0f;
        rudder_controller_data.sum_error_pi = 0.0f;

        //send message to QGC
        if(rudder_type == SAILING_RUD_STD_PI){
            sprintf(txt_msg, "Switched to normal PI with anti wind-up gain.");
            smq_send_log_info(txt_msg);
            //save new type of controller
            rudder_controller_data.rudder_controller_type = rudder_type;
        }
        else if(rudder_type == SAILING_RUD_COND_PI){
            sprintf(txt_msg, "Switched to conditional PI.");
            smq_send_log_info(txt_msg);
            //save new type of controller
            rudder_controller_data.rudder_controller_type = rudder_type;
        }
        else{
            sprintf(txt_msg, "Error: select a rudder controller: 0 or 1.");
            smq_send_log_critical(txt_msg);
            //use the old PI
        }
    }

    //make sure abs_rudder_saturation is >= 0 and is < MAX_RUDDER_SATURATION
    //make sure rud_cmd_45_left does not exceed rud limits
    abs_rudder_saturation = my_fabs(abs_rudder_saturation);
    if(abs_rudder_saturation > MAX_RUDDER_SATURATION){
        abs_rudder_saturation = MAX_RUDDER_SATURATION;
        smq_send_log_info("AS_MAX_RUD must be <= 1.");
    }

    rudder_controller_data.abs_rud_saturation = abs_rudder_saturation;
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
        int8_t error_code;
        if(is_tack_completed(ref_act_p, &error_code)){

            //we have just completed the tack maneuver
            tack_completed(ref_act_p, error_code);
            /* return to guidance guidance_module function so the controller for keeping
             * the new haul can start working
            */
            return;
        }
    }
    else{
        //we must start the tack maneuver now
        tack_data.boat_is_tacking = true;

         /*if we are using either LQR or MPC or P tack, save the time when we start tacking and
          * tell controller_data module that a tack has just been started
          */
         if(tack_data.tack_type == TACK_LQR || tack_data.tack_type == TACK_MPC
            || tack_data.tack_type == TACK_P){

             opc_data.time_started_tack_us = hrt_absolute_time();
             opc_data.time_started_valid = true;

             cd_optimal_tack_started();
         }
    }

    /*we are here beacuse ref_act_p->should_tack is true, so boat should tack.
     * Check which type of tack maneuver we should perform by checking
     * tack_data.tack_type
    */
    if(tack_data.tack_type == TACK_IMPLICIT){
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
        //no error can be happened, use erro_code = EVERYTHING_OK
        tack_completed(ref_act_p, EVERYTHING_OK);
    }
    else if(tack_data.tack_type == TACK_LQR){
        //LQR controller
        lqr_control_rudder(p_rudder_cmd, ref_act_p, strs_p);
        //use standard sail controller
        *p_sails_cmd = sail_controller(cd_get_alpha_dumas());
    }
    else if(tack_data.tack_type == TACK_MPC){
        //MPC controller
        mpc_control_rudder(p_rudder_cmd, ref_act_p, strs_p);
        //use standard sail controller
        *p_sails_cmd = sail_controller(cd_get_alpha_dumas());
    }
    else if(tack_data.tack_type == TACK_P){
        //P controller for p tack
        *p_rudder_cmd = p_tack_rudder(ref_act_p);
        //use standard sail controller
        *p_sails_cmd = sail_controller(cd_get_alpha_dumas());
    }
    else{
        //error, no valid type of tack! End tack for safety reason
        tack_completed(ref_act_p, FORCED_TACK_STOP);
    }

}

/**
 * Common action to perform when tack maneuver is completed.
 *
 * @param pointer to struct containing reference actions
 * @param error_code error code if something went wrong
*/
void tack_completed(struct reference_actions_s *ref_act_p, int8_t error_code){
    //we have just completed the tack maneuver

    /* Set should_tack to false so normal controllers
     * can compute new values for rudder and sails.
    */
    ref_act_p->should_tack = false;
    tack_data.boat_is_tacking = false;

    //notify to path_planning that we've completed the tack action
    pp_notify_tack_completed();

    //if we had used either LQR or MPC or P tack, notify tack completed to controller_data
    if(tack_data.tack_type == TACK_LQR || tack_data.tack_type == TACK_MPC
       || tack_data.tack_type == TACK_P)
        cd_optimal_tack_completed();

    //check error_code to give a feedback to QGroundControl
    if(error_code == EVERYTHING_OK){
        //notify to QGC that we've completed the tack action
        smq_send_log_info("Tack completed.");
    }
    else if(error_code == FORCED_TACK_STOP){
        //notify to QGC that we had to force stopping the tack
        smq_send_log_critical("Tack forced to be completed.");
    }
    else if(error_code == NO_MPC_SOLVE_FNC){
        //notify to QGC the error
        smq_send_log_critical("No MPC solve for this horizon!");
    }
    else{
        smq_send_log_critical("Error: check error_code in guidance_module");
    }
}

/**
 * Determine when a tack maneuver is completed
 *
 * If we are using either LQR or MPC or P tack:
 * the tack maneuver is completed if and only if state[1] and state[2] of the extended system
 * , computed by @see compute_state_extended_model(), is in the range
 * [ -opc_data.delta_values[i], opc_data.delta_values[i] ]
 * for at least opc_data.min_time_in_band_us micro seconds.
 * For safety reason, the tack will be considered completed if the time elapsead since the
 * starting of the maneuver is greater or equal to SAFETY_TIME_STOP_TCK constant; even
 * if the system is not in the tube.
 *
 * @param error_p <-- 0 if everything is fine, error code otherwise
 *
 * @return true if the tack maneuver can be considered completed, false otherwise.
 */
bool is_tack_completed(struct reference_actions_s *ref_act_p, int8_t *error_p){

    bool completed = false;//initial guess
    *error_p = EVERYTHING_OK; // let's hope we will not have any error

    //implicit tack
    if(tack_data.tack_type == TACK_IMPLICIT){
        completed = false;
    }
    else if(tack_data.tack_type == TACK_LQR || tack_data.tack_type == TACK_MPC
            || tack_data.tack_type == TACK_P){

        //check if the extended state is in the tube near the origin
        compute_state_extended_model(ref_act_p);
        bool state_in_band = true;//initial guess

        for(uint8_t i = 0; i < 2; i++){
            //remember not to check if yaw rate is in the tube 'cause we don't have a delta for it
            if(my_fabs(opc_data.state_extended_model[i+1]) >
                       opc_data.delta_values[i])
                state_in_band = false;
        }

        if(state_in_band == true){
            //the extended state is in the band near the origin

            if(opc_data.last_time_valid == false){
                //This is the first time the state (re)entered in the band near the origin, save time
                opc_data.last_time_in_band_us = hrt_absolute_time();
                opc_data.last_time_valid = true;
            }
            else{
                //the system was already in the tube near the origin
                uint64_t now_us = hrt_absolute_time();
                //check if the state has been in the tube for at least min_time_in_band_us
                if((now_us - opc_data.last_time_in_band_us) >=
                    opc_data.min_time_in_band_us){

                    //the tack can be considered completed
                    completed = true;
                    //set last_time_valid and time_started_valid to false
                    opc_data.last_time_valid = false;
                    opc_data.time_started_valid = false;
                }
            }
        }
        else{
            //now we are outside the tube, check if we were inside it previously
            if(opc_data.last_time_valid == true){
                //we were in the tube, but now we aren't, remember this
                opc_data.last_time_valid = false;
            }
        }
        //perfrom anyway safety check to see if we have to force stopping tack
        if(opc_data.time_started_valid == true){
            //we have already started the tack in the past
            //use time_started_tack_us to check if we have to force stopping the tack for safety reason
            uint64_t now_us = hrt_absolute_time();
            if((now_us - opc_data.time_started_tack_us) >= opc_data.safety_time_stop_tack){
                //the tack MUST be forced to be finished
                completed = true;
                //signal this updating error_p
                *error_p = FORCED_TACK_STOP;
                //set last_time_valid and time_started_valid to false
                opc_data.last_time_valid = false;
                opc_data.time_started_valid = false;
            }
        }
    }

    return completed;
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

    //compute a new command only if the time elapsed since time_last_lqr is greater or equal to lqr_sampling_time_us.
    if((now_us - opc_data.time_last_lqr) >= opc_data.lqr_sampling_time_us){

        //compute the new state of the extended model based on the latest measurements
        compute_state_extended_model(ref_act_p);

        /*apply LQR gain matrix to the actual state of the extended model,
         * this will give you the u_k of the extended state model
        */
        u_k = 0.0f;
        // u_k = K_LQR * extendedState
        for(uint8_t i = 0; i < 3; i++){
            u_k = u_k +
                  opc_data.lqr_gain[i] * opc_data.state_extended_model[i];
        }
        /** Pay attention: the gain matrix for the LQR controller computed by Matlab requires
         * a negative feedback control law! So the right command is
         * u_k = -K_LQR * extendedState
        */
        u_k = -u_k;

        //compute rudder command at step k to give to the real system: u_k + rudder_{k-1}
        *p_rudder_cmd = u_k + opc_data.state_extended_model[2];

        //update time_last_lqr
        opc_data.time_last_lqr = now_us;

        //save the new status of the optimal control
        strs_p->boat_opt_status.timestamp = hrt_absolute_time();
        strs_p->boat_opt_status.x1 = opc_data.state_extended_model[0];
        strs_p->boat_opt_status.x2 = opc_data.state_extended_model[1];
        strs_p->boat_opt_status.x3 = opc_data.state_extended_model[2];
        strs_p->boat_opt_status.opt_rud = *p_rudder_cmd;
        strs_p->boat_opt_status.type_controller = 0;//I am the LQR controller
        //set to -1 the MPC status 'cause I am the LQR
        strs_p->boat_opt_status.it = -1;
        strs_p->boat_opt_status.solvetime = -1.0f;
        strs_p->boat_opt_status.res_eq = -1.0f;
        strs_p->boat_opt_status.pobj = -1.0f;
        strs_p->boat_opt_status.dobj = -1.0f;
        strs_p->boat_opt_status.dgap = -1.0f;
        strs_p->boat_opt_status.rdgap = -1.0f;
        //boat_opt_status just updated
        strs_p->boat_opt_status_updated = true;
    }
    else{
        /* the time elapsed since the last time the LQR was computed is less than
         * LQR_MODEL_TS. Use the last command provided to the rudder that has been
         * saved by the function guidance_module into opc_data.rudder_latest
        */
        *p_rudder_cmd = opc_data.rudder_latest;
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
 * @param mpc_sampling_time_us  sampling time of the MPC controller
 * @param A A matrix that will be extended to be used in the MPC
 * @param B B matrix that will be extended to be used in the MPC
 * @param pred_horz_steps number of steps for the prediction horizon
*/
void gm_set_mpc_data(float h[4], float lb[2], float ub[2], float h_final[3][3],
                  int32_t mpc_sampling_time_us, float A[2][2], float B[2], int32_t pred_horz_steps){

    //set Hessina matrix
    for(uint8_t i = 0; i < 4; i++)
        opc_data.forces_params.Hessians[i] = h[i];

    //set lower bound values
    for(uint8_t i = 0; i < 2; i++)
        opc_data.forces_params.lowerBound[i] = lb[i];

    //set upper bound values
    for(uint8_t i = 0; i < 2; i++)
        opc_data.forces_params.upperBound[i] = ub[i];

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
                opc_data.forces_params.HessiansFinal[index_hf] = 0.0f;
            }
            else{
                //remember that HessianFinal is 4x4 (column major format) but h_final is 3x3
                opc_data.forces_params.HessiansFinal[index_hf] = h_final[i-1][j-1];
            }
        }
    }

    //check if we have an available mpc solve for the new prediction horizon
    if(pred_horz_steps == 20 || pred_horz_steps == 30){
        //save prediction horizon
        opc_data.pred_horz_steps = pred_horz_steps;
    }
    else{
        //the horizon inserted is not available
        smq_send_log_critical("Horizon not available");
        //use the last one valid
    }

    //save models sampling time
    opc_data.mpc_sampling_time_us = (uint64_t)mpc_sampling_time_us;

    //A_ext = [A, B; 0, I], B_ext = [B; I]

    //put A_ext in the vector C of forces_params_s (column major format)
    opc_data.forces_params.C[0] = 0.0f;
    opc_data.forces_params.C[1] = 0.0f;
    opc_data.forces_params.C[2] = 0.0f;

    opc_data.forces_params.C[3] = A[0][0];
    opc_data.forces_params.C[4] = A[1][0];
    opc_data.forces_params.C[5] = 0.0f;

    opc_data.forces_params.C[6] = A[0][1];
    opc_data.forces_params.C[7] = A[1][1];
    opc_data.forces_params.C[8] = 0.0f;

    opc_data.forces_params.C[9] = B[0];
    opc_data.forces_params.C[10] = B[1];
    opc_data.forces_params.C[11] = 1.0f;

    //forces D must be = [B_ext, -eye(3)], (column major form)
    opc_data.forces_params.D[0] = B[0];
    opc_data.forces_params.D[1] = B[1];
    opc_data.forces_params.D[2] = 1.0f;

    opc_data.forces_params.D[3] = -1.0f;
    opc_data.forces_params.D[4] = 0.0f;
    opc_data.forces_params.D[5] = 0.0f;

    opc_data.forces_params.D[6] = 0.0f;
    opc_data.forces_params.D[7] = -1.0f;
    opc_data.forces_params.D[8] = 0.0f;

    opc_data.forces_params.D[9] = 0.0f;
    opc_data.forces_params.D[10] = 0.0f;
    opc_data.forces_params.D[11] = -1.0f;
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

    *p_rudder_cmd = 0.0f;

    int solver_ret = 0;
    uint64_t now_us = hrt_absolute_time(); //absolute time in micro seconds

    #if TEST_MPC == 1

    #endif

    //compute a new command only if the time elapsed since time_last_lqr is greater or equal to LQR_MODEL_TS.
    #if TEST_MPC == 0
    if((now_us - opc_data.time_last_mpc) >= opc_data.mpc_sampling_time_us){
    #else
    if((now_us - opc_data.time_last_mpc) >= opc_data.mpc_sampling_time_us && mt_still_data()){//cancellare
    #endif
        //compute the new state of the extended model based on the latest measurements
        compute_state_extended_model(ref_act_p);

        #if TEST_MPC == 1
        mt_get_next_yr_y(&opc_data.state_extended_model);
        opc_data.state_extended_model[2] = opc_data.rudder_latest;
        printf("\n ---------------------- \n");
        #endif

        //init parameters before calling the solver
        compute_minusAExt_times_x0(&(opc_data.forces_params.minusAExt_times_x0));

        #if TEST_MPC == 1
//        for(uint8_t i = 0; i < 3; i++){
//            printf("x0[%d]: %2.4f \t", i,
//                   (double)  opc_data.state_extended_model[i]);
//        }
        #endif

        //call the solver and take computation time
        uint64_t time_before_sol = hrt_absolute_time();

        //based on the prediction horizon value, see which solver we have to call

        /* WARNING: if you want to add/modify a mpc solve function, remember to update
         * the if clause in gm_set_mpc_data() where pred_horz_steps is checked
        */
//        if(opc_data.pred_horz_steps == 10){
//            solver_ret = mpc_boatTack_h10_solve((mpc_boatTack_h10_params*) &(opc_data.forces_params),
//                                            (mpc_boatTack_h10_output*) &(opc_data.forces_output),
//                                            (mpc_boatTack_h10_info*) &(opc_data.forces_info));
//        }
//        else if(opc_data.pred_horz_steps == 15){
//            solver_ret = mpc_boatTack_h15_solve((mpc_boatTack_h15_params*) &(opc_data.forces_params),
//                                            (mpc_boatTack_h15_output*) &(opc_data.forces_output),
//                                            (mpc_boatTack_h15_info*) &(opc_data.forces_info));
//        }
        if(opc_data.pred_horz_steps == 20){
            solver_ret = mpc_boatTack_h20_solve((mpc_boatTack_h20_params*) &(opc_data.forces_params),
                                            (mpc_boatTack_h20_output*) &(opc_data.forces_output),
                                            (mpc_boatTack_h20_info*) &(opc_data.forces_info));
        }
//        else if(opc_data.pred_horz_steps == 25){
//            solver_ret = mpc_boatTack_h25_solve((mpc_boatTack_h25_params*) &(opc_data.forces_params),
//                                            (mpc_boatTack_h25_output*) &(opc_data.forces_output),
//                                            (mpc_boatTack_h25_info*) &(opc_data.forces_info));
//        }
        else if(opc_data.pred_horz_steps == 30){
            solver_ret = mpc_boatTack_h30_solve((mpc_boatTack_h30_params*) &(opc_data.forces_params),
                                            (mpc_boatTack_h30_output*) &(opc_data.forces_output),
                                            (mpc_boatTack_h30_info*) &(opc_data.forces_info));
        }
        else{
            //error, no solve function available!
            tack_completed(ref_act_p, NO_MPC_SOLVE_FNC);
            #if TEST_MPC == 1
            printf("\n ------ No solve function available! ------ \n");
            #endif
            return;
        }

        //compute how much time was required by the MPC problem resolution
        float solve_time_ms = ((float) (hrt_absolute_time() - time_before_sol)) / 1000.0f;

        #if TEST_MPC == 1
        printf("solvetime:  %4.4f  [mSec]\n", (double)solve_time_ms);
        #endif

        //check solver_ret before using result as rudder command!
        if(solver_ret == 1){
            //compute rudder command at step k to give to the real system: optimalU + rudder_{k-1}
            *p_rudder_cmd = opc_data.forces_output.u0[0] +
                            opc_data.state_extended_model[2];

            #if TEST_MPC == 1
            printf("rudStar: %1.3f \n", (double) *p_rudder_cmd);
            #endif
        }
        else{
            //something went wrong in the solver! Give the last command that has been given
            *p_rudder_cmd = opc_data.state_extended_model[2];
            #if TEST_MPC == 1
            printf("--- recovery Rud: %1.3f \n", (double) *p_rudder_cmd);
            #endif
        }

        #if TEST_MPC == 1
        printf("rudMatl: %1.3f\n", (double) mt_get_correct_rud());
        printf("\n ---------------------- \n \n");
        #endif

        //update time_last_lqr
        opc_data.time_last_mpc = now_us;

        //save optimal control data
        strs_p->boat_opt_status.timestamp = hrt_absolute_time();
        strs_p->boat_opt_status.x1 = opc_data.state_extended_model[0];
        strs_p->boat_opt_status.x2 = opc_data.state_extended_model[1];
        strs_p->boat_opt_status.x3 = opc_data.state_extended_model[2];
        strs_p->boat_opt_status.opt_rud = *p_rudder_cmd;
        strs_p->boat_opt_status.type_controller = 1; //I am the MPC controller
        strs_p->boat_opt_status.it = opc_data.forces_info.it;
        //solvetime in milliseconds
        strs_p->boat_opt_status.solvetime = solve_time_ms;
        strs_p->boat_opt_status.res_eq = opc_data.forces_info.res_eq;
        strs_p->boat_opt_status.pobj = opc_data.forces_info.pobj;
        strs_p->boat_opt_status.dobj = opc_data.forces_info.dobj;
        strs_p->boat_opt_status.dgap = opc_data.forces_info.dgap;
        strs_p->boat_opt_status.rdgap = opc_data.forces_info.rdgap;

        //boat_opt_status just updated
        strs_p->boat_opt_status_updated = true;
    }
    else{
        /* the time elapsed since the last time the MPC was computed is less than
         * MPC_MODEL_TS. Use the last command provided to the rudder that has been
         * saved by the function guidance_module into opc_data.rudder_latest
        */
        *p_rudder_cmd = opc_data.rudder_latest;
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
 * The updated state value is stored in opc_data.state_extended_model.
*/
void compute_state_extended_model(const struct reference_actions_s *ref_act_p){

    //yaw rate from measurements
    opc_data.state_extended_model[0] = cd_get_yaw_rate_sns();

    /* difference from the actual yaw and the desired yaw, assuming true wind
     * will be constant and there will be no drift (in general these
     * two assumptions are not true, but it is the best we can do).
    */
    opc_data.state_extended_model[1] = ref_act_p->alpha_star - cd_get_alpha_dumas();

    //latest command given to the rudder
    opc_data.state_extended_model[2] = opc_data.rudder_latest;
}

/**
 * Compute minusAExt_times_x0 needed by Forces to start the MPC optimization.
*/
void compute_minusAExt_times_x0(float *minusAExt_times_x0){

    /*
     * We do not have AExt memorized, but we do have forces.C vector in coulumn major form.
     * forces.C = [ 0       corresponding index:    0
     *              0                               1
     *              0                               2
     *              -
     *              a11                             3
     *              a21                             4
     *              0                               5
     *              -
     *              a12                             6
     *              a22                             7
     *              0                               8
     *              -
     *              b1                              9
     *              b2                              10
     *              1]                              11
    */
    uint8_t index_current_state = 0;
    for(uint8_t i = 3; i <= 5; i++){

        minusAExt_times_x0[index_current_state] =
                    -opc_data.forces_params.C[i] * opc_data.state_extended_model[0] -
                    opc_data.forces_params.C[i+3] * opc_data.state_extended_model[1] -
                    opc_data.forces_params.C[i+6] * opc_data.state_extended_model[2];

        index_current_state++;
    }
}

/**
 * Set data to specify the band around the origin in which the system,
 * controlled by either the LQR or the MPC, should go at the end of the tack maneuver.
 *
 * @param delta      vetor with three values, delta for yawRate, yaw and rudder, positive values in rads.
 * @param min_time_s minimum time the system should be in the band to consider the tack completed.
 * @param safety_time_stop_tack_s  max time after that a tack mst be forced to be considered completed
*/
void gm_set_band_data(float *delta, float min_time_s, float safety_time_stop_tack_s){

    //save delta and make sure every value is positive
    for(uint8_t i = 0; i < 2; i++){
        opc_data.delta_values[i] = (delta[i] > 0.0f) ? delta[i] : -delta[i];
    }
    //save mint_time
    min_time_s = (min_time_s > 0.0f) ? min_time_s : -min_time_s;
    //convert min_time_s in microsecond and save it
    opc_data.min_time_in_band_us = (uint64_t)((double)min_time_s * 1e6);

    //save max_time
    safety_time_stop_tack_s = (safety_time_stop_tack_s > 0.0f) ? safety_time_stop_tack_s : -safety_time_stop_tack_s;
    //convert max_time in microsecond and save it
    opc_data.safety_time_stop_tack = (uint64_t)((double)safety_time_stop_tack_s * 1e6);
    //safety_time_stop_tack must be >= MIN_SAFETY_TIME_STOP_TCK
    if(opc_data.safety_time_stop_tack < MIN_SAFETY_TIME_STOP_TCK)
        opc_data.safety_time_stop_tack = MIN_SAFETY_TIME_STOP_TCK;

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

    if(command > rudder_controller_data.abs_rud_saturation)
        command = rudder_controller_data.abs_rud_saturation;
    else if(command < -rudder_controller_data.abs_rud_saturation)
        command = -rudder_controller_data.abs_rud_saturation;

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

/**
 * Set Kp and Cp constant for the P controller during tack maneuver type = P_TACK
*/
void gm_set_p_tack_data(float kp, float cp){
    rudder_controller_data.p_tack_kp = kp;
    rudder_controller_data.p_tack_cp = cp;
}

/**
 *
*/
float p_tack_rudder(const struct reference_actions_s *ref_act_p){
    float error = ref_act_p->alpha_star - cd_get_alpha_dumas();
    float gain;

    //conditional P: rud = k(e) * e, where k(e) = kp / (1 + cp * abs(e))
    gain = rudder_controller_data.p_tack_kp / (1.0f + rudder_controller_data.p_tack_cp * my_fabs(error));

    return gain * error;
}

/**
 * Main function to control the boat while sailing.
 *
 * If the remote control is in the manual mode, just copy the rudder and
 * sail commands. In this way they will be shown in QGC and logged in the sdCard.
 * If the autonomous mode is selected, call autonomous sailing controllers.
*/
void gm_guidance_module(struct reference_actions_s *ref_act_p,
                     const struct parameters_qgc *param_qgc_p,
                     struct structs_topics_s *strs_p){

    float rudder_command = 0.0f;
    float sail_command = 0.0f;
    float alpha;//angle with respect to the wind
    //get alpha from the moving average filter in controller_data module
    alpha = cd_get_alpha_dumas();

    //check if we are in the manual mode
    if(strs_p->rc_channels.channels[RC_MODE_INDEX] == RC_MANUAL_MODE){
        //read rudder and sails command from RC and save them
        rudder_command = strs_p->rc_channels.channels[RC_RUD_INDEX];
        sail_command = strs_p->rc_channels.channels[RC_SAIL_INDEX];
        //tel to path planning what's the current alpha
        pp_set_current_alpha(alpha);
    }
    else{
        //Autonomous mode

        //check if path_planning() told us to tack
        if(ref_act_p->should_tack){
            //perform tack maneuver
            tack_action(ref_act_p, &rudder_command, &sail_command, strs_p);
        }
        if(!(ref_act_p->should_tack)){
            //if the boat should not tack, compute rudder and sails actions to follow reference alpha
            //PI controller for rudder
            rudder_command = pi_controller(&(ref_act_p->alpha_star), &alpha);

            //sails control only if AS_SAIL param from QGC is negative
            if(param_qgc_p->sail_servo < 0.0f)
                sail_command = sail_controller(alpha);
            else
                sail_command = param_qgc_p->sail_servo;
        }


        //saturation for safety reason
        rudder_command = rudder_saturation(rudder_command);
        sail_command = sail_saturation(sail_command);

        #if SIMULATION_FLAG == 1
        //rudder_command = param_qgc_p->deva1;
        #endif
    }

    #if TEST_MPC == 1
    mpc_control_rudder(&rudder_command, ref_act_p, strs_p);
    #endif

    //update actuator value
    strs_p->actuators.control[0] = rudder_command;
    strs_p->actuators.control[3] =  sail_command;

    //update rudder_latest in any case, even if we are not tacking
    opc_data.rudder_latest = rudder_command;

    //save first debug values for post-processing, other values set in path_planning()
    strs_p->boat_guidance_debug.alpha = alpha;
    strs_p->boat_guidance_debug.twd_mean = cd_get_twd_sns();
    strs_p->boat_guidance_debug.app_mean = cd_get_app_wind_sns();
    strs_p->boat_guidance_debug.timestamp = hrt_absolute_time();
    strs_p->boat_guidance_debug.rudder_action = rudder_command;
    strs_p->boat_guidance_debug.sail_action = sail_command;

    #if SIMULATION_FLAG == 1
    //strs_p->airspeed.true_airspeed_m_s = ref_act_p->alpha_star - get_alpha_dumas();
    #endif
}

