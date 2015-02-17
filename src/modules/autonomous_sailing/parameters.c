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
 * @file parameters.c
 *
 * Parameters from QGroundControl for autonomous sailing application.
 *
 * @author Marco Tranzatto <marco.tranzatto@gmail.com>
 */

#include "parameters.h"

static const float deg2rad = 0.0174532925199433f; // pi / 180



/**
 * Reference angle with respect true wind [deg]
 *
 * Use Dumas'convention.
 *
 * @min -180
 * @max 180
 */
PARAM_DEFINE_FLOAT(AS_ALST_ANG_D, 45.0f);

/**
 * 1 if you wish to use the alpha star specified by AS_ALSTR, 0 otherwise
 *
 * @min 0
 * @max 1
 */
PARAM_DEFINE_INT32(AS_ALST_SET, 1);

/**
 * Swtich this value from 0 to 1 if you want to tack.
 * Then rememeber to switch it to 0 after the tack is completed.
 * Attention: if AS_TCK_NOW switches from 0 to 1, the parameter
 * AS_ALST_ANG_D will NOT be considered!
*/
PARAM_DEFINE_INT32(AS_TCK_NOW, 0);

/**
 * Sails position
 *
 * Default value for sails position. 0.56 = fully closed, 0.0 = fully opened.
 * If a negative value is set, then the sail control is in charge to control sails.
 *
 * @min 0
 * @max 0.56
 */
PARAM_DEFINE_FLOAT(AS_SAIL, -1.0f);


/**
 * Rudder maximum command when tacking from port haul to starboard haul.
 * It has to be a positive value!
 *
 *
 * @min 0
 * @max 0.9
 */
PARAM_DEFINE_FLOAT(AS_RD_45_CMD, 0.8f);

/**
 * Alpha angle [deg].
 *
 * @min 0
 * @max 180
 */
PARAM_DEFINE_FLOAT(AS_RD_X1_AL, 0.0f);

/**
 * Alpha angle [deg].
 *
 * @min 0
 * @max 180
 */
PARAM_DEFINE_FLOAT(AS_RD_X2_AL, 20.0f);

/**
 * Sails command when sails should be considered fully closed.
 *
 * @min 0
 * @max 0.56
 */
PARAM_DEFINE_FLOAT(AS_SAI_CL_CMD, 0.56f);

/**
 * Positive alpha angle [deg] at which sails should start opening
 *
 * @min 0
 * @max 180
 */
PARAM_DEFINE_FLOAT(AS_SAI_X1_AL, 45.0f);

/**
 * Positive alpha angle [deg] at which sails should be fully opened
 *
 * @min 0
 * @max 180
 */
PARAM_DEFINE_FLOAT(AS_SAI_X2_AL, 150.0f);

/**
 * Positive alpha angle [deg] at which tack from port to starboard
 * should be considered completed
 *
 * @min 0
 * @max 180
 */
PARAM_DEFINE_FLOAT(AS_TK_ST_AL, 35.0f);


/**
 * Type of tack maneuver
 *
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
 * @min 0
 * @max 4
 */
PARAM_DEFINE_INT32(AS_TY_TCK, 0);

/**
 * Proportional gain for rudder PI.
 *
 *
 * @min 0
 * @max ?
 */
PARAM_DEFINE_FLOAT(AS_RUD_P, 0.03f);

/**
 * Integral gain for rudder PI.
 *
 *
 * @min 0
 * @max ?
 */
PARAM_DEFINE_FLOAT(AS_RUD_I, 0.0f);

/**
 * Constant fo anti-wind up in normal digital PI
 *
 *
 * @min 0
 * @max ?
 */
PARAM_DEFINE_FLOAT(AS_RUD_KAW, 0.5f);

/**
 * Constant used in conditionl integral to adjust denominator in integral action.
 *
 *
 * @min 0
 * @max ?
 */
PARAM_DEFINE_FLOAT(AS_RUD_CI, 1.0f);

/**
 * Constant used in conditionl integral to adjust denominator in proportional action.
 *
 *
 * @min 0
 * @max ?
 */
PARAM_DEFINE_FLOAT(AS_RUD_CP, 1.0f);

/**
 * Select which type of rudder control you want to have to track a desired
 * alpha angle.
 *
 * 0: standard PI with anti-wind up gain
 * 1: conditional PI
 * 2: LQR controller
 *
 * @min 0
 * @max 2
 */
PARAM_DEFINE_INT32(AS_RUD_TYPE, 0);


/**
 * Latitude of origin of NED system, in degrees * E7.
 *
 *
 * @min -900000000
 * @max 900000000
 */
PARAM_DEFINE_INT32(AS_R_LAT0_E7, 473494820);

/**
 * Longitude of origin of NED system, in degrees * E7.
 *
 *
 * @min -1800000000
 * @max 1800000000
 */
PARAM_DEFINE_INT32(AS_R_LON0_E7, 85605120);

/**
 * Altitude of origin of NED system, in millimeters.
 *
 *
 * @min 0
 * @max ?
 */
PARAM_DEFINE_INT32(AS_R_ALT0_E3, 406000);

/**
 * AS_WIN_AL, specifies the number of samples for the moving average of true wind angle (alpha).
 *
 *
 * @min 1
 * @max ?
 */
PARAM_DEFINE_INT32(AS_WIN_AL, 10);

/**
 * AS_WIN_APP, specifies the number of samples for the moving average of apparent wind direction.
 *
 *
 * @min 1
 * @max ?
 */
PARAM_DEFINE_INT32(AS_WIN_APP, 10);

/**
 * AS_WIN_TWD, specifies the number of samples for the moving average of true wind direction.
 *
 *
 * @min 1
 * @max ?
 */
PARAM_DEFINE_INT32(AS_WIN_TWD, 10);

/**
 * AS_MEAN_WIND_D, specifies the mean wind direction [deg], in [-180, 180].
 * Positive on the right (going from North to East), negative on the left (going from North to West).
 *
 *
 * @min -180
 * @max 180
 */
PARAM_DEFINE_FLOAT(AS_MEAN_WIND_D, 0.0f);

/**
 * Latitude of top mark, in degrees * E7.
 *
 *
 * @min -900000000
 * @max 900000000
 */
PARAM_DEFINE_INT32(AS_T_LAT_E7, 473459370);

/**
 * Longitude of top mark, in degrees * E7.
 *
 *
 * @min -1800000000
 * @max 1800000000
 */
PARAM_DEFINE_INT32(AS_T_LON_E7, 85547940);

/**
 * Altitude of top mark, in millimeters.
 *
 *
 * @min 0
 * @max ?
 */
PARAM_DEFINE_INT32(AS_T_ALT_E3, 406000);

/**
 * Total numbers of grid lines
 *
 *
 * @min 1
 * @max ?
 */
PARAM_DEFINE_INT32(AS_P_TOT_G, 1);

/**
 * X coordinate in Race frame of grid line of index AS_P_INDEX, [m]
 *
 *
 * @min ?
 * @max ?
 */
PARAM_DEFINE_FLOAT(AS_P_X_M, 0.0f);

/**
 * 1 if you want to add a new grid line
 *
 *
 * @min 0
 * @max 1
 */
PARAM_DEFINE_INT32(AS_P_ADD, 0);

/**
 * 1 if you want to re-insert the same grid lines you used before
 *
 *
 * @min 0
 * @max 1
 */
PARAM_DEFINE_INT32(AS_REIN_GRS, 0);

/**
 * Set how much time should passed without any new cog value.
 * @see set_max_time_cog_not_up
 */
PARAM_DEFINE_FLOAT(AS_COG_DELAY_S, 1.5f);



//------------------------------------- Parameters for optimal control ----


/**
 * First value of the optimal gain matrix K, for the LQR controller
 *
 */
PARAM_DEFINE_FLOAT(ASO_LQR_K1, -0.2256399447f);

/**
 * Second value of the optimal gain matrix K, for the LQR controller
 *
 */
PARAM_DEFINE_FLOAT(ASO_LQR_K2, -1.3190228742f);

/**
 * Third value of the optimal gain matrix K, for the LQR controller
 *
 */
PARAM_DEFINE_FLOAT(ASO_LQR_K3, 0.1245043534f);

/**
 * First value of the Hessina cost matrix, for the MPC
 */
PARAM_DEFINE_FLOAT(ASO_MPC_H1, 5.0f);

/**
 * Second value of the Hessina cost matrix, for the MPC
 */
PARAM_DEFINE_FLOAT(ASO_MPC_H2, 0.01f);

/**
 * Third value of the Hessina cost matrix, for the MPC
 */
PARAM_DEFINE_FLOAT(ASO_MPC_H3, 10.0f);

/**
 * Fourth value of the Hessina cost matrix, for the MPC
 */
PARAM_DEFINE_FLOAT(ASO_MPC_H4, 0.01f);

/**
 * First lower bound value, for the MPC
 */
PARAM_DEFINE_FLOAT(ASO_MPC_LB1, -0.3820419310f);

/**
 * Second lower bound value, for the MPC
 */
PARAM_DEFINE_FLOAT(ASO_MPC_LB2, -0.9f);

/**
 * First upper bound value, for the MPC
 */
PARAM_DEFINE_FLOAT(ASO_MPC_UB1, 0.3820419310f);

/**
 * Second upper bound value, for the MPC
 */
PARAM_DEFINE_FLOAT(ASO_MPC_UB2, 0.9f);

/**
 * Hessian matrix for final cost, only for state, not for input. For the MPC.
 */
PARAM_DEFINE_FLOAT(ASO_MPC_HF22, 1.8334714813f);
PARAM_DEFINE_FLOAT(ASO_MPC_HF23, 8.9970253273f);
PARAM_DEFINE_FLOAT(ASO_MPC_HF24, -1.3281703473f);

PARAM_DEFINE_FLOAT(ASO_MPC_HF32, 8.9970253273f);
PARAM_DEFINE_FLOAT(ASO_MPC_HF33, 60.6690232034f);
PARAM_DEFINE_FLOAT(ASO_MPC_HF34, -5.5144280101f);

PARAM_DEFINE_FLOAT(ASO_MPC_HF42, -1.3281703473f);
PARAM_DEFINE_FLOAT(ASO_MPC_HF43, -5.5144280101f);
PARAM_DEFINE_FLOAT(ASO_MPC_HF44, 1.8832249457f);

/**
 * Create a band around the origin for the yaw rate value.
 * Value in degrees.
 *
 * @min 0
*/
PARAM_DEFINE_FLOAT(ASO_DLT_YR_D, 5.0f);

/**
 * Create a band around the origin for the yaw value.
 * Value in degrees.
 *
 * @min 0
*/
PARAM_DEFINE_FLOAT(ASO_DLT_Y_D, 8.0f);

/**
 * Create a band around the origin for the rudder command.
 *
 * @min 0
 * @max 0.9
*/
PARAM_DEFINE_FLOAT(ASO_DLT_RD_CM, 0.15f);

/**
 * Min time (in seconds) the state of the system should stay in
 * the band near the origin (defined by @see ASO_DLT_YR_D, @see ASO_DLT_Y_D
 * and @see ASO_DLT_RD_CM) in order to consider the tack maneuver completed.
 *
 * @min 0
*/
PARAM_DEFINE_FLOAT(ASO_STP_TCK_S, 0.8f);

#if SIMULATION_FLAG == 1

//---------------------------------------- Simulation variables --------------------

/**
 * Simulated Latitude, in degrees * E7.
 *
 *
 * @min -900000000
 * @max 900000000
 */
PARAM_DEFINE_INT32(ASIM_LAT_E7, 473494820);

/**
 * Simulated Longitude, in degrees * E7.
 *
 *
 * @min -1800000000
 * @max 1800000000
 */
PARAM_DEFINE_INT32(ASIM_LON_E7, 85605120);

/**
 * Simulated Altitude, in millimeters.
 *
 *
 * @min 0
 * @max ?
 */
PARAM_DEFINE_INT32(ASIM_ALT_E3, 406000);

/**
 * Simulated Course over ground, in deg, sign opposite to Dumas convention.
 *
 *
 * @min -180
 * @max 180
 */
PARAM_DEFINE_FLOAT(ASIM_COG_D, 0.0f);

/**
 * Simulated true wind direction, in deg, sign opposite to Dumas convention.
 *
 *
 * @min -180
 * @max 180
 */
PARAM_DEFINE_FLOAT(ASIM_TWD_D, 0.0f);

/**
 * Simulated yaw, in deg, sign opposite to Dumas convention.
 *
 *
 * @min -180
 * @max 180
 */
PARAM_DEFINE_FLOAT(ASIM_YAW_D, 0.0f);

/**
 * Temporary value for debug porpuses.
 */
PARAM_DEFINE_FLOAT(ASIM_DEVA1, 0.0f);

#endif



static struct pointers_param_qgc_s{

    param_t alpha_star_pointer;         /**< pointer to param AS_ALST_ANG_D*/
    param_t use_alpha_star_pointer;         /**< pointer to param AS_ALST_SET*/
    param_t tack_now;                   /**< pointer to param TEST_MPC */

    param_t sail_pointer;         /**< pointer to param AS_SAIL*/

    param_t tack_type_pointer;       /**< pointer to param AS_TY_TCK*/

    param_t rud_p_gain_pointer;       /**< pointer to param AS_RUD_P*/
    param_t rud_i_gain_pointer;       /**< pointer to param AS_RUD_I*/
    param_t rud_kaw_pointer;       /**< pointer to param AS_RUD_KAW*/
    param_t rud_ci_pointer;       /**< pointer to param AS_RUD_CI*/
    param_t rud_cp_pointer;       /**< pointer to param AS_RUD_CP*/
    param_t rud_controller_type_pointer;       /**< pointer to param AS_RUD_TYPE*/

    param_t tack_rudder_cmd;        /**< pointer to param AS_RD_45_CMD*/
    param_t tack_rudder_x1;      /**< pointer to param AS_RD_X1_AL*/
    param_t tack_rudder_x2;      /**< pointer to param AS_RD_X2_AL*/
    param_t sails_closed_cmd;       /**< pointer to param AS_SAI_CL_CMD*/
    param_t sails_closed_alpha;     /**< pointer to param AS_SAI_X1_AL*/
    param_t sails_opened_alpha;     /**< pointer to param AS_SAI_X2_AL*/
    param_t tack_stop_alpha;        /**< pointer to param AS_TK_ST_AL*/

    param_t lat0_pointer;         /**< pointer to param AS_R_LAT0_E7*/
    param_t lon0_pointer;         /**< pointer to param AS_R_LON0_E7*/
    param_t alt0_pointer;         /**< pointer to param AS_R_ALT0_E3*/


    param_t moving_alpha_window_pointer;/**< pointer to param AS_WIN_AL*/
    param_t moving_apparent_window_pointer;/**< pointer to param AS_WIN_APP*/
    param_t moving_twd_window_pointer;/**< pointer to param AS_WIN_TWD*/

    param_t mean_wind_pointer;/**< pointer to param AS_MEAN_WIND_D*/

    param_t lat_tmark_pointer;         /**< pointer to param AS_T_LAT_E7*/
    param_t lon_tmark_pointer;         /**< pointer to param AS_T_LON_E7*/
    param_t alt_tmark_pointer;         /**< pointer to param AS_T_ALT_E3*/

    param_t grids_number_pointer;         /**< pointer to param AS_P_TOT_G*/
    param_t grid_x_pointer;         /**< pointer to param AS_P_X_M*/
    param_t grid_add_pointer;         /**< pointer to param AS_P_ADD*/
    param_t repeat_past_grids_pointer;    /**< pointer to param AS_REIN_GRS */

    //-- params for LQR controller
    param_t lqr_k1_poniter; /**< pointer to  ASO_LQR_K1*/
    param_t lqr_k2_poniter; /**< pointer to  ASO_LQR_K2*/
    param_t lqr_k3_poniter; /**< pointer to  ASO_LQR_K3*/

    //-- params for MPC controller
    param_t mpc_h1_pointer; /**< pointer to ASO_MPC_H1*/
    param_t mpc_h2_pointer; /**< pointer to ASO_MPC_H2*/
    param_t mpc_h3_pointer; /**< pointer to ASO_MPC_H3*/
    param_t mpc_h4_pointer; /**< pointer to ASO_MPC_H4*/

    param_t mpc_lb1_pointer; /**< pointer to ASO_MPC_LB1*/
    param_t mpc_lb2_pointer; /**< pointer to ASO_MPC_LB2*/

    param_t mpc_ub1_pointer; /**< pointer to ASO_MPC_UB1*/
    param_t mpc_ub2_pointer; /**< pointer to ASO_MPC_UB1*/

    param_t mpc_hf_22_pointer; /**< pointer to ASO_MPC_HF22*/
    param_t mpc_hf_23_pointer; /**< pointer to ASO_MPC_HF23*/
    param_t mpc_hf_24_pointer; /**< pointer to ASO_MPC_HF24*/

    param_t mpc_hf_32_pointer; /**< pointer to ASO_MPC_HF32*/
    param_t mpc_hf_33_pointer; /**< pointer to ASO_MPC_HF33*/
    param_t mpc_hf_34_pointer; /**< pointer to ASO_MPC_HF34*/

    param_t mpc_hf_42_pointer; /**< pointer to ASO_MPC_HF42*/
    param_t mpc_hf_43_pointer; /**< pointer to ASO_MPC_HF43*/
    param_t mpc_hf_44_pointer; /**< pointer to ASO_MPC_HF44*/

    //--- params to define the band near the origin
    param_t delta_yaw_rate_pointer; /**< pointer to ASO_DLT_YR_D*/
    param_t delta_yaw_pointer; /**< pointer to ASO_DLT_Y_D*/
    param_t delta_rudder_pointer; /**< pointer to ASO_DLT_RD_CM*/

    param_t min_time_in_band_poniter; /**< pointer to ASO_STP_TCK_S*/


    //---cog delay
    param_t cog_max_delay_pointer;/**< pointer to param AS_COG_DELAY_S*/

    //-- simulation params

    #if SIMULATION_FLAG == 1
    param_t lat_sim_pointer; /**< pointer to param ASIM_LAT_E7*/
    param_t lon_sim_pointer; /**< pointer to param ASIM_LON_E7*/
    param_t alt_sim_pointer; /**< pointer to param ASIM_ALt_E3*/

    param_t twd_sim_pointer; /**< pointer to param ASIM_TWD_D*/
    param_t cog_sim_pointer; /**< pointer to param ASIM_COG_D*/


    param_t yaw_sim_pointer; /**< pointer to param ASIM_YAW_D*/
    param_t deva1_sim_pointer; /**< pointer to param ASIM_DEVA1 */
    #endif
}pointers_param_qgc;


/**
* Initialize parameters.
*
*/
void param_init(struct parameters_qgc *params_p,
                struct structs_topics_s *strs_p,
                const struct published_fd_s *pubs_p){

    //initialize pointer to parameters
    pointers_param_qgc.alpha_star_pointer    = param_find("AS_ALST_ANG_D");
    pointers_param_qgc.use_alpha_star_pointer    = param_find("AS_ALST_SET");
    pointers_param_qgc.tack_now = param_find("AS_TCK_NOW");

    pointers_param_qgc.sail_pointer    = param_find("AS_SAIL");

    pointers_param_qgc.tack_type_pointer  = param_find("AS_TY_TCK");

    pointers_param_qgc.rud_p_gain_pointer  = param_find("AS_RUD_P");
    pointers_param_qgc.rud_i_gain_pointer  = param_find("AS_RUD_I");
    pointers_param_qgc.rud_kaw_pointer = param_find("AS_RUD_KAW");
    pointers_param_qgc.rud_ci_pointer  = param_find("AS_RUD_CI");
    pointers_param_qgc.rud_cp_pointer  = param_find("AS_RUD_CP");
    pointers_param_qgc.rud_controller_type_pointer  = param_find("AS_RUD_TYPE");

    pointers_param_qgc.tack_rudder_cmd = param_find("AS_RD_45_CMD");
    pointers_param_qgc.tack_rudder_x1 = param_find("AS_RD_X1_AL");
    pointers_param_qgc.tack_rudder_x2 = param_find("AS_RD_X2_AL");
    pointers_param_qgc.sails_closed_cmd = param_find("AS_SAI_CL_CMD");
    pointers_param_qgc.sails_closed_alpha = param_find("AS_SAI_X1_AL");
    pointers_param_qgc.sails_opened_alpha = param_find("AS_SAI_X2_AL");
    pointers_param_qgc.tack_stop_alpha = param_find("AS_TK_ST_AL");

    pointers_param_qgc.lat0_pointer    = param_find("AS_R_LAT0_E7");
    pointers_param_qgc.lon0_pointer    = param_find("AS_R_LON0_E7");
    pointers_param_qgc.alt0_pointer    = param_find("AS_R_ALT0_E3");

    pointers_param_qgc.moving_alpha_window_pointer = param_find("AS_WIN_AL");
    pointers_param_qgc.moving_apparent_window_pointer = param_find("AS_WIN_APP");
    pointers_param_qgc.moving_twd_window_pointer = param_find("AS_WIN_TWD");

    pointers_param_qgc.mean_wind_pointer = param_find("AS_MEAN_WIND_D");

    pointers_param_qgc.lat_tmark_pointer    = param_find("AS_T_LAT_E7");
    pointers_param_qgc.lon_tmark_pointer    = param_find("AS_T_LON_E7");
    pointers_param_qgc.alt_tmark_pointer    = param_find("AS_T_ALT_E3");

    pointers_param_qgc.grids_number_pointer    = param_find("AS_P_TOT_G");
    pointers_param_qgc.grid_x_pointer    = param_find("AS_P_X_M");

    pointers_param_qgc.grid_add_pointer = param_find("AS_P_ADD");
    pointers_param_qgc.repeat_past_grids_pointer = param_find("AS_REIN_GRS");

    //--- params for lqr controller
    pointers_param_qgc.lqr_k1_poniter = param_find("ASO_LQR_K1");
    pointers_param_qgc.lqr_k2_poniter = param_find("ASO_LQR_K2");
    pointers_param_qgc.lqr_k3_poniter = param_find("ASO_LQR_K3");

    //--- params for MPC controller
    pointers_param_qgc.mpc_h1_pointer = param_find("ASO_MPC_H1");
    pointers_param_qgc.mpc_h2_pointer = param_find("ASO_MPC_H2");
    pointers_param_qgc.mpc_h3_pointer = param_find("ASO_MPC_H3");
    pointers_param_qgc.mpc_h4_pointer = param_find("ASO_MPC_H4");

    pointers_param_qgc.mpc_lb1_pointer = param_find("ASO_MPC_LB1");
    pointers_param_qgc.mpc_lb2_pointer = param_find("ASO_MPC_LB2");

    pointers_param_qgc.mpc_ub1_pointer = param_find("ASO_MPC_UB1");
    pointers_param_qgc.mpc_ub2_pointer = param_find("ASO_MPC_UB2");

    pointers_param_qgc.mpc_hf_22_pointer = param_find("ASO_MPC_HF22");
    pointers_param_qgc.mpc_hf_23_pointer = param_find("ASO_MPC_HF23");
    pointers_param_qgc.mpc_hf_24_pointer = param_find("ASO_MPC_HF24");

    pointers_param_qgc.mpc_hf_32_pointer = param_find("ASO_MPC_HF32");
    pointers_param_qgc.mpc_hf_33_pointer = param_find("ASO_MPC_HF33");
    pointers_param_qgc.mpc_hf_34_pointer = param_find("ASO_MPC_HF34");

    pointers_param_qgc.mpc_hf_42_pointer = param_find("ASO_MPC_HF42");
    pointers_param_qgc.mpc_hf_43_pointer = param_find("ASO_MPC_HF43");
    pointers_param_qgc.mpc_hf_44_pointer = param_find("ASO_MPC_HF44");

    //--- band params
    pointers_param_qgc.delta_yaw_rate_pointer = param_find("ASO_DLT_YR_D");
    pointers_param_qgc.delta_yaw_pointer = param_find("ASO_DLT_Y_D");
    pointers_param_qgc.delta_rudder_pointer = param_find("ASO_DLT_RD_CM");

    pointers_param_qgc.min_time_in_band_poniter = param_find("ASO_STP_TCK_S");

    //----cog delay
    pointers_param_qgc.cog_max_delay_pointer = param_find("AS_COG_DELAY_S");

    #if SIMULATION_FLAG == 1

    pointers_param_qgc.lat_sim_pointer = param_find("ASIM_LAT_E7");
    pointers_param_qgc.lon_sim_pointer = param_find("ASIM_LON_E7");
    pointers_param_qgc.alt_sim_pointer = param_find("ASIM_ALT_E3");

    pointers_param_qgc.cog_sim_pointer = param_find("ASIM_COG_D");
    pointers_param_qgc.twd_sim_pointer = param_find("ASIM_TWD_D");

    pointers_param_qgc.yaw_sim_pointer = param_find("ASIM_YAW_D");
    pointers_param_qgc.deva1_sim_pointer = param_find("ASIM_DEVA1");

    #endif

    //get parameters but do not add any grid lines at start up
    param_update(params_p, strs_p, false, pubs_p);

}

/** Update local copy of parameters.
 *
*/
void param_update(struct parameters_qgc *params_p,
                  struct structs_topics_s *strs_p, bool update_path_param,
                  const struct published_fd_s *pubs_p){

    //----- alpha star
    float alpha_tmp;
    int32_t set_alpha;
    int32_t tack_now;

    param_get(pointers_param_qgc.alpha_star_pointer, &alpha_tmp);
    //convert alpha in rad
    alpha_tmp = alpha_tmp * deg2rad;

    //take set_alpha to see if alpha_tmp has to be set as the new alpha star value
    param_get(pointers_param_qgc.use_alpha_star_pointer, &set_alpha);

    //tack tack_now to see if the boat should tack now, if so, DO NOT set alpha_tmp as new alpha star
    param_get(pointers_param_qgc.tack_now, &tack_now);

    //set alpha_tmp as the new alpha star ONLY if set_alpha is not 0 AND tack_now is 0
    if(set_alpha != 0 && tack_now == 0)
        set_alpha_star(alpha_tmp);

    //pass tack_now to path_planning module, only if update_path_param is true
    if(update_path_param)
        boat_should_tack(tack_now);

    //----- sail_servo
    param_get(pointers_param_qgc.sail_pointer, &(params_p->sail_servo));

    //----- tack type
    int32_t tack_type;
    float alpha_min_stop_tack_r;

    param_get(pointers_param_qgc.tack_type_pointer, &tack_type);
    param_get(pointers_param_qgc.tack_stop_alpha, &alpha_min_stop_tack_r);
    alpha_min_stop_tack_r = alpha_min_stop_tack_r * deg2rad;
    set_tack_data((uint16_t)tack_type, alpha_min_stop_tack_r);

    //----- param for rudder controller
    float rud_p;
    float rud_i;
    float rud_ci;
    float rud_cp;
    float rud_kaw;
    int32_t rudder_controller_type;
    float alpha_rudder_x1_r;
    float alpha_rudder_x2_r;
    float rud_cmd_45_left;

    param_get(pointers_param_qgc.rud_p_gain_pointer, &rud_p);
    param_get(pointers_param_qgc.rud_i_gain_pointer, &rud_i);
    param_get(pointers_param_qgc.rud_kaw_pointer, &rud_kaw);
    param_get(pointers_param_qgc.rud_ci_pointer, &rud_ci);
    param_get(pointers_param_qgc.rud_cp_pointer, &rud_cp);
    param_get(pointers_param_qgc.rud_controller_type_pointer, &rudder_controller_type);

    param_get(pointers_param_qgc.tack_rudder_x1, &alpha_rudder_x1_r);
    param_get(pointers_param_qgc.tack_rudder_x2, &alpha_rudder_x2_r);
    param_get(pointers_param_qgc.tack_rudder_cmd, &rud_cmd_45_left);

    alpha_rudder_x1_r = alpha_rudder_x1_r * deg2rad;
    alpha_rudder_x2_r = alpha_rudder_x2_r * deg2rad;

    set_rudder_data(rud_p, rud_i, rud_cp, rud_ci, rudder_controller_type, rud_kaw,
                    alpha_rudder_x1_r, alpha_rudder_x2_r, rud_cmd_45_left);

    //----- sails controller
    float sail_closed_cmd;
    float alpha_sail_closed_r;
    float alpha_sail_opened_r;

    param_get(pointers_param_qgc.sails_closed_cmd, &sail_closed_cmd);
    param_get(pointers_param_qgc.sails_closed_alpha, &alpha_sail_closed_r);
    param_get(pointers_param_qgc.sails_opened_alpha, &alpha_sail_opened_r);

    alpha_sail_closed_r = alpha_sail_closed_r * deg2rad;
    alpha_sail_opened_r = alpha_sail_opened_r * deg2rad;

    set_sail_data(sail_closed_cmd, alpha_sail_closed_r, alpha_sail_opened_r);

    //----- reference geo coordinate
    int32_t lat0;
    int32_t lon0;
    int32_t alt0;
    //lat0
    param_get(pointers_param_qgc.lat0_pointer, &lat0);

    //lon0
    param_get(pointers_param_qgc.lon0_pointer, &lon0);

    //alt0
    param_get(pointers_param_qgc.alt0_pointer, &alt0);

    //update NED origin using API in navigation.h
    set_ref0(&lat0, &lon0, &alt0);


    //----- moving windows
    uint16_t window_alpha;
    uint16_t window_apparent;
    uint16_t window_twd;

    param_get(pointers_param_qgc.moving_alpha_window_pointer, &window_alpha);
    //update window size using API in controller_data.h
    update_k(window_alpha);

    param_get(pointers_param_qgc.moving_apparent_window_pointer, &window_apparent);
    //update window size using API in controller_data.h
    update_k_app(window_apparent);

    param_get(pointers_param_qgc.moving_twd_window_pointer, &window_twd);
    //update window size using API in controller_data.h
    update_k_twd(window_twd);

    //----- mean wind
    float mean_wind;
    param_get(pointers_param_qgc.mean_wind_pointer, &mean_wind);
    //convert mean_wind in rad
    mean_wind = mean_wind * deg2rad;
    //set mean wind angle in navigation.h
    set_mean_wind_angle(mean_wind);

    //----- top mark geo coordinate
    int32_t lat_tmark;
    int32_t lon_tmark;
    int32_t alt_tmark;
    //lat_tmark
    param_get(pointers_param_qgc.lat_tmark_pointer, &lat_tmark);

    //lon_tmark
    param_get(pointers_param_qgc.lon_tmark_pointer, &lon_tmark);

    //alt_tmark
    param_get(pointers_param_qgc.alt_tmark_pointer, &alt_tmark);

    //set top mark position
    set_pos_top_mark(&lat_tmark, &lon_tmark, &alt_tmark);

    //----- number of grids
    int32_t grids_number;
    float grids_x_m;
    param_get(pointers_param_qgc.grids_number_pointer, &grids_number);

    //x coordinate of current grid line
    param_get(pointers_param_qgc.grid_x_pointer, &grids_x_m);

    //check if we have to add a new grid line
    int32_t temp = 0;
    param_get(pointers_param_qgc.grid_add_pointer, &temp);
    if(temp > 0 && update_path_param){
        //set x coordinate of a new grid line
        set_grid_qgc(grids_x_m);
    }

    //set the new number of grid lines
    set_grids_number_qgc(grids_number);

    param_get(pointers_param_qgc.repeat_past_grids_pointer, &temp);
    bool use_last_grids = (temp > 0) ? true : false;
    reuse_last_grids(use_last_grids);



    //-- param for LQR controller
    float lqr_k1;
    float lqr_k2;
    float lqr_k3;

    param_get(pointers_param_qgc.lqr_k1_poniter, &lqr_k1);
    param_get(pointers_param_qgc.lqr_k2_poniter, &lqr_k2);
    param_get(pointers_param_qgc.lqr_k3_poniter, &lqr_k3);

    set_lqr_gain(lqr_k1, lqr_k2, lqr_k3);

    //-- param for MPC controller
    float mpc_h[4];
    float mpc_lb[2];
    float mpc_ub[2];
    float mpc_hf[3][3];

    param_get(pointers_param_qgc.mpc_h1_pointer, &mpc_h[0]);
    param_get(pointers_param_qgc.mpc_h2_pointer, &mpc_h[1]);
    param_get(pointers_param_qgc.mpc_h3_pointer, &mpc_h[2]);
    param_get(pointers_param_qgc.mpc_h4_pointer, &mpc_h[3]);

    param_get(pointers_param_qgc.mpc_lb1_pointer, &mpc_lb[0]);
    param_get(pointers_param_qgc.mpc_lb2_pointer, &mpc_lb[1]);

    param_get(pointers_param_qgc.mpc_ub1_pointer, &mpc_ub[0]);
    param_get(pointers_param_qgc.mpc_ub2_pointer, &mpc_ub[1]);

    param_get(pointers_param_qgc.mpc_hf_22_pointer, &mpc_hf[0][0]);
    param_get(pointers_param_qgc.mpc_hf_23_pointer, &mpc_hf[0][1]);
    param_get(pointers_param_qgc.mpc_hf_24_pointer, &mpc_hf[0][2]);

    param_get(pointers_param_qgc.mpc_hf_32_pointer, &mpc_hf[1][0]);
    param_get(pointers_param_qgc.mpc_hf_33_pointer, &mpc_hf[1][1]);
    param_get(pointers_param_qgc.mpc_hf_34_pointer, &mpc_hf[1][2]);

    param_get(pointers_param_qgc.mpc_hf_42_pointer, &mpc_hf[2][0]);
    param_get(pointers_param_qgc.mpc_hf_43_pointer, &mpc_hf[2][1]);
    param_get(pointers_param_qgc.mpc_hf_44_pointer, &mpc_hf[2][2]);

    set_mpc_data(mpc_h, mpc_lb, mpc_ub, mpc_hf);

    //--- define band around origin
    float delta_vect[3];
    float min_time_in_band;

    param_get(pointers_param_qgc.delta_yaw_rate_pointer, &delta_vect[0]);
    param_get(pointers_param_qgc.delta_yaw_pointer, &delta_vect[1]);
    param_get(pointers_param_qgc.delta_rudder_pointer, &delta_vect[2]);

    //convert delta values from deg to rad
    for(uint8_t i = 0; i < 3; i++)
        delta_vect[i] = delta_vect[i] * deg2rad;

    param_get(pointers_param_qgc.min_time_in_band_poniter, &min_time_in_band);

    set_band_data(delta_vect, min_time_in_band);

    //--- cog delay
    float cog_max_delay_sec;
    param_get(pointers_param_qgc.cog_max_delay_pointer, &cog_max_delay_sec);
    set_max_time_cog_not_up(cog_max_delay_sec);

    //save boat_opt_matrices
    strs_p->boat_opt_mat.timestamp = hrt_absolute_time();

    strs_p->boat_opt_mat.lqr_k1 = lqr_k1;
    strs_p->boat_opt_mat.lqr_k2 = lqr_k2;
    strs_p->boat_opt_mat.lqr_k3 = lqr_k3;

    strs_p->boat_opt_mat.mpc_h1 = mpc_h[0];
    strs_p->boat_opt_mat.mpc_h2 = mpc_h[1];
    strs_p->boat_opt_mat.mpc_h3 = mpc_h[2];
    strs_p->boat_opt_mat.mpc_h4 = mpc_h[3];

    strs_p->boat_opt_mat.mpc_lb1 = mpc_lb[0];
    strs_p->boat_opt_mat.mpc_lb2 = mpc_lb[1];

    strs_p->boat_opt_mat.mpc_ub1 = mpc_ub[0];
    strs_p->boat_opt_mat.mpc_ub2 = mpc_ub[1];

    orb_publish(ORB_ID(boat_opt_mat), pubs_p->boat_opt_mat, &(strs_p->boat_opt_mat));

    //save interested param in boat_qgc_param and publish this topic
    strs_p->boat_qgc_param1.timestamp = hrt_absolute_time();
    strs_p->boat_qgc_param1.rud_p = rud_p;
    strs_p->boat_qgc_param1.rud_i = rud_i;
    strs_p->boat_qgc_param1.rud_kaw = rud_kaw;
    strs_p->boat_qgc_param1.rud_cp = rud_cp;
    strs_p->boat_qgc_param1.rud_ci = rud_ci;
    strs_p->boat_qgc_param1.rud_contr_type = rudder_controller_type;
    strs_p->boat_qgc_param1.lat0 = lat0;
    strs_p->boat_qgc_param1.lon0 = lon0;
    strs_p->boat_qgc_param1.alt0 = alt0;
    strs_p->boat_qgc_param1.latT = lat_tmark;
    strs_p->boat_qgc_param1.lonT = lon_tmark;
    strs_p->boat_qgc_param1.altT = alt_tmark;
    strs_p->boat_qgc_param1.mean_wind_direction_r = mean_wind;
    orb_publish(ORB_ID(boat_qgc_param1), pubs_p->boat_qgc_param1, &(strs_p->boat_qgc_param1));

    strs_p->boat_qgc_param2.timestamp = hrt_absolute_time();
    strs_p->boat_qgc_param2.window_alpha = window_alpha;
    strs_p->boat_qgc_param2.window_apparent = window_apparent;
    strs_p->boat_qgc_param2.window_twd = window_twd;
    strs_p->boat_qgc_param2.type_of_tack = (uint16_t)tack_type;
    orb_publish(ORB_ID(boat_qgc_param2), pubs_p->boat_qgc_param2, &(strs_p->boat_qgc_param2));

    #if SIMULATION_FLAG == 1

    //----- simulation coordinates
    int32_t lat_sim;
    int32_t lon_sim;
    int32_t alt_sim;
    //lat_sim
    param_get(pointers_param_qgc.lat_sim_pointer, &lat_sim);

    //lon_sim
    param_get(pointers_param_qgc.lon_sim_pointer, &lon_sim);

    //alt_sim
    param_get(pointers_param_qgc.alt_sim_pointer, &alt_sim);

    //set lat, lon and alt to gps_filtered struct to simulate
    strs_p->gps_filtered.lat = ((double)lat_sim) / 1e7;
    strs_p->gps_filtered.lon = ((double)lon_sim) / 1e7;
    strs_p->gps_filtered.alt = alt_sim / 1e3;

    //cog_sim
    param_get(pointers_param_qgc.cog_sim_pointer, &(params_p->cog_sim));
    //convert cog in rad
    params_p->cog_sim = params_p->cog_sim * deg2rad;

    //twd_sim
    param_get(pointers_param_qgc.twd_sim_pointer, &(params_p->twd_sim));
    //convert twd_sim in rad
    params_p->twd_sim = params_p->twd_sim * deg2rad;

    //yaw_sim
    param_get(pointers_param_qgc.yaw_sim_pointer, &(params_p->yaw_sim));
    //convert yaw_sim in rad
    params_p->yaw_sim = params_p->yaw_sim * deg2rad;

    //set them in the appropriate struct to simulate heading changing
    strs_p->att.yaw = params_p->yaw_sim;

    //deva1
    param_get(pointers_param_qgc.deva1_sim_pointer, &(params_p->deva1));

    #endif
}

