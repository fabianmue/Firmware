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

/**
 * Stop tack, used in guidance_module to decide whetever the tack maneuver is finished using yaw.
 * Offset in degrees.
 *
 *
 * @min -180
 * @max 180
 */
PARAM_DEFINE_FLOAT(AS_SPTC_Y_D, 60.0f);

/**
 * Stop tack, used in guidance_module to decide whetever the tack maneuver is finished using roll angle
 *
 *
 * @min 0
 * @max ?
 */
PARAM_DEFINE_FLOAT(AS_SPTC_R, 2.0f);

/**
 * Reference angle with respect true wind [rad]
 *
 * Use Dumas'convention.
 *
 * @min -pi
 * @max pi
 */
PARAM_DEFINE_FLOAT(AS_ALST_ANG, 0.5f);

/**
 * 1 if you wish to use the alpha star specified by AS_ALSTR, 0 otherwise
 *
 * @min 0
 * @max 1
 */
PARAM_DEFINE_INT32(AS_ALST_SET, 1);

/**
 * Sails position
 *
 * Default value for sails position. 0 = max sheet out, 0.56 = max sheet in.
 * If a negative value is set, then the sail control is in charge to control sails.
 *
 * @min 0 (max sheet out)
 * @max 0.56 (max sheet in)
 */
PARAM_DEFINE_FLOAT(AS_SAIL, -1.0f);

/**
 * Number of possibile sail positions.
 * Used in guidance_module to set sail position.
 *
 * @min 1
 * @max ?
 */
PARAM_DEFINE_INT32(AS_N_SPOS, 4);

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
 * Constant used in conditionl integral to adjust den.
 *
 *
 * @min 0
 * @max ?
 */
PARAM_DEFINE_FLOAT(AS_RUD_C, 1.0f);

/**
 * 1 if you wish to use a condition integral PI. 0 otherwise
 *
 * @min 0
 * @max 1
 */
PARAM_DEFINE_INT32(AS_RUD_CONDPI, 1);


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
PARAM_DEFINE_INT32(AS_WIN_AL, 30);

/**
 * AS_WIN_APP, specifies the number of samples for the moving average of apparent wind direction.
 *
 *
 * @min 1
 * @max ?
 */
PARAM_DEFINE_INT32(AS_WIN_APP, 3);

/**
 * AS_WIN_TWD, specifies the number of samples for the moving average of true wind direction.
 *
 *
 * @min 1
 * @max ?
 */
PARAM_DEFINE_INT32(AS_WIN_TWD, 3);

/**
 * AS_MEAN_WIND, specifies the mean wind direction [rad], in [-pi, pi].
 * Positive on the right (going from North to East), negative on the left (going from North to West).
 *
 *
 * @min -pi
 * @max pi
 */
PARAM_DEFINE_FLOAT(AS_MEAN_WIND_R, 0.0f);

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
PARAM_DEFINE_FLOAT(AS_P_X_M, 0);

/**
 * 1 if you want to add a new grid line
 *
 *
 * @min 0
 * @max 1
 */
PARAM_DEFINE_INT32(AS_P_ADD, 0);

//------------------------------------- Parameters for starting optimal path following ----

/**
 * 1 if you want to start following optimal path trajectory, 0 or -1 if you want to insert each grid line
 *
 *
 * @min -1
 * @max 1
 */
PARAM_DEFINE_INT32(ASP_START, 0);

/**
 * Absolute value of reference angle with respect true wind [rad]
 *
 * Use Dumas'convention.
 *
 * @min 0
 * @max pi
 */
PARAM_DEFINE_FLOAT(ASP_ALST_ANG, 0.5f);

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
 * Simulated Course over ground, in rads, sign opposite to Dumas convention.
 *
 *
 * @min -pi
 * @max pi
 */
PARAM_DEFINE_FLOAT(ASIM_COG_R, 0.0f);

/**
 * Simulated true wind direction, in rads, sign opposite to Dumas convention.
 *
 *
 * @min -pi
 * @max pi
 */
PARAM_DEFINE_FLOAT(ASIM_TWD_R, 0.0f);

/**
 * Simulated yaw, in rads, sign opposite to Dumas convention.
 *
 *
 * @min -pi
 * @max pi
 */
PARAM_DEFINE_FLOAT(ASIM_YAW_R, 0.0f);

#endif



static struct pointers_param_qgc_s{

    param_t alpha_star_pointer;         /**< pointer to param AS_ALST_ANG*/
    param_t use_alpha_star_pointer;         /**< pointer to param AS_ALST_SET*/


    param_t sail_pointer;         /**< pointer to param AS_SAIL*/

    param_t sail_positions_pointer;       /**< pointer to param AS_N_SPOS*/

    param_t rud_p_gain_pointer;       /**< pointer to param AS_RUD_P*/
    param_t rud_i_gain_pointer;       /**< pointer to param AS_RUD_I*/
    param_t rud_c_pointer;       /**< pointer to param AS_RUD_C*/
    param_t rud_conditional_pi_pointer;       /**< pointer to param AS_RUD_CONDPI*/

    param_t lat0_pointer;         /**< pointer to param AS_R_LAT0_E7*/
    param_t lon0_pointer;         /**< pointer to param AS_R_LON0_E7*/
    param_t alt0_pointer;         /**< pointer to param AS_R_ALT0_E3*/

    param_t stop_tack_roll_pointer;      /**< pointer to param AS_SPTC_R*/
    param_t stop_tack_yaw_pointer;      /**< pointer to param AS_SPTC_Y_D */

    param_t moving_alpha_window_pointer;/**< pointer to param AS_WIN_AL*/
    param_t moving_apparent_window_pointer;/**< pointer to param AS_WIN_APP*/
    param_t moving_twd_window_pointer;/**< pointer to param AS_WIN_TWD*/

    param_t mean_wind_pointer;/**< pointer to param AS_MEAN_WIND_R*/

    param_t lat_tmark_pointer;         /**< pointer to param AS_T_LAT_E7*/
    param_t lon_tmark_pointer;         /**< pointer to param AS_T_LON_E7*/
    param_t alt_tmark_pointer;         /**< pointer to param AS_T_ALT_E3*/

    param_t grids_number_pointer;         /**< pointer to param AS_P_TOT_G*/
    param_t grid_x_pointer;         /**< pointer to param AS_P_X_M*/
    param_t grid_add_pointer;         /**< pointer to param AS_P_ADD*/

    //-- params for optimal path
    param_t start_path_following_pointer;         /**< pointer to param ASP_START*/
    param_t abs_alpha_star_pointer;         /**< pointer to param ASP_ALST_ANG*/

    //-- simulation params

    #if SIMULATION_FLAG == 1
    param_t lat_sim_pointer; /**< pointer to param ASIM_LAT_E7*/
    param_t lon_sim_pointer; /**< pointer to param ASIM_LON_E7*/
    param_t alt_sim_pointer; /**< pointer to param ASIM_ALt_E3*/

    param_t twd_sim_pointer; /**< pointer to param ASIM_TWD_R*/
    param_t cog_sim_pointer; /**< pointer to param ASIM_COG_R*/


    param_t yaw_sim_pointer; /**< pointer to param ASIM_YAW_R*/
    #endif
}pointers_param_qgc;


/**
* Initialize parameters.
*
*/
void param_init(struct parameters_qgc *params_p,
                struct structs_topics_s *strs_p){

    //initialize pointer to parameters
    pointers_param_qgc.alpha_star_pointer    = param_find("AS_ALST_ANG");
    pointers_param_qgc.use_alpha_star_pointer    = param_find("AS_ALST_SET");

    pointers_param_qgc.sail_pointer    = param_find("AS_SAIL");

    pointers_param_qgc.sail_positions_pointer  = param_find("AS_N_SPOS");

    pointers_param_qgc.rud_p_gain_pointer  = param_find("AS_RUD_P");
    pointers_param_qgc.rud_i_gain_pointer  = param_find("AS_RUD_I");
    pointers_param_qgc.rud_c_pointer  = param_find("AS_RUD_C");
    pointers_param_qgc.rud_conditional_pi_pointer  = param_find("AS_RUD_CONDPI");

    pointers_param_qgc.lat0_pointer    = param_find("AS_R_LAT0_E7");
    pointers_param_qgc.lon0_pointer    = param_find("AS_R_LON0_E7");
    pointers_param_qgc.alt0_pointer    = param_find("AS_R_ALT0_E3");

    pointers_param_qgc.stop_tack_roll_pointer = param_find("AS_SPTC_R");
    pointers_param_qgc.stop_tack_yaw_pointer = param_find("AS_SPTC_Y_D");

    pointers_param_qgc.moving_alpha_window_pointer = param_find("AS_WIN_AL");
    pointers_param_qgc.moving_apparent_window_pointer = param_find("AS_WIN_APP");
    pointers_param_qgc.moving_twd_window_pointer = param_find("AS_WIN_TWD");

    pointers_param_qgc.mean_wind_pointer = param_find("AS_MEAN_WIND_R");

    pointers_param_qgc.lat_tmark_pointer    = param_find("AS_T_LAT_E7");
    pointers_param_qgc.lon_tmark_pointer    = param_find("AS_T_LON_E7");
    pointers_param_qgc.alt_tmark_pointer    = param_find("AS_T_ALT_E3");

    pointers_param_qgc.grids_number_pointer    = param_find("AS_P_TOT_G");
    pointers_param_qgc.grid_x_pointer    = param_find("AS_P_X_M");

    pointers_param_qgc.grid_add_pointer = param_find("AS_P_ADD");

    //-- optimal path following
    pointers_param_qgc.start_path_following_pointer = param_find("ASP_START");
    pointers_param_qgc.abs_alpha_star_pointer = param_find("ASP_ALST_ANG");

    #if SIMULATION_FLAG == 1

    pointers_param_qgc.lat_sim_pointer = param_find("ASIM_LAT_E7");
    pointers_param_qgc.lon_sim_pointer = param_find("ASIM_LON_E7");
    pointers_param_qgc.alt_sim_pointer = param_find("ASIM_ALT_E3");

    pointers_param_qgc.cog_sim_pointer = param_find("ASIM_COG_R");
    pointers_param_qgc.twd_sim_pointer = param_find("ASIM_TWD_R");

    pointers_param_qgc.yaw_sim_pointer = param_find("ASIM_YAW_R");

    #endif

    //get parameters but do not add any grid lines at start up
    param_update(params_p, strs_p, false);

}

/** Update local copy of parameters.
 *
*/
void param_update(struct parameters_qgc *params_p,
                  struct structs_topics_s *strs_p, bool update_path_param){

    //----- alpha star
    float alpha_tmp;
    int32_t set_alpha;
    param_get(pointers_param_qgc.alpha_star_pointer, &alpha_tmp);
    param_get(pointers_param_qgc.use_alpha_star_pointer, &set_alpha);
    if(set_alpha)
        set_alpha_star(alpha_tmp);

    //----- sail_servo
    param_get(pointers_param_qgc.sail_pointer, &(params_p->sail_servo));

    //----- number of possibile sail positions
    int32_t positions_temp;
    param_get(pointers_param_qgc.sail_positions_pointer, &positions_temp);
    set_sail_positions(positions_temp);

    //----- param for rudder PI
    float rud_p;
    float rud_i;
    float rud_c;
    int32_t use_cond_pi;

    param_get(pointers_param_qgc.rud_p_gain_pointer, &rud_p);
    param_get(pointers_param_qgc.rud_i_gain_pointer, &rud_i);
    param_get(pointers_param_qgc.rud_c_pointer, &rud_c);
    param_get(pointers_param_qgc.rud_conditional_pi_pointer, &use_cond_pi);

    set_pi_rudder_data(rud_p, rud_i, rud_c, use_cond_pi);

    //p_gain for sail P
    //param_get(pointers_param_qgc.sai_p_gain_pointer, &(params_p->sail_p_gain));

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

    //----- stop tack values
    float roll_stop;
    float yaw_stop;
    param_get(pointers_param_qgc.stop_tack_roll_pointer, &roll_stop);
    param_get(pointers_param_qgc.stop_tack_yaw_pointer, &yaw_stop);
    //set it in the guidance_module
    set_stop_tack(roll_stop, yaw_stop);

    //----- moving windows
    uint16_t moving_window;
    param_get(pointers_param_qgc.moving_alpha_window_pointer, &moving_window);
    //update window size using API in controller_data.h
    update_k(moving_window);

    param_get(pointers_param_qgc.moving_apparent_window_pointer, &moving_window);
    //update window size using API in controller_data.h
    update_k_app(moving_window);

    param_get(pointers_param_qgc.moving_twd_window_pointer, &moving_window);
    //update window size using API in controller_data.h
    update_k_twd(moving_window);

    //----- mean wind
    float mean_wind;
    param_get(pointers_param_qgc.mean_wind_pointer, &mean_wind);
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

    //-- params for optimal path following
    //use these params only if update_path_param is true
    if(update_path_param){
        float abs_alpha_star;
        int32_t start_following;

        param_get(pointers_param_qgc.abs_alpha_star_pointer, &abs_alpha_star);
        param_get(pointers_param_qgc.start_path_following_pointer, &start_following);
        //make sure abs_alpha_star is positive
        abs_alpha_star = (abs_alpha_star > 0) ? abs_alpha_star : -abs_alpha_star;

        //pass these two values to path_planning module
        start_following_optimal_path(start_following, abs_alpha_star);
    }

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

    //twd_sim
    param_get(pointers_param_qgc.twd_sim_pointer, &(params_p->twd_sim));

    //yaw_sim
    param_get(pointers_param_qgc.yaw_sim_pointer, &(params_p->yaw_sim));

    //set them in the appropriate struct to simulate heading changing
    strs_p->att.yaw = params_p->yaw_sim;

    #endif
}

