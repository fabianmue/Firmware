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
 * Sails position
 *
 * ?????.
 * Default value for sails position (must be converted into degrees) 0 = max sheet out, 0.56 = max sheet in.
 *
 * @min 0 (max sheet out)
 * @max 0.56 (max sheet in)
 */
PARAM_DEFINE_FLOAT(AS_SAIL, 0.5f);


/**
 * Proportional gain.
 *
 *
 * @min 0
 * @max ?
 */
PARAM_DEFINE_FLOAT(AS_GAIN_P, 0.03f);

/**
 * Integral gain.
 *
 *
 * @min 0
 * @max ?
 */
PARAM_DEFINE_FLOAT(AS_GAIN_I, 0.0f);

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
 * Stop tack, used in guidance_module to decide whetever the tack maneuver is finisched
 *
 *
 * @min 0
 * @max ?
 */
PARAM_DEFINE_FLOAT(AS_STP_TCK, 2.0f);

/**
 * AS_WINDOW, specifies the number of samples for the moving wind average mean.
 *
 *
 * @min 1
 * @max ?
 */
PARAM_DEFINE_INT32(AS_WINDOW, 30);

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
 * Index of grid line to be set
 *
 *
 * @min 0
 * @max AS_P_TOT_G - 1
 */
//PARAM_DEFINE_INT32(AS_P_INDEX, 0);

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
 * 1 = boat should tack as soon as possibile
 *
 *
 * @min 0
 * @max 1
 */
PARAM_DEFINE_INT32(ASIM_TACK, 0);

#endif



static struct pointers_param_qgc_s{
    param_t sail_pointer;         /**< pointer to param AS_SAIL*/
    //param_t rudder_pointer;       /**< pointer to param AS_RUDDER*/

    param_t p_gain_pointer;       /**< pointer to param AS_P_GAIN*/
    param_t i_gain_pointer;       /**< pointer to param AS_I_GAIN*/

    param_t lat0_pointer;         /**< pointer to param AS_R_LAT0_E7*/
    param_t lon0_pointer;         /**< pointer to param AS_R_LON0_E7*/
    param_t alt0_pointer;         /**< pointer to param AS_R_ALT0_E3*/

    param_t stop_tack_pointer;      /**< pointer to param AS_STP_TCK*/

    param_t moving_window_pointer;/**< pointer to param AS_WINDOW*/

    param_t mean_wind_pointer;/**< pointer to param AS_MEAN_WIND_R*/

    param_t lat_tmark_pointer;         /**< pointer to param AS_T_LAT_E7*/
    param_t lon_tmark_pointer;         /**< pointer to param AS_T_LON_E7*/
    param_t alt_tmark_pointer;         /**< pointer to param AS_T_ALT_E3*/

    param_t grids_number_pointer;         /**< pointer to param AS_P_TOT_G*/
    //param_t grid_index_pointer;         /**< pointer to param AS_P_INDEX*/
    param_t grid_x_pointer;         /**< pointer to param AS_P_X_M*/
    param_t grid_add_pointer;         /**< pointer to param AS_P_ADD*/

    #if SIMULATION_FLAG == 1
    param_t lat_sim_pointer; /**< pointer to param ASIM_LAT_E7*/
    param_t lon_sim_pointer; /**< pointer to param ASIM_LON_E7*/
    param_t alt_sim_pointer; /**< pointer to param ASIM_ALt_E3*/

    param_t twd_sim_pointer; /**< pointer to param ASIM_TWD_R*/
    param_t cog_sim_pointer; /**< pointer to param ASIM_COG_R*/

    param_t tack_sim_pointer;/**< pointer to params ASIM_TACK */
    #endif
}pointers_param_qgc;


/**
* Initialize parameters.
*
*/
void param_init(struct parameters_qgc *params_p,
                struct structs_topics_s *strs_p){

    //initialize pointer to parameters
    pointers_param_qgc.sail_pointer    = param_find("AS_SAIL");
    //pointers_param_qgc.rudder_pointer  = param_find("AS_RUDDER");

    pointers_param_qgc.p_gain_pointer  = param_find("AS_GAIN_P");
    pointers_param_qgc.i_gain_pointer  = param_find("AS_GAIN_I");

    pointers_param_qgc.lat0_pointer    = param_find("AS_R_LAT0_E7");
    pointers_param_qgc.lon0_pointer    = param_find("AS_R_LON0_E7");
    pointers_param_qgc.alt0_pointer    = param_find("AS_R_ALT0_E3");

    pointers_param_qgc.stop_tack_pointer = param_find("AS_STP_TCK");

    pointers_param_qgc.moving_window_pointer = param_find("AS_WINDOW");

    pointers_param_qgc.mean_wind_pointer = param_find("AS_MEAN_WIND_R");

    pointers_param_qgc.lat_tmark_pointer    = param_find("AS_T_LAT_E7");
    pointers_param_qgc.lon_tmark_pointer    = param_find("AS_T_LON_E7");
    pointers_param_qgc.alt_tmark_pointer    = param_find("AS_T_ALT_E3");

    pointers_param_qgc.grids_number_pointer    = param_find("AS_P_TOT_G");
    //pointers_param_qgc.grid_index_pointer    = param_find("AS_P_INDEX");
    pointers_param_qgc.grid_x_pointer    = param_find("AS_P_X_M");

    pointers_param_qgc.grid_add_pointer = param_find("AS_P_ADD");

    #if SIMULATION_FLAG == 1

    pointers_param_qgc.lat_sim_pointer = param_find("ASIM_LAT_E7");
    pointers_param_qgc.lon_sim_pointer = param_find("ASIM_LON_E7");
    pointers_param_qgc.alt_sim_pointer = param_find("ASIM_ALT_E3");

    pointers_param_qgc.cog_sim_pointer = param_find("ASIM_COG_R");
    pointers_param_qgc.twd_sim_pointer = param_find("ASIM_TWD_R");

    pointers_param_qgc.tack_sim_pointer = param_find("ASIM_TACK");

    #endif

    //get parameters but do not add any grid lines at start up
    param_update(params_p, strs_p, false);

}

/** Update local copy of parameters.
 *
*/
void param_update(struct parameters_qgc *params_p,
                  struct structs_topics_s *strs_p, bool update_path_param){


    //sail_servo
    param_get(pointers_param_qgc.sail_pointer, &(params_p->sail_servo));

    //rudder_servo
    //param_get(pointers_param_qgc.rudder_pointer, &(params_p->rudder_servo));

    //p_gain
    param_get(pointers_param_qgc.p_gain_pointer, &(params_p->p_gain));


    //i_gain
    param_get(pointers_param_qgc.i_gain_pointer, &(params_p->i_gain));

    //lat0
    param_get(pointers_param_qgc.lat0_pointer, &(params_p->lat0));

    //lon0
    param_get(pointers_param_qgc.lon0_pointer, &(params_p->lon0));

    //alt0
    param_get(pointers_param_qgc.alt0_pointer, &(params_p->alt0));

    //update NED origin using API in navigation.h
    set_ref0(&(params_p->lat0), &(params_p->lon0), &(params_p->alt0));

    //stop tack value
    float tmpF = 1.0f;
    param_get(pointers_param_qgc.stop_tack_pointer, &tmpF);
    //set it in the guidance_module
    set_stop_tack(tmpF);

    //moving window
    param_get(pointers_param_qgc.moving_window_pointer, &(params_p->moving_window));

    //update window size using API in controller_data.h
    update_k(params_p->moving_window);

    //mean wind
    param_get(pointers_param_qgc.mean_wind_pointer, &(params_p->mean_wind));

    //set mean wind angle in navigation.h
    set_mean_wind_angle(params_p->mean_wind);

    //lat_tmark
    param_get(pointers_param_qgc.lat_tmark_pointer, &(params_p->lat_tmark));

    //lon_tmark
    param_get(pointers_param_qgc.lon_tmark_pointer, &(params_p->lon_tmark));

    //alt_tmark
    param_get(pointers_param_qgc.alt_tmark_pointer, &(params_p->alt_tmark));

    //set top mark position
    set_pos_top_mark(&(params_p->lat_tmark), &(params_p->lon_tmark), &(params_p->alt_tmark));

    //number of grids
    param_get(pointers_param_qgc.grids_number_pointer, &(params_p->grids_number));

    //x coordinate of current grid line
    param_get(pointers_param_qgc.grid_x_pointer, &(params_p->grids_x_m));

    //check if we have to add a new grid line
    int32_t temp;
    param_get(pointers_param_qgc.grid_add_pointer, &temp);
    if(temp > 0 && update_path_param){
        //set x coordinate of a new grid line
        set_grid(params_p->grids_x_m);
    }

    //set the new number of grid lines
    set_grids_number(params_p->grids_number);   

    #if SIMULATION_FLAG == 1

    //lat_sim
    param_get(pointers_param_qgc.lat_sim_pointer, &(params_p->lat_sim));

    //lon_sim
    param_get(pointers_param_qgc.lon_sim_pointer, &(params_p->lon_sim));

    //alt_sim
    param_get(pointers_param_qgc.alt_sim_pointer, &(params_p->alt_sim));

    //set lat, lon and alt to gps_filtered struct to simulate
    strs_p->gps_filtered.lat = ((double)params_p->lat_sim) / 1e7;
    strs_p->gps_filtered.lon = ((double)params_p->lon_sim) / 1e7;
    strs_p->gps_filtered.alt = params_p->alt_sim / 1e3;

    //cog_sim
    param_get(pointers_param_qgc.cog_sim_pointer, &(params_p->cog_sim));

    //twd_sim
    param_get(pointers_param_qgc.twd_sim_pointer, &(params_p->twd_sim));

    //tack_sim
    param_get(pointers_param_qgc.tack_sim_pointer, &(params_p->tack_sim));

    #endif
}

