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
 * Default heading angle w.r.t. relative wind, in degrees.
 *
 *
 * @min -90
 * @max 90
 */
PARAM_DEFINE_FLOAT(AS_RUDDER, 30.0f);

/**
 * Proportional gain.
 *
 *
 * @min 0
 * @max ?
 */
PARAM_DEFINE_FLOAT(AS_P_GAIN, 0.03f);

/**
 * Integral gain.
 *
 *
 * @min 0
 * @max ?
 */
PARAM_DEFINE_FLOAT(AS_I_GAIN, 0.0f);

/**
 * Latitude of origin of NED system, in degrees * E7.
 *
 *
 * @min -900000000
 * @max 900000000
 */
PARAM_DEFINE_INT32(AS_LAT0, 473494820);

/**
 * Longitude of origin of NED system, in degrees * E7.
 *
 *
 * @min -1800000000
 * @max 1800000000
 */
PARAM_DEFINE_INT32(AS_LON0, 85605120);

/**
 * Altitude of origin of NED system, in millimeters.
 *
 *
 * @min 0
 * @max ?
 */
PARAM_DEFINE_INT32(AS_ALT0, 406);

/**
 * Epsilon, specifies when the next target could be considered reached, in meters.
 *
 *
 * @min 0
 * @max ?
 */
PARAM_DEFINE_FLOAT(AS_EPSI, 2.0f);

/**
 * AS_WIN, specifies the number of samples for the moving wind average mean.
 *
 *
 * @min 1
 * @max ?
 */
PARAM_DEFINE_INT32(AS_WIN, 10);



#ifdef SIMULATION_FLAG

//---------------------------------------- Simulation variables --------------------

/**
 * Simulated Latitude, in degrees * E7.
 *
 *
 * @min -900000000
 * @max 900000000
 */
PARAM_DEFINE_INT32(AS_LAT_SIM, 473494820);

/**
 * Simulated Longitude, in degrees * E7.
 *
 *
 * @min -1800000000
 * @max 1800000000
 */
PARAM_DEFINE_INT32(AS_LON_SIM, 85605120);

/**
 * Simulated Altitude, in millimeters.
 *
 *
 * @min 0
 * @max ?
 */
PARAM_DEFINE_INT32(AS_ALT_SIM, 406);

/**
 * Simulated Course over ground, in rads, sign opposite to Dumas convention.
 *
 *
 * @min -pi
 * @max pi
 */
PARAM_DEFINE_FLOAT(AS_COG_SIM, 0.0f);

/**
 * Simulated true wind direction, in rads, sign opposite to Dumas convention.
 *
 *
 * @min -pi
 * @max pi
 */
PARAM_DEFINE_FLOAT(AS_TWD_SIM, 0.0f);

/**
 * 1 = boat should tack as soon as possibile
 *
 *
 * @min -pi
 * @max pi
 */
PARAM_DEFINE_INT32(AS_TCK_SIM, 0);

#endif



static struct pointers_param_qgc_s{
    param_t sail_pointer;         /**< pointer to param AS_SAIL*/
    param_t rudder_pointer;       /**< pointer to param AS_RUDDER*/

    param_t p_gain_pointer;       /**< pointer to param AS_P_GAIN*/
    param_t i_gain_pointer;       /**< pointer to param AS_I_GAIN*/

    param_t lat0_pointer;         /**< pointer to param AS_LAT0*/
    param_t lon0_pointer;         /**< pointer to param AS_LON0*/
    param_t alt0_pointer;         /**< pointer to param AS_ALT0*/

    param_t epsilon_pointer;      /**< pointer to param AS_EPSI*/

    param_t moving_window_pointer;/**< pointer to param AS_WIN*/

    #ifdef SIMULATION_FLAG
    param_t lat_sim_pointer; /**< pointer to param AS_LATS*/
    param_t lon_sim_pointer; /**< pointer to param AS_LONS*/
    param_t alt_sim_pointer; /**< pointer to param AS_ALTS*/

    param_t twd_sim_pointer; /**< pointer to param AS_TWDS*/
    param_t cog_sim_pointer; /**< pointer to param AS_COGS*/

    param_t tack_sim_pointer;/**< pointer to params AS_TCKS */
    #endif
}pointers_param_qgc;


/**
* Initialize parameters.
*
*/
void param_init(struct parameters_qgc *params_p){

    //initialize pointer to parameters
    pointers_param_qgc.sail_pointer    = param_find("AS_SAIL");
    pointers_param_qgc.rudder_pointer  = param_find("AS_RUDDER");

    pointers_param_qgc.p_gain_pointer  = param_find("AS_P_GAIN");
    pointers_param_qgc.i_gain_pointer  = param_find("AS_I_GAIN");

    pointers_param_qgc.lat0_pointer    = param_find("AS_LAT0");
    pointers_param_qgc.lon0_pointer    = param_find("AS_LON0");
    pointers_param_qgc.alt0_pointer    = param_find("AS_ALT0");

    pointers_param_qgc.epsilon_pointer = param_find("AS_EPSI");

    pointers_param_qgc.moving_window_pointer = param_find("AS_WIN");

    #ifdef SIMULATION_FLAG

    pointers_param_qgc.lat_sim_pointer = param_find("AS_LAT_SIM");
    pointers_param_qgc.lon_sim_pointer = param_find("AS_LON_SIM");
    pointers_param_qgc.alt_sim_pointer = param_find("AS_ALT_SIM");

    pointers_param_qgc.cog_sim_pointer = param_find("AS_COG_SIM");
    pointers_param_qgc.twd_sim_pointer = param_find("AS_TWD_SIM");

    pointers_param_qgc.tack_sim_pointer = param_find("AS_TCK_SIM");

    #endif

    //get parameters
    param_update(params_p);

}

/** Update local copy of parameters.
 *
*/
void param_update(struct parameters_qgc *params_p){


    //sail_servo
    param_get(pointers_param_qgc.sail_pointer, &(params_p->sail_servo));

    //rudder_servo
    param_get(pointers_param_qgc.rudder_pointer, &(params_p->rudder_servo));

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

    //update NED origin
    set_ref0(&(params_p->lat0), &(params_p->lon0), &(params_p->alt0));

    //epsilon
    param_get(pointers_param_qgc.epsilon_pointer, &(params_p->epsilon));

    //moving window
    param_get(pointers_param_qgc.moving_window_pointer, &(params_p->moving_window));

    //update window size
    update_k(params_p->moving_window);

    #ifdef SIMULATION_FLAG

    //lat_sim
    param_get(pointers_param_qgc.lat_sim_pointer, &(params_p->lat_sim));

    //lon_sim
    param_get(pointers_param_qgc.lon_sim_pointer, &(params_p->lon_sim));

    //alt_sim
    param_get(pointers_param_qgc.alt_sim_pointer, &(params_p->alt_sim));

    //cog_sim
    param_get(pointers_param_qgc.cog_sim_pointer, &(params_p->cog_sim));

    //twd_sim
    param_get(pointers_param_qgc.twd_sim_pointer, &(params_p->twd_sim));

    //tack_sim
    param_get(pointers_param_qgc.tack_sim_pointer, &(params_p->tack_sim));

    #endif
}

