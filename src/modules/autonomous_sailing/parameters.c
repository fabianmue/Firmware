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
* Initialize parameters.
*
*/
void param_init(struct pointers_param_qgc *pointers_p,
                struct parameters_qgc *params_p){

    //initialize pointer to parameters
    pointers_p->sail_pointer    = param_find("AS_SAIL");
    pointers_p->rudder_pointer  = param_find("AS_RUDDER");

    pointers_p->p_gain_pointer  = param_find("AS_P_GAIN");
    pointers_p->i_gain_pointer  = param_find("AS_I_GAIN");

    pointers_p->lat0_pointer    = param_find("AS_LAT0");
    pointers_p->lon0_pointer    = param_find("AS_LON0");
    pointers_p->alt0_pointer    = param_find("AS_ALT0");

    pointers_p->epsilon_pointer = param_find("AS_EPSI");

    pointers_p->moving_window_pointer = param_find("AS_WIN");

    //get parameters
    param_get(pointers_p->sail_pointer, &(params_p->sail_servo));
    param_get(pointers_p->rudder_pointer, &(params_p->rudder_servo));

    param_get(pointers_p->p_gain_pointer, &(params_p->p_gain));
    param_get(pointers_p->i_gain_pointer, &(params_p->i_gain));

    param_get(pointers_p->lat0_pointer, &(params_p->lat0));
    param_get(pointers_p->lon0_pointer, &(params_p->lon0));
    param_get(pointers_p->alt0_pointer, &(params_p->alt0));

    param_get(pointers_p->epsilon_pointer, &(params_p->epsilon));

    param_get(pointers_p->moving_window_pointer, &(params_p->moving_window));

    //update window size
    update_k(params_p->moving_window);

}

/** Check if any paramter has been updated, if so take appropriate actions
 *
*/
void param_check_update(struct pointers_param_qgc *pointers_p,
                        struct parameters_qgc *params_p){

    float app_f;
    int32_t app_i;

    //check sail_servo
    param_get(pointers_p->sail_pointer, &app_f);
    if(params_p->sail_servo != app_f){
        params_p->sail_servo = app_f;
    }

    //check rudder_servo
    param_get(pointers_p->rudder_pointer, &app_f);
    if(params_p->rudder_servo != app_f){
        params_p->rudder_servo = app_f;
    }

    //check p_gain
    param_get(pointers_p->p_gain_pointer, &app_f);
    if(params_p->p_gain != app_f){
        params_p->p_gain = app_f;
    }

    //check i_gain
    param_get(pointers_p->i_gain_pointer, &app_f);
    if(params_p->i_gain != app_f){
        params_p->i_gain = app_f;
    }

    //check lat0
    param_get(pointers_p->lat0_pointer, &app_i);
    if(params_p->lat0 != app_i){
        params_p->lat0 = app_i;
        //update NED origin
        set_ref0(&(params_p->lat0), &(params_p->lon0), &(params_p->alt0));
    }

    //check lon0
    param_get(pointers_p->lon0_pointer, &app_i);
    if(params_p->lon0 != app_i){
        params_p->lon0 = app_i;
        //update NED origin
        set_ref0(&(params_p->lat0), &(params_p->lon0), &(params_p->alt0));
    }

    //check alt0
    param_get(pointers_p->alt0_pointer, &app_i);
    if(params_p->alt0 != app_i){
        params_p->alt0 = app_i;
        //update NED origin
        set_ref0(&(params_p->lat0), &(params_p->lon0), &(params_p->alt0));
    }

    //check epsilon
    param_get(pointers_p->epsilon_pointer, &app_f);
    if(params_p->epsilon != app_f){
        params_p->epsilon = app_f;
    }

    //check moving window
    param_get(pointers_p->moving_window_pointer, &app_i);
    if(params_p->moving_window != app_i){
        params_p->moving_window = app_i;

        //update window size
        update_k(params_p->moving_window);
    }
}

