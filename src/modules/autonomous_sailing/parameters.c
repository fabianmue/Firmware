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

#include <systemlib/param/param.h>

/*
 * Define the QGroundControl parameters here:
 * Warning: name can not be too long!!!
 */

struct parameters_qgc{
    float rudder_servo;
    float sail_servo;

    float p_gain;
    float i_gain;

    int32_t lat0;
    int32_t lon0;
    int32_t alt0;

    float epsilon;

    uint16_t moving_window;
};

struct pointers_param_qgc{
    param_t sail_pointer;         /**< pointer to param AS_SAIL*/
    param_t rudder_pointer;       /**< pointer to param AS_RUDDER*/

    param_t p_gain_pointer;       /**< pointer to param AS_P_GAIN*/
    param_t i_gain_pointer;       /**< pointer to param AS_I_GAIN*/

    param_t lat0_pointer;         /**< pointer to param AS_LAT0*/
    param_t lon0_pointer;         /**< pointer to param AS_LON0*/
    param_t alt0_pointer;         /**< pointer to param AS_ALT0*/

    param_t epsilon_pointer;      /**< pointer to param AS_EPSI*/

    param_t moving_window_pointer;/**< pointer to param AS_WIN*/
};


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
PARAM_DEFINE_INT32(AS_LAT0, 85605120);

/**
 * Longitude of origin of NED system, in degrees * E7.
 *
 *
 * @min -180
 * @max 180
 */
PARAM_DEFINE_INT32(AS_LON0, 473494820);

/**
 * Altitude of origin of NED system, in millimeters.
 *
 *
 * @min 0
 * @max ?
 */
PARAM_DEFINE_INT32(AS_ALT0, 0);

/**
 * Epsilon, specifies when the next target could be considered reached, in meters.
 *
 *
 * @min 0
 * @max ?
 */
PARAM_DEFINE_FLOAT(AS_EPSI, 5.0f);

/**
 * AS_WIN, specifies the number of samples for the moving wind average mean.
 *
 *
 * @min 1
 * @max ?
 */
PARAM_DEFINE_INT32(AS_WIN, 10);

