/****************************************************************************
 *
 *   Copyright (C) 2012 PX4 Development Team. All rights reserved.
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
 * @file boat_qgc_param.h
 *
 * Useful parameters set by the user during sailing.
 *
 */

#ifndef BOAT_QGC_PARAM_H
#define BOAT_QGC_PARAM_H

#include <stdint.h>
#include "../uORB.h"

struct boat_qgc_param1_s {
    uint64_t timestamp;
    float rud_p;            /// P constant of the PI controller of the rudder
    float rud_i;            /// I constant of the PI controller of the rudder
    float rud_kaw;          /// Kaw constant of the PI controller of the rudder
    float rud_cp;           /// CP constant of the PI controller of the rudder
    float rud_ci;           /// Ci constant of the PI controller of the rudder
    uint8_t rud_contr_type; /// Type of controller for the rudder
    int32_t lat0;           /// Latitude in 1E-7 degrees of the NED system origin
    int32_t lon0;           /// Longitude in 1E-7 degrees of the NED system origin
    int32_t alt0;           /// Altitude in 1E-3 meters (millimeters) above MSL of the NED system origin
    int32_t latT;           /// Latitude in 1E-7 degrees of the top mark buoy
    int32_t lonT;           /// Longitude in 1E-7 degrees of the top mark buoy
    int32_t altT;           /// Altitude in 1E-3 meters (millimeters) above MSL of the top mark buoy
    float mean_wind_direction_r;    /// Mean wind direction used to set X-axis direction of the race frame
};

struct boat_qgc_param2_s {
    uint64_t timestamp;
    uint16_t window_alpha;      /// Window size of moving average filter of alpha
    uint16_t window_apparent;   /// Window size of moving average filter of apparent wind
    uint16_t window_twd;        /// Window size of moving average filter of twd
    uint16_t type_of_tack;      ///type of tack set by AS_TY_TCK parameter
};

ORB_DECLARE(boat_qgc_param1);
ORB_DECLARE(boat_qgc_param2);

#endif // BOAT_QGC_PARAM_H
