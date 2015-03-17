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
    uint16_t window_alpha;      /// Window size of moving average filter of alpha
    uint16_t window_twd;        /// Window size of moving average filter of twd
    float delta1;               /// Delta1 used to define the band in optimal tack maneuver
    float delta2;               /// Delta2 used to define the band in optimal tack maneuver
    uint16_t use_fixed_twd;     /// True if a fixed twd is used to compute alpha angle
    float p_tack_kp;            /// kp value for P controller during tack
    float p_tack_cp;            /// cp value for P controller during tack
};

struct boat_qgc_param2_s {
    uint64_t timestamp;
    int32_t lat0;           /// Latitude in 1E-7 degrees of the NED system origin
    int32_t lon0;           /// Longitude in 1E-7 degrees of the NED system origin
    int32_t alt0;           /// Altitude in 1E-3 meters (millimeters) above MSL of the NED system origin
    int32_t latT;           /// Latitude in 1E-7 degrees of the top mark buoy
    int32_t lonT;           /// Longitude in 1E-7 degrees of the top mark buoy
    int32_t altT;           /// Altitude in 1E-3 meters (millimeters) above MSL of the top mark buoy
    float mean_wind_direction_r;    /// Mean wind direction used to set X-axis direction of the race frame
};

struct boat_qgc_param3_s{
    uint64_t timestamp;
    int32_t lqr_sampl_time_us; ///sampling time of the LQR controller, in uSec
    int32_t mpc_sampl_time_us; ///sampling time of the MPC controller, in uSec
    float mpc_a11;              ////A(1,1) value (model NOT extended), MPC model
    float mpc_a12;              ////A(1,2) value (model NOT extended), MPC model
    float mpc_a21;              ////A(2,1) value (model NOT extended), MPC model
    float mpc_a22;              ////A(2,2) value (model NOT extended), MPC model
    float mpc_b1;              ////B(1) value (model NOT extended), MPC model
    float mpc_b2;              ////B(2) value (model NOT extended), MPC model
    uint16_t window_alpha_tack;      /// Window size of moving average filter of alpha during tack
    uint16_t window_twd_tack;      /// Window size of moving average filter of the TWD during tack
    uint16_t pred_horizon_steps;      /// Steps of the prediction horizon
    int32_t type_of_tack;      /// Type of tack set by AS_TY_TCK parameter
    int32_t use_only_yaw_man;      /// Value of AS_TCK_USE_Y
};

struct essc_log_s {
	uint64_t timestamp;
	float k; 				/**< Stepsize between to consecutive sail-angle changes [rad] */
    uint16_t windowsize;    /**< Size of the window for averaging the speed */
    float period; 			/**< Time between two consecutive changes in sail-angle [us] */
};

ORB_DECLARE(boat_qgc_param1);
ORB_DECLARE(boat_qgc_param2);
ORB_DECLARE(boat_qgc_param3);
ORB_DECLARE(essc_log);

#endif // BOAT_QGC_PARAM_H
