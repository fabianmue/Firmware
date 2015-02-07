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
 * @file guidance_module.h
 *
 * Guidance module.
 * Implementation of controller(s) used to make the boat sailing autonomously.
 *
 * @author Marco Tranzatto <marco.tranzatto@gmail.com>
 */

#ifndef GUIDANCE_MODULE_H
#define GUIDANCE_MODULE_H

#include <math.h>

#include "path_planning.h"
#include "parameters.h"
#include "topics_handler.h"
#include "controller_data.h"
#include "simulation_utility.h"
#include "mpc_boatTack.h"

//log messages to QGroundControl
#include "send_msg_qgc.h"

#define RUDDER_SATURATION 0.9f /// 0.9f = most left rudder position, -0.9f, most right rudder position
#define RUDDER_45_LEFT 0.85f /// rudder at 45 deg and the boat steers on the left

#define SAIL_SATURATION 0.56f  /// 0.56f = sails fully closed; 0.0f = sails fully opened
#define SAIL_FULLY_OPENED 0.0f
#define SAIL_20 0.56f   ///sails are opened at 20 deg

/** @brief Implement next control action*/
void guidance_module(struct reference_actions_s *ref_act_p,
                     const struct parameters_qgc *param_qgc_p,
                     struct structs_topics_s *strs_p);


/** @brief Set data of the PI which controls rudder*/
void set_rudder_data(float p, float i, float cp,
                     float ci, int32_t rudder_controller_type, float kaw,
                     float alpha_rudder_x1_r, float alpha_rudder_x2_r, float rud_cmd_45_left);

/** @brief set which kind of tack maneuver should be performed */
void set_tack_data(uint16_t tack_type, float alpha_min_stop_tack_r);

/** @brief set data of the sail controller*/
void set_sail_data(float sail_closed_cmd, float alpha_sail_closed_r, float alpha_sail_opened_r);

/** @brief set lqr gain for lqr tack maneuver*/
void set_lqr_gain(float lqr_k1, float lqr_k2, float lqr_k3);

/** @brief set MPC cost function, lower and upper bound */
void set_mpc_data(float h[4], float lb[2], float ub[2], float h_final[3][3]);

#endif //GUIDANCE_MODULE_H
