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

//log messages to QGroundControl
#include "send_msg_qgc.h"

#define RUDDER_SATURATION 0.9f
#define SAIL_SATURATION 0.56f

/** @brief Implement next control action*/
void guidance_module(struct reference_actions_s *ref_act_p,
                     const struct parameters_qgc *param_qgc_p,
                     struct structs_topics_s *strs_p);

/** @brief Set the stop value to see if the tack maneuver is completed*/
void set_stop_tack(float roll_stop, float yaw_stop);

/** @brief Set the number of positions the sail can be at*/
void set_sail_positions(int32_t num);

/** @brief Set data of the PI which controls rudder*/
void set_pi_rudder_data(float p, float i, float c, int32_t use_conditional, float kaw);

#endif //GUIDANCE_MODULE_H
