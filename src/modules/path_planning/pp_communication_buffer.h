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
 * @file pp_communication_buffer.h
 *
 * Use this module to update each field in path_planning topic and publish it.
 *
 * @author Marco Tranzatto <marco.tranzatto@gmail.com>
 */

#ifndef PP_COMMUNICATION_BUFFER_H
#define PP_COMMUNICATION_BUFFER_H

#include "pp_topics_handler.h"
#include "pp_send_msg_qgc.h"
#include "pp_config.h"
#include <stdio.h>
#include <math.h>

/** @brief command a tack or a jybe */
bool cb_do_maneuver(float new_alpha_star);

/** @brief ask if an already started maneuver has been completed */
bool cb_is_maneuver_completed(void);

/** @brief copy new data from autonomous_sailing app */
void cb_new_as_data(const struct structs_topics_s *strs_p);

/** @brief publish path_planning module if it has been updated */
void cb_publish_pp_if_updated(void);

/** @brief set new X and Y coordinates in race frame */
void cb_set_race_coordinates(float x_m, float y_m);

/** @brief set a new value for alpha_star */
bool cb_set_alpha_star(float new_alpha_star);

/** @brief init module */
void cb_init(void);

/** @brief get current alpha_star value */
float cb_get_alpha_star(void);

/** @brief copy new data from remote controller*/
void cb_new_rc_data(const struct structs_topics_s *strs_p);

#endif // PP_COMMUNICATION_BUFFER_H
