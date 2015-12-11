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
#include "pp_gridlines_handler.h"
#include "pp_special_cmd_as.h"
#include <stdio.h>
#include <math.h>
#include <stdint.h>

//definition of hauls
#define HAUL_PORT 0
#define HAUL_STARBOARD 1

// ----- usefull functions to control the boat

/** @brief Set the failsafe-flag in path planning-topic */
bool cb_set_failsafe(bool state);

/** @brief return true, if the boat is in autonomous mode */
bool cb_is_autonomous_mode(void);

/** @brief store the current NED-Position in the pathplanning-topic */
bool cb_new_position(float north, float east);

/** @brief store the current Heading known by the Navigator in the pathplanning-topic */
bool cb_new_heading(float heading);

/** @brief Return the heading of the boat in compass frame */
float cb_get_heading(void);

/** @brief Store the current reference Heading in the pathplanning-topic */
bool cb_new_refheading(float ref_heading);

/** @brief store the current Wind Direction known by the Navigator in the pathplanning-topic */
bool cb_new_wind(float wind);

/** @brief Send the new target position to the Pathplanning Topic */
// bool cb_new_target(float north, float east);

/** @brief Store the current obstacle position in the pathplanning-topic */
// bool cb_new_obstacle(float o_north, float o_east);

/** @brief Send the Number of the next Waypoint to QGround Control */
// bool cb_new_targetnum(uint8_t tar_num);

/** @brief command a tack or a jybe */
bool cb_do_maneuver(float new_alpha_star);

/** @brief ask if an already started maneuver has been completed */
bool cb_is_maneuver_completed(void);

/** @brief set a new value for alpha_star */
bool cb_set_alpha_star(float new_alpha_star);

/** @brief get current alpha_star value */
float cb_get_alpha_star(void);

/** @brief get the current haul of the boat */
uint8_t cb_get_haul(void);

/** @brief get alpha angle sent by autonomous_sailing app */
float cb_get_alpha(void);

/** @brief get true wind information */
void cb_get_tw_info(float *twd_p, float *tws_p);

/** @brief tell the boat to tack */
bool cb_tack_now(void);

/** @brief set if you want to use a fixed TWD or the TWD by a moving average filter */
void cb_use_fixed_twd(bool use_fixed_twd);

// ----- end usefull functions

// ----- functions used by other modules, do not change or use them

/** @brief copy new data from autonomous_sailing app */
void cb_new_as_data(int boat_guidance_debug_sub);

/** @brief publish path_planning module if it has been updated */
void pp_cb_publish_if_updated(void);

/** @brief init module */
void pp_cb_init(void);

/** @brief copy new data from remote controller*/
void cb_new_rc_data(const struct pp_structs_topics_s *strs_p);

#if USE_GRID_LINES == 1

/** @brief set alpha_star velocity if using grid lines*/
void cb_set_alpha_star_vel(float vel_r_s);

/** @brief notify to communication_buffer to start sailing downwind*/
void cb_reached_last_griline(void);

/** @brief set alpha_star during downwind course after last grid line*/
void cb_set_downwind_alpha_star(float alpha_star);

#endif //USE_GRID_LINES == 1

#endif // PP_COMMUNICATION_BUFFER_H
