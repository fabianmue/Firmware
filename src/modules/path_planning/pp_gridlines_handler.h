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
 * @file pp_gridlines_handler.h
 *
 * handler for gir lines system.
 *
 * @author Marco Tranzatto <marco.tranzatto@gmail.com>
 */

#ifndef PP_GRIDLINES_HANDLER_H
#define PP_GRIDLINES_HANDLER_H

#include <stdint.h>
#include <stdio.h>
#include "pp_topics_handler.h"

/** @brief Initialize the grid lines struct.*/
void gh_init_grids(void);

/** @brief set number of grid lines from QGroundControl*/
void gh_set_grids_number_qgc(int16_t size);

/** @brief set the x coordinate of a grid line from QGroundControl*/
void gh_set_grid_qgc(float x_m);

/** @brief based on gps position decide reference actions*/
void gh_path_planning(struct structs_topics_s *strs_p);

/** @brief set a new value for reference alpha star*/
void gh_set_alpha_star(float val);

/** @brief notify that the tack maneuver is completed*/
void gh_notify_tack_completed(void);

/** @brief re-insert the grid lines used before*/
void gh_reuse_last_grids(bool use);

/** @brief tell to the boat if it should tack or not*/
void gh_boat_should_tack(int32_t tack_now);

/** @brief tell to path planning which is the current alpha*/
void gh_set_current_alpha(float alpha_dumas);

#endif // PP_GRIDLINES_HANDLER_H
