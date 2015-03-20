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
#include "pp_send_msg_qgc.h"
#include "pp_communication_buffer.h"
#include "pp_navigation_module.h"

/** @brief Initialize the grid lines struct.*/
void gh_init_grids(void);

/** @brief set number of grid lines from QGroundControl*/
void gh_set_grids_number_qgc(int16_t size);

/** @brief set the ray of a grid line from QGroundControl*/
void gh_set_grid_qgc(float ray_m);

/** @brief based on gps position decide reference actions*/
void gh_gridlines_handler(void);

/** @brief re-insert the grid lines used before*/
void gh_reuse_last_grids(bool use);

/** @brief tell to path planning which is the current alpha*/
void gh_set_current_alpha(float alpha_dumas);

/** @brief get the next grid line to reach, if any*/
bool gh_get_next_gridline(float *next_grid_p);

#endif // PP_GRIDLINES_HANDLER_H
