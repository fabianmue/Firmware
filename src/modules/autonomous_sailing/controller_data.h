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
 * @file controller_data.h
 *
 * Store information used by controller.
 *
 * @author Marco Tranzatto <marco.tranzatto@gmail.com>
 */

#ifndef CONTROLLER_DATA_H
#define CONTROLLER_DATA_H

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>



/** @brief initialize controller data before starting*/
void init_controller_data(void);

/** @brief modify size of the moving window for alpha angle*/
void update_k(const uint16_t k);

/** @brief modify size of the moving window for apparent angle*/
void update_k_app(const uint16_t k);

/** @brief modify size of the moving window for true wind angle*/
void update_k_twd(const uint16_t k);

/** @brief update course over ground with a new value*/
void update_cog(const float cog_r);

/** @brief update true wind (estimated) direction with a new value*/
void update_twd(const float twd_r);

/** @brief update apparent wind direction with a new value*/
void update_app_wind(const float app_r);

/** @brief get the average value of alpha*/
float get_alpha(void);

/** @brief get the average value of apparent wind in sensor frame*/
float get_app_wind_sns(void);

/** @brief get the average value oftrue wind direction in sensor frame*/
float get_twd_sns(void);

/** @brief update yaw angle (w.r.t. true North)*/
void update_yaw(const float yaw_r);

/** @brief compute alpha angle using yaw angle instead of course over ground*/
float get_alpha_yaw(void);

/** @brief set the maximum time after that alpha = alpha_yaw if cod not updated*/
void set_max_time_cog_not_up(float max_time_cog_not_up_sec);

#endif // CONTROLLER_DATA_H
