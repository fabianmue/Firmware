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
 * @file navigation.h
 *
 * Computes NED position from geodedical information.
 *
 * @author Marco Tranzatto <marco.tranzatto@gmail.com>
 */

#ifndef NAVIGATION_H_
#define NAVIGATION_H_


#include <math.h>
#include <stdint.h>

//Include topics necessary
#include "topics_handler.h"

//local position of the boat in the Race frame, in meters.
struct local_position_race_s{
    float x_race_m;
    float y_race_m;
};


/** @brief set origin of NED frame.*/
void set_ref0(const int32_t  *_lat0_d_e7_p, const int32_t  *_lon0_d_e7_p, const int32_t  *_alt0_mm_p);

/** @brief set the angle of the mean wind w.r.t. true North*/
void set_mean_wind_angle(float mean_wind);

/** @brief set the origin of the top mark buoy*/
void set_pos_top_mark(const int32_t  *lat_d_e7_p, const int32_t  *lon_d_e7_p, const int32_t  *alt_mm_p);

/** @brief Convert GPS data in position in Race frame coordinate*/
void navigation_module(const struct structs_topics_s *strs_p,
                       struct local_position_race_s *lp_p);

/** @brief get the angle of the mean wind w.r.t. true North*/
float get_mean_wind_angle(void);

/** @brief update rotation matrix body to world*/
void update_r_ned_body(float R[3][3], bool valid_matrix);

/** @brief update volicities on the NED frame*/
void update_ned_vel(float vn, float ve, float vd);

/** @brief get longitudinal velocity in the body frame*/
float get_u_vel(void);

#endif /* NAVIGATION_H_ */
