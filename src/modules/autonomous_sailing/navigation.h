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

#include <uORB/topics/vehicle_global_position.h>
#include <math.h>
#include <stdint.h>

/** @brief convert geodedical coordinates into NED coordinate.*/
void geo_to_ned(const struct vehicle_global_position_s *gps_p,
                int32_t *north_cm_p, int32_t *east_cm_p, int32_t *down_cm_p);

/** @brief convert geodedical coordinates into ECEF coordinate.*/
void geo_to_ecef(const int32_t  *lat_d_e7_p, const int32_t  *lon_d_e7_p, const int32_t  *alt_mm_p,
                 int32_t  *x_cm_p, int32_t  *y_cm_p, int32_t  *z_cm_p);

/** @brief convert ECEF coordinates into NED coordinate.*/
void ecef_to_ned(const int32_t *x_cm_p, const int32_t *y_cm_p, const int32_t *z_cm_p,
                 int32_t *north_cm_p, int32_t *east_cm_p, int32_t *down_cm_p);

/** @brief set origin of NED frame.*/
void set_ref0(const int32_t  *_lat0_d_e7_p, const int32_t  *_lon0_d_e7_p, const int32_t  *_alt0_mm_p);

/** @brief convert Deg*E7 in rad */
float degE7_to_rad(const int32_t  *deg_e7_p);

#endif /* NAVIGATION_H_ */
