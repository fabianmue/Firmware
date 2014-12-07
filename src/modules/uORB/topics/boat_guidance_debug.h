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
 * @file boat_guidance_debug.h
 *
 */

#ifndef BOAT_GUIDANCE_DEBUG_H
#define BOAT_GUIDANCE_DEBUG_H

#include <stdint.h>

struct boat_guidance_debug_s {
    uint64_t timestamp;
    float alpha_star;           ///reference true wind angle [rad], Dumas' convention
    float alpha;                ///actual measurements of alpha [rad], Dumas' convention
    float rudder_action;        ///input for rudder servo motor
    float sail_action;          ///input for sail servo motor
    float next_grid_line;       ///value [m] of the next grid line in Race frame
    float x_race;               ///current x-coordinate [m] in Race frame
    float y_race;               ///current y-coordinate [m] in Race frame
    uint8_t should_tack;         ///1 if boat should tack  as soon as possibile
    float twd_mean;             /**< True wind diretion mean value*/
    float app_mean;             /**< Apparent wind direction mean value*/
};

ORB_DECLARE(boat_guidance_debug);

#endif // BOAT_GUIDANCE_DEBUG_H
