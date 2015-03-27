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
 * @file boat_pp_debug.h
 *
 * Data that is logged during tests of the pathplanning application
 *
 */

#ifndef BOAT_PP_DEBUG_H
#define BOAT_PP_DEBUG_H

#include <stdint.h>
#include "../uORB.h"

struct boat_pp_debug1_s {
    uint64_t timestamp;
    float obsthorizon;		/**< Obstacle horizon [m] */
    float obstsafetyrad;    /**< Safety Radius around obstacles */
    float windowsize;		/**< Windowsize of smoothing filter in costfunctionpathplanning */
    float glee;				/**< Lee-Cost weight */
    float gm;				/**< Maneuver-Cost weight */
    float go;				/**< Obstacle-Cost weight */
    float gs; 				/**< Same-Heading-Cost weight */
    float gt;				/**< Tactical-Cost weight */
    float gw;				/**< Wind and Target-Cost weight */
    float period;			/**< Time between two consecutive executions of the Pathplanning algorithm */
};

ORB_DECLARE(boat_pp_debug1);

#endif // BOAT_QGC_PARAM_H
