/****************************************************************************
 *
 *   Copyright (c) 2014 PX4 Development Team. All rights reserved.
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
 * @file boat_opt_status.h
 *
 *  Optimal control status
 */
#ifndef BOAT_OPT_STATUS_H
#define BOAT_OPT_STATUS_H

#include <stdint.h>
#include "../uORB.h"



struct boat_opt_status_s {

    uint64_t	timestamp;		/**< Microseconds since system boot */
    float x1;                   /// extended state[0]
    float x2;                   /// extended state[1]
    float x3;                   /// extended state[2]
    float opt_rud;              /// optimal rudder command computed
    int32_t type_controller;    /// who set the values: 0 = lqr, 1 = MPC
    int32_t it;                 /// MPC solver: iteration number
    float solvetime;            /// MPC solver: solvertime
    float res_eq;               /// MPC solver: inf-norm of equality constraint residuals
    float pobj;                 /// MPC solver: primal objective
    float dobj;                 /// MPC solver: dual objective *
    float dgap;                 /// MPC solver: duality gap := pobj - dobj
    float rdgap;                /// MPC solver: relative duality gap := |dgap / pobj |
};

ORB_DECLARE(boat_opt_status);


#endif // BOAT_OPT_STATUS_H
