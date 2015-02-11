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
 * @file boat_optimal_control.h
 *
 *
 */

#ifndef BOAT_OPTIMAL_CONTROL_H
#define BOAT_OPTIMAL_CONTROL_H

#include <stdint.h>
#include "../uORB.h"

struct boat_opt_mat_s{
    uint64_t timestamp;
    float lqr_k1;   /// first value LQR gain matrix
    float lqr_k2;   /// second value LQR gain matrix
    float lqr_k3;   /// third value LQR gain matrix
    float mpc_h1;   /// first diagonal matrix of hessian matrix H used in the MPC
    float mpc_h2;   /// second diagonal matrix of hessian matrix H used in the MPC
    float mpc_h3;   /// third diagonal matrix of hessian matrix H used in the MPC
    float mpc_h4;   /// fourth diagonal matrix of hessian matrix H used in the MPC
    float mpc_lb1;  /// first lower bound value in the MPC
    float mpc_lb2;  /// second lower bound value in the MPC
    float mpc_ub1;  /// first upper bound value in the MPC
    float mpc_ub2;  /// second upper bound value in the MPC
};

/*struct boat_opt_ctr_s{
    uint64_t timestamp;
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
};*/


ORB_DECLARE(boat_opt_mat);
//ORB_DECLARE(boat_opt_ctr);

#endif // BOAT_OPTIMAL_CONTROL_H
