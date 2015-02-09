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
 * @file boat_opt_matrices.h
 *
 * Useful matrices set by the user during sailing.
 * These matrices are used by either MPC or LQR controllers.
 *
 */

#ifndef BOAT_OPT_MATRICES_H
#define BOAT_OPT_MATRICES_H

#include <stdint.h>

struct boat_opt_matrices_s{
    uint64_t timestamp;
    float lqr_k1;   /// first value LQR gain matrix
    float lqr_k2;   /// second value LQR gain matrix
    float lqr_k3;   /// third value LQR gain matrix
    float mpc_h1;   /// first diagonal matrix of hessian amtrix H used in the MPC
    float mpc_h2;   /// second diagonal matrix of hessian amtrix H used in the MPC
    float mpc_h3;   /// third diagonal matrix of hessian amtrix H used in the MPC
    float mpc_h4;   /// fourth diagonal matrix of hessian amtrix H used in the MPC
    float mpc_lb1;  /// first lower bound value in the MPC
    float mpc_lb2;  /// second lower bound value in the MPC
    float mpc_ub1;  /// first upper bound value in the MPC
    float mpc_ub2;  /// second upper bound value in the MPC
};

ORB_DECLARE(boat_opt_matrices);

#endif // BOAT_OPT_MATRICES_H
