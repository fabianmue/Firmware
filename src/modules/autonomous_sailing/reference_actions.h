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
* @file reference_actions.h
*
* Reference actions computed offline.
*
* @author Marco Tranzatto <marco.tranzatto@gmail.com>
*/

#ifndef REFERENCE_ACTIONS_H
#define REFERENCE_ACTIONS_H
#include <stdint.h>

extern int16_t total_grids_number; /// total number of grid lines
extern float d_x; /// distance [m] from each grid line
extern float d_y; /// distance [m] from every discrete y point

extern int8_t actions_w1_h1[7][31]; /// optimal actions for wind from NNW and port haul
extern int8_t actions_w1_h2[7][31]; /// optimal actions for wind from NNW and starboard haul
extern int8_t actions_w2_h1[7][31]; /// optimal actions for wind from NNE and port haul
extern int8_t actions_w2_h2[7][31]; /// optimal actions for wind from NNE and starboard haul

extern int16_t actions_row_number; /// number of rows in actions_* matrixes
extern int16_t actions_col_number; /// number of cols in actions_* matrixes
extern int16_t y_max[7]; /// half number of points (-1) on each gridline

#endif // REFERENCE_ACTIONS_H
