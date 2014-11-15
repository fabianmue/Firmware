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
 * @file controller_data.c
 *
 * Store information used by controller.
 *
 * @author Marco Tranzatto <marco.tranzatto@gmail.com>
 */

#include "controller_data.h"

//actual raw measurements from parser_200WX
static struct{
    float cog_r; ///course over ground [rad] (NOT heading), according to Dumas angle definition (Chap 1.3)
    float twd_r; ///true wind estimated direction [rad], according to Dumas angle definition (Chap 1.3)
}measurements_raw;

//filtered measurements
static struct{
    float* alpha_p; ///poniter to vector of last K values of true wind angle, [rad], according to Dumas angle definition (Chap 1.3)
    uint32_t k;     ///number of elements in alpha_p
    int32_t oldestValue; ///index of oldest value
    float alpha;    ///moving average value from alpha_p
}measurements_filtered;


/** Initialize all the structures necessary to compute moving average window of true wind angle (alpha)*/
void init_controller_data(){

    measurements_raw.cog_r = 0.0f;
    measurements_raw.twd_r = 0.0f;

    measurements_filtered.alpha = NULL;
    measurements_filtered.k = 0;
    measurements_filtered.oldestValue = -1;
    measurements_filtered.alpha = 0.0f;
}

/** Free memory and allocate new space for new dimension
 *
 * @param k new dimension of the moving window
*/
void update_k(const uint32_t k){

    //some controls before freeing memory
    if(k == measurements_filtered.k)
        return; //nothing to do

    if(measurements_filtered.alpha_p != NULL)
        free(measurements_filtered.alpha_p); //free memory

    measurements_filtered.alpha_p = malloc(sizeof(float) * k);
    measurements_filtered.k = k;
    measurements_filtered.oldestValue = -1;

}

/** Update course over ground with a new value supplied by GPS
 *
 * @param cog course over ground [rad], positive on the right, negative on the left (Opposite to Dumas' convention)
*/
void update_cog(const float cog_r){

    //save cog according to Dumas angle definition (Chap 1.3)
    measurements_raw.cog_r = -1 * cog_r;
}
