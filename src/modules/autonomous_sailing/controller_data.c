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

#ifndef NULL
    #define NULL 0
#endif

//private functions
/** @brief compute moving average to calculate true wind angle (alpha)*/
void compute_avg();

/** @brief filter new data computing alpha and a new avg value*/
void filter_new_data();

//actual raw measurements from parser_200WX
static struct{
    float cog_r; ///course over ground [rad] (NOT heading), according to Dumas angle definition (Chap 1.3)
    float twd_r; ///true wind estimated direction [rad], according to Dumas angle definition (Chap 1.3)
}measurements_raw;

//filtered measurements
static struct{
    float* alpha_p; ///poniter to vector of last K values of true wind angle, [rad], according to Dumas angle definition (Chap 1.3)
    uint16_t k;     ///number of elements in alpha_p
    int16_t oldestValue; ///index of oldest value
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

    //use default value for k
    update_k(DEFAULT_AVG_WINDOW);
}

/** Free memory and allocate new space for new dimension
 *
 * @param k new dimension of the moving window
*/
void update_k(const uint16_t k){

    //some controls before freeing memory
    if(k == measurements_filtered.k)
        return; //nothing to do

    if(measurements_filtered.alpha_p != NULL)
        free(measurements_filtered.alpha_p); //free memory

    measurements_filtered.alpha_p = malloc(sizeof(float) * k);

    measurements_filtered.k = k;

    //initialize all the elements of alpha_p to 0
    for(int i = 0; i < measurements_filtered.k; i++){
        measurements_filtered.alpha_p[i] = 0.0f;
    }

    measurements_filtered.oldestValue = 0;

}

/** Update course over ground with a new value supplied by GPS
 *
 * @param cog_r course over ground [rad], positive on the right, negative on the left (Opposite to Dumas' convention)
*/
void update_cog(const float cog_r){

    //save cog according to Dumas angle definition (Chap 1.3)
    measurements_raw.cog_r = -1 * cog_r;

    //filter new raw data
    filter_new_data();
}

/** Update ctrue wind (estimated) direction with a new value supplied by weather station
 *
 * @param twd_r true wind direction [rad], positive on the right, negative on the left (Opposite to Dumas' convention)
*/
void update_twd(const float twd_r){

    //save cog according to Dumas angle definition (Chap 1.3)
    measurements_raw.twd_r = -1 * twd_r;

    //filter new raw data
    filter_new_data();
}

/** Compute moving average from values in alpha_p */
void compute_avg(){

    float temp = 0.0f;

    for(uint16_t i = 0; i < measurements_filtered.k; i++){
        temp += measurements_filtered.alpha_p[i];
    }

    //compute average value of alpha
    measurements_filtered.alpha = temp / measurements_filtered.k;
}

/** Compute instant alpha from a new cog and/or twd value, then update mean value of alpha*/
void filter_new_data(){
    //cog or twd has just been updated, compute new istant alpha according to Dumas
    float instant_alpha = measurements_raw.cog_r - measurements_raw.twd_r;

    //if |instant_alpha|<= pi we're sailing upwind, so everything is ok

    //save new data by deleting the oldest value
    measurements_filtered.alpha_p[measurements_filtered.oldestValue] = instant_alpha;

    //update oldest value index
    if(measurements_filtered.oldestValue >= measurements_filtered.k)
        measurements_filtered.oldestValue = 0;
    else
        measurements_filtered.oldestValue++;

    //compute new mean alpha
    compute_avg();

}

/** Return the average value of alpha computed from the last k values*/
float get_alpha(){
    return measurements_filtered.alpha;
}

