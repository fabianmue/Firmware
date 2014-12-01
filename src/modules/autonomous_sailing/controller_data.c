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

#define PRINT_DEBUG 0

#define M_PI_F 3.14159265358979323846f
#define TWO_PI_F 2 * M_PI_F

#ifndef NULL
    #define NULL 0
#endif

//private functions
/** @brief compute moving average to calculate true wind angle (alpha)*/
void compute_avg(void);

/** @brief filter new data computing alpha and a new avg value*/
void filter_new_data(void);

//actual raw measurements from parser_200WX
static struct{
    float cog_r; ///course over ground [rad] (NOT heading), according to Dumas angle definition (Chap 1.3)
    float twd_r; ///true wind estimated direction [rad], according to Dumas angle definition (Chap 1.3)
    bool cog_updated;///true if cog_r has been updated and the new value has to be used to compute a new instant alpha
    bool twd_updated;///true if twd_r has been updated and the new value has to be used to compute a new instant alpha
}measurements_raw;

//filtered measurements
static struct{
    float *alpha_p; ///poniter to vector of last K values of true wind angle, [rad], according to Dumas angle definition (Chap 1.3)
    float *app_wind_p;///pointer to vector of last k_app values of apparent wind read by weather station
    float *twd_p; ///pointer to vector of last k_twd values of true wind read by weather station

    uint16_t k;     ///number of elements in alpha_p
    uint16_t k_app;     ///number of elements in app_wind_p
    uint16_t k_twd;     ///number of elements in twd_p

    int16_t oldestValue; ///index of oldest value in alpha_p
    int16_t oldestValueApp; ///index of oldest value in app_wind_p
    int16_t oldestValueTwd; ///index of oldest value in twd_p

    float alpha;    ///moving average value from alpha_p
    float apparent_wind;///average from app_wind_p
    float twd;///average from twd_p
}measurements_filtered;


/** Initialize all the structures necessary to compute moving average window of true wind angle (alpha)*/
void init_controller_data(void){

    measurements_raw.cog_r = 0.0f;
    measurements_raw.twd_r = 0.0f;
    measurements_raw.cog_updated = false;
    measurements_raw.twd_updated = false;

    measurements_filtered.alpha_p = NULL;
    measurements_filtered.k = 0;
    measurements_filtered.oldestValue = -1;
    measurements_filtered.alpha = 0.0f;

    measurements_filtered.app_wind_p = NULL;
    measurements_filtered.apparent_wind = 0.0f;
    measurements_filtered.k_app = 0;

    measurements_filtered.twd_p = NULL;
    measurements_filtered.k_twd = 0;
    measurements_filtered.oldestValueTwd = -1;
    measurements_filtered.twd = 0.0f;

    //set k to 1 since a real value is not provided
    update_k(1);

    //set k_app to 1 since a real value is not provided
    update_k_app(1);

    #if PRINT_DEBUG == 1
    //printf("init_controller_data \n");
    #endif
}

/** Free memory and allocate new space for new dimension
 *
 * @param k new dimension of the moving window for apparent wind
*/
void update_k_app(const uint16_t k){

    //some controls before freeing memory
    if(k == measurements_filtered.k_app || k == 0)
        return; //nothing to do

    if(measurements_filtered.app_wind_p != NULL)
        free(measurements_filtered.app_wind_p); //free memory

    measurements_filtered.app_wind_p = malloc(sizeof(float) * k);

    measurements_filtered.k_app = k;

    //initialize all the elements of alpha_p to 0
    for(uint16_t i = 0; i < measurements_filtered.k_app; i++){
        measurements_filtered.app_wind_p[i] = 0.0f;
    }

    measurements_filtered.oldestValueApp = 0;

    measurements_filtered.apparent_wind = 0.0f;


    #if PRINT_DEBUG == 1
    //printf("update_k k: %d \n", measurements_filtered.k);
    #endif

}

/** Free memory and allocate new space for new dimension
 *
 * @param k new dimension of the moving window for true wind direction
*/
void update_k_twd(const uint16_t k){

    //some controls before freeing memory
    if(k == measurements_filtered.k_twd || k == 0)
        return; //nothing to do

    if(measurements_filtered.twd_p != NULL)
        free(measurements_filtered.twd_p); //free memory

    measurements_filtered.twd_p = malloc(sizeof(float) * k);

    measurements_filtered.k_twd = k;

    //initialize all the elements of alpha_p to 0
    for(uint16_t i = 0; i < measurements_filtered.k_twd; i++){
        measurements_filtered.twd_p[i] = 0.0f;
    }

    measurements_filtered.oldestValueTwd = 0;

    measurements_filtered.twd = 0.0f;


    #if PRINT_DEBUG == 1
    //printf("update_k k: %d \n", measurements_filtered.k);
    #endif

}

/** Free memory and allocate new space for new dimension
 *
 * @param k new dimension of the moving window for alpha
*/
void update_k(const uint16_t k){

    //some controls before freeing memory
    if(k == measurements_filtered.k || k == 0)
        return; //nothing to do

    if(measurements_filtered.alpha_p != NULL)
        free(measurements_filtered.alpha_p); //free memory

    measurements_filtered.alpha_p = malloc(sizeof(float) * k);

    measurements_filtered.k = k;

    //initialize all the elements of alpha_p to 0
    for(uint16_t i = 0; i < measurements_filtered.k; i++){
        measurements_filtered.alpha_p[i] = 0.0f;
    }

    measurements_filtered.oldestValue = 0;


    #if PRINT_DEBUG == 1
    //printf("update_k k: %d \n", measurements_filtered.k);
    #endif

}

/** Update course over ground with a new value supplied by GPS
 *
 * @param cog_r course over ground [rad], positive on the right, negative on the left (Opposite to Dumas' convention)
*/
void update_cog(const float cog_r){

    //save cog according to Dumas angle definition (Chap 1.3)
    measurements_raw.cog_r = -1 * cog_r;

    //set updated flag
    measurements_raw.cog_updated = true;

    #if PRINT_DEBUG == 1
    printf("saved cog %2.3f \n", (double)measurements_raw.cog_r);
    #endif
}

/** Update apparent direction with a new value supplied by weather station
 *
 * @param app_r apparent wind direction [rad], positive on the right, negative on the left (Opposite to Dumas' convention)
*/
void update_app_wind(const float app_r){

    //just for now, save oldest value of apparent wind direction
    float oldestVal = measurements_filtered.app_wind_p[measurements_filtered.oldestValueApp];

    //delete oldest value in app_wind_p to save app_r
    measurements_filtered.app_wind_p[measurements_filtered.oldestValueApp] = app_r;

    //update oldest value index
    measurements_filtered.oldestValueApp++;

    if(measurements_filtered.oldestValueApp >= measurements_filtered.k_app)
        measurements_filtered.oldestValueApp = 0;

    /* TODO Robust check when apparent angle switches between -pi and pi, the mean
     * will be 0, but the "true" average apparent wind in this case is not blowing
     * from the stern, but from the bow!
    */

    //update apparent wind mean
//    measurements_filtered.apparent_wind = 0.0f;
//    for(uint16_t i = 0; i < measurements_filtered.k_app; i++){

//        measurements_filtered.apparent_wind += measurements_filtered.app_wind_p[i];
//    }

//    measurements_filtered.apparent_wind /= measurements_filtered.k_app;

    measurements_filtered.apparent_wind = measurements_filtered.apparent_wind -
                                          oldestVal / measurements_filtered.k_app +
                                          app_r / measurements_filtered.k_app;

}

/** Update true wind (estimated) direction with a new value supplied by weather station
 *
 * @param twd_r true wind direction [rad], positive on the right, negative on the left (Opposite to Dumas' convention)
*/
void update_twd(const float twd_r){

    //save cog according to Dumas angle definition (Chap 1.3)
    measurements_raw.twd_r = -1 * twd_r;

    //set updated flag
    measurements_raw.twd_updated = true;

    //just for now, save oldest value of twd
    float oldestVal = measurements_filtered.twd_p[measurements_filtered.oldestValueTwd];

    //delete oldest value in twd_p to save twd_r
    measurements_filtered.twd_p[measurements_filtered.oldestValueTwd] = twd_r;

    //update oldest value index
    measurements_filtered.oldestValueTwd++;

    if(measurements_filtered.oldestValueTwd >= measurements_filtered.k_twd)
        measurements_filtered.oldestValueTwd = 0;

    //update twd mean

    measurements_filtered.twd = measurements_filtered.twd -
                                          oldestVal / measurements_filtered.k_twd +
                                          twd_r / measurements_filtered.k_twd;

    #if PRINT_DEBUG == 1
    printf("saved twd %2.3f \n", (double)measurements_raw.twd_r);
    #endif
}

/** Compute moving average from values in alpha_p */
void compute_avg(void){

    float temp = 0.0f;

    for(uint16_t i = 0; i < measurements_filtered.k; i++){
        temp += measurements_filtered.alpha_p[i];

        #if PRINT_DEBUG == 1
        printf(" measurements_filtered.alpha_p[%d] %2.3f \n", i, (double) measurements_filtered.alpha_p[i]);
        #endif
    }

    //compute average value of alpha
    measurements_filtered.alpha = temp / measurements_filtered.k;

    #if PRINT_DEBUG == 1
    printf("temp %2.3f \n", (double)temp);
    //printf("measurements_filtered.k %2.3f \n", (double)measurements_filtered.k);
    //printf("measurements_filtered.alpha %2.3f \n", (double)measurements_filtered.alpha);
    #endif
}

/** Compute instant alpha from a new cog and/or twd value.
 *  Should be called only if either cog_r or twd_r have been updated.
 *  At the end, set flags of updated to false.
*/
void filter_new_data(void){

    //cog or twd has just been updated, compute new istant alpha according to Dumas
    float instant_alpha = measurements_raw.cog_r - measurements_raw.twd_r;

    //if |instant_alpha|<= pi/2 we're sailing upwind, so everything is ok
    //constrain alpha to be the CLOSER angle between TWD and COG
    if(instant_alpha > M_PI_F) instant_alpha = instant_alpha - TWO_PI_F;
    else if(instant_alpha < -M_PI_F) instant_alpha = instant_alpha + TWO_PI_F;

    //save new data by deleting the oldest value
    measurements_filtered.alpha_p[measurements_filtered.oldestValue] = instant_alpha;

    //update oldest value index
    measurements_filtered.oldestValue++;

    if(measurements_filtered.oldestValue >= measurements_filtered.k)
        measurements_filtered.oldestValue = 0;

    //set updated flag to false 'cause cog and twd values have been used
    measurements_raw.cog_updated = false;
    measurements_raw.twd_updated = false;

    #if PRINT_DEBUG == 1
    //printf("instant_alpha %2.3f \n", (double)instant_alpha);
    //printf("New oldestValue %d \n", measurements_filtered.oldestValue);
    #endif

}

/** Return the average value of alpha computed from the last k values
 *
 * Before returing avg value, check if either cog_r or twd_r have been updated.
 * If so, compute a new instant alpha and add it to the vector alpha_p.
 * Then compute new avg alpha and return it.
 * If neither cog_r nor twd_r have been updated, return old value of alpha.
 *
 * @return moving average value of true wind angle from the last k values of instant alpha.
*/
float get_alpha(void){

    //check if either cog_r or twd_r have been updated
    if(measurements_raw.cog_updated || measurements_raw.twd_updated){

        //compute new instant alpha by filtering the new data
        filter_new_data();

        //compute new mean alpha
        compute_avg();
    }

    #if PRINT_DEBUG == 1
    //printf(" *** get_alpha.alpha %2.3f *** \n", (double)measurements_filtered.alpha);
    #endif

    return measurements_filtered.alpha;
}

/** Return the average value of apparent wind direction computed from the last k_app values
 *
 * @return moving average value of apparent wind direction
*/
float get_app_wind(void){

    return measurements_filtered.apparent_wind;
}

/** Return the average value of true wind direction computed from the last k_twd values
 *
 * @return moving average value of true wind direction
*/
float get_twd(void){

    return measurements_filtered.twd;
}

