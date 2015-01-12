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

/** @brief compute a robust average of sensor "frame" measurements*/
float robust_avg_sns(float *p_meas, const uint16_t k);

//actual raw measurements from parser_200WX
static struct{
    float cog_r; ///course over ground [rad] (NOT heading), according to Dumas angle definition (Chap 1.3)
    float twd_r; ///true wind estimated direction [rad], according to Dumas angle definition (Chap 1.3)
    bool cog_updated;///true if cog_r has been updated and the new value has to be used to compute a new instant alpha
    bool twd_updated;///true if twd_r has been updated and the new value has to be used to compute a new instant alpha
    float yaw_r;///last yaw (w.r.t. true North) angle provided by the Kalman filter
    uint64_t time_last_cog_update;///last time when cog_r has been updated
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

//other user's paramters
static struct{
    uint64_t max_time_cog_not_up;//maximum time without updating cog, in microseconds
}user_params = {
    .max_time_cog_not_up = 1 * 1000000 //1 sec
};


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

    //set k_twd to 1 since a real value is not provided
    update_k_twd(1);

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

/**
 * Update course over ground with a new value supplied by GPS.
 *
 * Set the flag of cog_updated only if the new cog_r supplied is different
 * from the last one. This is necessary because when no cog message
 * is received by parser_200WX, the cog value remains constant.
 *
 * @param cog_r course over ground [rad], positive on the right, negative on the left (Opposite to Dumas' convention)
*/
void update_cog(const float cog_r){

    //save cog according to Dumas angle definition (Chap 1.3, Dumas' thesis)
    float cog_r_dumas = -1 * cog_r;

    //check if cog_r_dumas is different from the previous cog_r value saved
    if(cog_r_dumas != measurements_raw.cog_r){
        //save new cog value
        measurements_raw.cog_r = cog_r_dumas;

        //set updated flag
        measurements_raw.cog_updated = true;

        //update time when a new cog has been provided
        measurements_raw.time_last_cog_update = hrt_absolute_time();
    }

    #if PRINT_DEBUG == 1
    printf("saved cog %2.3f \n", (double)measurements_raw.cog_r);
    #endif
}

/** Update apparent direction with a new value supplied by weather station
 *
 * @param app_r apparent wind direction [rad], positive on the right, negative on the left (Opposite to Dumas' convention)
*/
void update_app_wind(const float app_r){

    //delete oldest value in app_wind_p to save app_r
    measurements_filtered.app_wind_p[measurements_filtered.oldestValueApp] = app_r;

    //update oldest value index
    measurements_filtered.oldestValueApp++;

    if(measurements_filtered.oldestValueApp >= measurements_filtered.k_app)
        measurements_filtered.oldestValueApp = 0;

    //update apparent wind mean using a robust mean
    measurements_filtered.apparent_wind = robust_avg_sns(measurements_filtered.app_wind_p,
                                                         measurements_filtered.k_app);

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

    //TODO wind between -pi and pi will give as mean value 0, but it is NOT correct!!!

    //delete oldest value in twd_p to save twd_r
    measurements_filtered.twd_p[measurements_filtered.oldestValueTwd] = twd_r;

    //update oldest value index
    measurements_filtered.oldestValueTwd++;

    if(measurements_filtered.oldestValueTwd >= measurements_filtered.k_twd)
        measurements_filtered.oldestValueTwd = 0;

    //update twd mean using a robust mean
    measurements_filtered.twd = robust_avg_sns(measurements_filtered.twd_p,
                                               measurements_filtered.k_twd);

    #if PRINT_DEBUG == 1
    printf("saved twd %2.3f \n", (double)measurements_raw.twd_r);
    #endif
}

/** Compute moving average from values in alpha_p */
void compute_avg(void){

    float temp = 0.0f;
    float temp_avg;

    for(uint16_t i = 0; i < measurements_filtered.k; i++){
        temp += measurements_filtered.alpha_p[i];

        #if PRINT_DEBUG == 1
        printf(" measurements_filtered.alpha_p[%d] %2.3f \n", i, (double) measurements_filtered.alpha_p[i]);
        #endif
    }

    //compute average value of alpha. This alpha used the cog values, see later.
    temp_avg = temp / measurements_filtered.k;

    /*
     * Since cog values provided by the GPS can suffer of a low frequency updating, the
     * alpha angle computed with not updated cog values can be quite different from the real one.
     * To avoid this, we check the difference between the actual absolute time
     * and the last time che cog has been update (@see update_cog).
     * If this difference is greater than max_time_cog_not_up,
     * then we use the alpha angle
     * computed using the yaw angle instead of the one computed with
     * the course over ground angle.
    */
    if((hrt_absolute_time() - measurements_raw.time_last_cog_update) <=
            user_params.max_time_cog_not_up)
        measurements_filtered.alpha = temp_avg;
    else
        measurements_filtered.alpha = get_alpha_yaw();

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
    //constrain alpha to be the CLOSEST angle between TWD and COG
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
 * Before returing avg value, checks if either cog_r or twd_r have been updated.
 * If so, computes a new instant alpha and add it to the vector alpha_p.
 * Then computes new avg alpha and return it.
 * If neither cog_r nor twd_r have been updated, returns old value of alpha.
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

/**
 * Update a pre-exsiting moving mean in a robust way.
 *
 * Angles measurements from weather station and GPS are in interval [-pi, pi].
 *
 *                North
 *
 *                  0
 *                  |
 *                  |
 *  West  -pi/2 ____|____ pi/2  East
 *                  |
 *                  |
 *                  |
 *               -pi|pi
 *
 *                 South
 *
 *
 * If an angle switches between -pi and pi, a normal mean will return 0, that is wrong.
 * This robust mean uses the Mitsuta method to compute a "robust" mean.
 *
 * @param p_meas        array containing angles to mean
 * @param k             number of element in p_meas
 * @return              new robust mean
*/
float robust_avg_sns(float *p_meas, const uint16_t k){

    float new_mean;
    float D;
    float delta;
    float sum;
    float tmp_meas;

    /*Be careful: Mitsuta method only works if angles are between 0 and 2*pi AND if
    if consecutive raw readings differ by less than pi */

    //convert angle read from interval [-pi, pi] to [0, 2pi]
    tmp_meas = (p_meas[0] < 0) ? p_meas[0] + TWO_PI_F : p_meas[0];
    sum = tmp_meas;
    D = tmp_meas;

    for(uint16_t i = 1; i < k; i++){
        //convert sensor angle from interval [-pi, pi] to [0, 2pi]
        tmp_meas = (p_meas[i] < 0) ? p_meas[i] + TWO_PI_F : p_meas[i];

        delta = tmp_meas - D;

        if(delta < -M_PI_F)
            D = D + delta + TWO_PI_F;
        else if(delta < M_PI_F)
            D = D + delta;
        else
            D = D + delta - TWO_PI_F;

        sum = sum + D;
    }
    //compute mean
    new_mean = sum / k;

    //additional cautions
    if(new_mean < 0)
        new_mean = new_mean + TWO_PI_F;
    else if(new_mean > TWO_PI_F)
        new_mean = new_mean - TWO_PI_F;

    //convert new_mean from Mitsuta convention to sensor convention
    new_mean = (new_mean > M_PI_F) ? new_mean - TWO_PI_F : new_mean;

    return new_mean;
}

/**
 * Update yaw angle value.
*/
void update_yaw(const float yaw_r){
    measurements_raw.yaw_r = yaw_r;
}

/**
 * Get alpha angle (angle with respet to the wind) using yaw angle.
 *
 * @see get_alpha uses course over ground provided by the GPS raw measurements
 * to compute the alpha angle. This function computes alpha = yaw - twd, where
 * yaw is the last yaw angle provided by the Kalman filter and set by @see update_yaw.
 * twd is the value provided by @see get_twd().
*/
float get_alpha_yaw(void){
    float alpha;

    /* Since yaw angle provided by the Kalman filter is already a filtered value,
     * we use it "as it is" without using any moving average filter.
     * get_twd() provides a robust (@see robust_avg_sns) true wind angle
     * measurement mean value.
    */
    alpha = measurements_raw.yaw_r - get_twd();

    //if |instant_alpha|<= pi/2 we're sailing upwind, so everything is ok
    //constrain alpha to be the CLOSEST angle between TWD and COG
    if(alpha > M_PI_F) alpha = alpha - TWO_PI_F;
    else if(alpha < -M_PI_F) alpha = alpha + TWO_PI_F;

    return alpha;
}
