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
 * Store information used by controllers.
 *
 * @author Marco Tranzatto <marco.tranzatto@gmail.com>
 */

#include "controller_data.h"

#define PRINT_DEBUG 0

#define M_PI_F 3.14159265358979323846f
#define TWO_PI_F 2 * M_PI_F

static float rad2deg = 57.2957795130823f;
static char txt_msg[150]; ///used to send messages to QGC

#ifndef NULL
    #define NULL 0
#endif

//private functions
/** @brief compute a new value for alpha in our sensor convention*/
void update_alpha_sns(void);

/** @brief compute a robust average of sensor "frame" measurements*/
float robust_avg_sns(float *p_meas, const uint16_t k);

/** @brief compute lambda value to compute a linear combination of alpha_cog and alpha_yaw*/
float compute_lambda(void);

/** @brief compute instant alpha in sensor frame*/
float compute_instant_alpha_sns(float angle);

/** @brief fill a float vector with a constant value*/
void fill_fvector(float* vector, uint16_t vector_size, float value);

/** @brief print on QGC information if using fixed true wind */
void print_fixed_twd_info(void);

//actual raw measurements from parser_200WX and other usefull data
static struct{
    float cog_sns;///last cog value supplied by the gps
    float alpha_cog_sns;///last alpha angle computed using cog, according to our sensor convention
    float alpha_yaw_sns;/// last alpha angle computed using yaw, according to our sensor convention
    uint64_t time_last_cog_update;///last time when cog_r has been updated
    float yaw_rate_sns; ///last yaw rate [rad/s], according to our sensor convention
}measurements_raw;

//filtered measurements & other usefull data to filter them
static struct{
    float *alpha_p; ///poniter to vector of last K values of true wind angle, [rad], according to Dumas angle definition (Chap 1.3)
    float *app_wind_p;///pointer to vector of last k_app values of apparent wind read by weather station
    float *twd_p; ///pointer to vector of last k_twd values of true wind read by weather station

    uint16_t k;         ///total number of elements in alpha_p
    uint16_t k_app;     ///total number of elements in app_wind_p
    uint16_t k_twd;     ///totla number of elements in twd_p

    uint16_t opt_tack_alpha_win; ///window size of alpha filter when tacking either with LQR or MPC
    uint16_t opt_tack_twd_win; ///window size of TWD filter when tacking either with LQR or MPC

    uint16_t alpha_window_length; ///effective length of the window for the moving average filter of alpha
    uint16_t twd_window_length; ///effective length of the window for the moving average filter of alpha

    bool opt_tack_going; ///true if the boat is tacking using either LQR or MPC

    int16_t oldestValue; ///index of oldest value in alpha_p
    int16_t oldestValueApp; ///index of oldest value in app_wind_p
    int16_t oldestValueTwd; ///index of oldest value in twd_p

    float alpha_sns;    ///moving average value from alpha_p
    float apparent_wind_sns;///average from app_wind_p
    float twd_sns;///average from twd_p
}measurements_filtered;

//other user's paramters
static struct{
    uint64_t max_time_cog_not_up;///maximum time without updating cog, in microseconds
    bool use_fixed_twd; ///true if alpha must be computed with a constant twd value
    float fixed_twd_r; ///fixed true wind direction in rad
}user_params = {
    .max_time_cog_not_up = 2 * 1000000,//micro seconds
    .use_fixed_twd = false, //us twd from get_twd_sns()
    .fixed_twd_r = 0
};

/**
 * Set max_time_cog_not_up.
 *
 * The alpha angle is computed using a convex combination of alpha_cog and
 * alpha_yaw. If the cog value from the gps is not updated for more than
 * max_time_cog_not_up seconds, then alpha will be equal to alpha_yaw.
 *
 * @param max_time_cog_not_up_sec   seconds used in @see compute_lambda() to compute lambda
*/
void cd_set_max_time_cog_not_up(float max_time_cog_not_up_sec){
    user_params.max_time_cog_not_up = (uint64_t)(max_time_cog_not_up_sec * 1000000.0f);
}

/**
 * Initialize all the necessary structurs.
 * Call this function only once, before starting the while loop in autonomous_sailing app.
*/
void cd_init_controller_data(void){
    measurements_raw.alpha_cog_sns = 0.0f;
    measurements_raw.alpha_yaw_sns = 0.0f;
    measurements_raw.cog_sns = 0.0f;

    measurements_filtered.alpha_p = NULL;
    measurements_filtered.k = 0;
    measurements_filtered.oldestValue = -1;
    measurements_filtered.alpha_sns = 0.0f;
    measurements_filtered.opt_tack_alpha_win = 0;
    measurements_filtered.alpha_window_length = 0;

    measurements_filtered.app_wind_p = NULL;
    measurements_filtered.apparent_wind_sns = 0.0f;
    measurements_filtered.k_app = 0;

    measurements_filtered.twd_p = NULL;
    measurements_filtered.k_twd = 0;
    measurements_filtered.oldestValueTwd = -1;
    measurements_filtered.twd_sns = 0.0f;
    measurements_filtered.opt_tack_twd_win = 0;
    measurements_filtered.twd_window_length = 0;

    //we are not taking at the beginning
    measurements_filtered.opt_tack_going = false;

    //set k to 1 since real values are not provided
    cd_update_k(1, 1);

    //set k_app to 1 since a real value is not provided
    cd_update_k_app(1);

    //set k_twd to 1 since real values are not provided
    cd_update_k_twd(1, 1);
}

/** Free memory and allocate new space for new dimension
 *
 * @param k new dimension of the moving window for apparent wind
*/
void cd_update_k_app(const uint16_t k){

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

    measurements_filtered.apparent_wind_sns = 0.0f;
}

/** Free memory and allocate new space for new dimension
 *
 * @param k new dimension of the moving window for true wind direction
 * @param opt_tack_twf_win opt_tack_twd_win size when tacking with either LQR or MPC, must be <= k
*/
void cd_update_k_twd(const uint16_t k, uint16_t opt_tack_twd_win){

    //has k a new value?
    if(k != measurements_filtered.k_twd && k != 0){
        //free memory
        if(measurements_filtered.twd_p != NULL)
            free(measurements_filtered.twd_p);

        measurements_filtered.twd_p = malloc(sizeof(float) * k);

        measurements_filtered.k_twd = k;

        //initialize all the elements of twd_p to 0
        fill_fvector(measurements_filtered.twd_p, k, 0.0f);

        measurements_filtered.oldestValueTwd = 0;

        measurements_filtered.twd_sns = 0.0f;
    }

    //has opt_tack_twd_win a new value ?
    if(opt_tack_twd_win != measurements_filtered.opt_tack_twd_win && opt_tack_twd_win != 0){
        //make sure opt_tack_twd_win <=  measurements_filtered.k_twd
        if(opt_tack_twd_win >  measurements_filtered.k_twd)
            opt_tack_twd_win =  measurements_filtered.k_twd;
        measurements_filtered.opt_tack_twd_win = opt_tack_twd_win;
    }

    //to set the real size of the windowd of the filter, check if we are tacking
    if(measurements_filtered.opt_tack_going == true)
        measurements_filtered.twd_window_length = measurements_filtered.opt_tack_twd_win;
    else
        measurements_filtered.twd_window_length = measurements_filtered.k_twd;
}

/** Free memory and allocate new space for new dimension
 *
 * @param k new dimension of the moving window for alpha
 * @param opt_tack_alpha_win window size when tacking with either LQR or MPC, must be <= k
*/
void cd_update_k(const uint16_t k, uint16_t opt_tack_alpha_win){

    //has k a new value?
    if(k != measurements_filtered.k && k != 0){
        //free memory
        if(measurements_filtered.alpha_p != NULL)
            free(measurements_filtered.alpha_p);

        measurements_filtered.alpha_p = malloc(sizeof(float) * k);

        measurements_filtered.k = k;

        //initialize all the elements of alpha_p to 0
        fill_fvector(measurements_filtered.alpha_p, k, 0.0f);

        measurements_filtered.oldestValue = 0;

        measurements_filtered.alpha_sns = 0.0f;
    }

    //has opt_tack_alpha_win a new value ?
    if(opt_tack_alpha_win != measurements_filtered.opt_tack_alpha_win && opt_tack_alpha_win != 0){
        //make sure opt_tack_alpha_win <= measurements_filtered.k
        if(opt_tack_alpha_win > measurements_filtered.k)
            opt_tack_alpha_win = measurements_filtered.k;
        measurements_filtered.opt_tack_alpha_win = opt_tack_alpha_win;
    }

    //to set the real size of the windowd of the filter, check if we are tacking
    if(measurements_filtered.opt_tack_going == true)
        measurements_filtered.alpha_window_length = measurements_filtered.opt_tack_alpha_win;
    else
        measurements_filtered.alpha_window_length = measurements_filtered.k;
}

/**
 * Compute the alpha angle in our sensor convention
 *
 * The alpha angle is defined as angle - true wind direction,
 * where angle can be either the yaw angle or the corse over ground angle.
 * The true wind direction can be either the twd provided by a moving
 * average filter, @see get_twd_sns(), or a fixed twd set by the
 * user via QGroundControl.
 *
 * @param angle     yaw or cog angle, in sensor convention
 * @return          instant alpha computed using angle, in sensor frame
*/
float compute_instant_alpha_sns(float angle)
{
    float alpha;
    float twd;

    //see which twd angle we must use
    if(user_params.use_fixed_twd == true)
        twd = user_params.fixed_twd_r;
    else
        twd = cd_get_twd_sns();

    //compute alpha
    alpha = angle - twd;

    //constrain alpha to be the CLOSEST angle between TWD and angle
    if(alpha > M_PI_F) alpha = alpha - TWO_PI_F;
    else if(alpha < -M_PI_F) alpha = alpha + TWO_PI_F;

    return alpha;
}

/**
 * Update course over ground with a new value supplied by GPS.
 *
 * Compute a new value of alpha_cog using the new cog value.
 *
 * @param cog_r course over ground [rad], according to our sensor convention
*/
void cd_update_raw_cog(const float cog_r){

    //check if cog_r is different from the previous cog_r value saved
    if(cog_r != measurements_raw.cog_sns){
        //save new cog value
        measurements_raw.cog_sns = cog_r;

        //update time when a new cog has been provided
        measurements_raw.time_last_cog_update = hrt_absolute_time();

        //compute a new value for alpha_cog and save it
        measurements_raw.alpha_cog_sns = compute_instant_alpha_sns(cog_r);

        //since there is a new value of alpha_cog, update the alpha value in the sensor frame
        update_alpha_sns();
    }
}

/** Update apparent direction with a new value supplied by weather station
 *
 * @param app_r apparent wind direction [rad], positive on the right, negative on the left (Opposite to Dumas' convention)
*/
void cd_update_raw_app_wind(const float app_r){

    //delete oldest value in app_wind_p to save app_r
    measurements_filtered.app_wind_p[measurements_filtered.oldestValueApp] = app_r;

    //update oldest value index
    measurements_filtered.oldestValueApp++;

    if(measurements_filtered.oldestValueApp >= measurements_filtered.k_app)
        measurements_filtered.oldestValueApp = 0;

    //update apparent wind mean using a robust mean
    measurements_filtered.apparent_wind_sns = robust_avg_sns(measurements_filtered.app_wind_p,
                                                             measurements_filtered.k_app);

}

/** Update true wind (estimated) direction with a new value supplied by weather station
 *
 * @param twd_r true wind direction [rad], positive on the right, negative on the left (Opposite to Dumas' convention)
*/
void cd_update_raw_twd(const float twd_r){

    //delete oldest value in twd_p to save twd_r
    measurements_filtered.twd_p[measurements_filtered.oldestValueTwd] = twd_r;

    //update oldest value index
    measurements_filtered.oldestValueTwd++;

    if(measurements_filtered.oldestValueTwd >= measurements_filtered.twd_window_length)
        measurements_filtered.oldestValueTwd = 0;

    //update twd mean using a robust mean
    measurements_filtered.twd_sns = robust_avg_sns(measurements_filtered.twd_p,
                                                   measurements_filtered.twd_window_length);
}


/**
 * Compute a new value for the alpha angle in our sensor frame.
 *
 * Compute a convex combination of alpha_cog and alpha_yaw based on time, then
 * update the vector containing the old value of alpha and use
 * @see robust_avg_sns() with these values.
*/
void update_alpha_sns(void){

    float alpha_lambda_sns;
    float lambda;

    //compute the convex combination between alpha_cog and alpha_yaw
    lambda = compute_lambda();
    alpha_lambda_sns = (1.0f - lambda) * measurements_raw.alpha_cog_sns +
                    lambda * measurements_raw.alpha_yaw_sns;

    //save new alpha_lambda by deleting the oldest value
    measurements_filtered.alpha_p[measurements_filtered.oldestValue] = alpha_lambda_sns;

    //update oldest value index
    measurements_filtered.oldestValue++;


    if(measurements_filtered.oldestValue >= measurements_filtered.alpha_window_length)
        measurements_filtered.oldestValue = 0;

    //use @see robust_avg_sns function to compute a robust mean of alpha angle
    measurements_filtered.alpha_sns = robust_avg_sns(measurements_filtered.alpha_p,
                                                     measurements_filtered.alpha_window_length);

}

/** Return the average value of alpha computed from the last k values
 *
 * The angle with respect to the wind (alpha) is obtained from the
 * application of a moving average filter to alpha_lambda.
 *
 * @return alpha angle (angle with respect to the true wind) in Dumas's convention.
*/
float cd_get_alpha_dumas(void){

    //alpha in Dumas' convention is opposite to alpha in our sensor frame
    return -measurements_filtered.alpha_sns;
}

/** Return the average value of apparent wind direction computed from the last k_app values
 *
 * @return moving average value of apparent wind direction in sensor frame
*/
float cd_get_app_wind_sns(void){

    return measurements_filtered.apparent_wind_sns;
}

/** Return the average value of true wind direction computed from the last k_twd values
 *
 * @return moving average value of true wind direction in sensor frame
*/
float cd_get_twd_sns(void){

    return measurements_filtered.twd_sns;
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
 * Update yaw angle  and yawRate value.
 *
 * @param yaw_r heading angle provided by the Kalman filter.
 * @param yaw_aret_r yaw rate value provided by the Kalman filter.
*/
void cd_update_raw_yaw_yaw_rate(float yaw_r, float yaw_rate_r){
    //compute a new value for alpha_yaw using yaw_r
    measurements_raw.alpha_yaw_sns = compute_instant_alpha_sns(yaw_r);

    //save yawRate
    measurements_raw.yaw_rate_sns = yaw_rate_r;

    //since there is a new value of alpha_yaw, update the alpha_sns value
    update_alpha_sns();
}

/**
 * Get alpha angle (angle with respet to the wind) using yaw angle.
 *
 * This function computes alpha = yaw - twd.
 * The results is in Dumas' convention.
 *
 * @return alpha_yaw in Dumas' convention
*/
float cd_get_alpha_yaw_dumas(void){

    /* Since Dumas' convention is opposite to our sensor convention,
     * just change the sign of alpha_yaw_sns
     */
    return -measurements_raw.alpha_yaw_sns;
}

/**
 * Compute lambda coefficient based on how much time has elapsed
 * since the last COG value was updated from the gps.
 *
 * This lambda value should be used to compute a mean alpha like:
 * alpha = (1 - lambda) * alpha_cog + lambda * alpha_yaw
*/
float compute_lambda(void){

    uint64_t diff_us;
    float diff_norm;
    float lambda = 0.0f;

    /* compute the time difference from the actual time and the
     * last time the cog value was updated (@see update_raw_cog) and
     * normalize max_time_cog_not_up
    */
    diff_us = hrt_absolute_time() - measurements_raw.time_last_cog_update;
    diff_norm = diff_us / ((float)user_params.max_time_cog_not_up);

    if(diff_us >= user_params.max_time_cog_not_up){
        lambda = 1.0f;
    }
    else{
        lambda = diff_norm;
    }

    //make sure lambda is between 0 and 1
    if(lambda < 0.0f)
        lambda = 0.0f;
    else if(lambda > 1.0f)
        lambda = 1.0f;

    return lambda;
}

/**
 * Get latest yaw rate value in sensor frame.
*/
float cd_get_yaw_rate_sns(void){
    return measurements_raw.yaw_rate_sns;
}

/**
 * Notify to controller_data module that a tack performed by either LQR or MPC has
 * just started.
 *
 * This function changes the moving average windows size for alpha angle and twd, using
 * the window size set through @see update_k (second param) and @see update_k_twd (second param).
 *
*/
void cd_optimal_tack_started(void){
    //safety check
    if(measurements_filtered.opt_tack_going == false){
        measurements_filtered.opt_tack_going = true;

        //copy in the whole alpha_p the last average value of alpha
        fill_fvector(measurements_filtered.alpha_p, measurements_filtered.k, measurements_filtered.alpha_sns);

        //copy in the whole twd_p the last average value of twd
        fill_fvector(measurements_filtered.twd_p, measurements_filtered.k_twd, measurements_filtered.twd_sns);

        //use the window size for the optimal tack for alpha and twd
        measurements_filtered.alpha_window_length = measurements_filtered.opt_tack_alpha_win;
        measurements_filtered.twd_window_length = measurements_filtered.opt_tack_twd_win;

        //start filling the two sample vectors from the first position
        measurements_filtered.oldestValue = 0;
        measurements_filtered.oldestValueTwd = 0;
    }

}

/**
 * Notify to controller_data module that a tack performed by either LQR or MPC has
 * just been completed.
 *
 * This function changes the moving average windows size for alpha angle and twd, using
 * the original window size set through @see update_k (first param) and @see update_k_twd (first param).
*/
void cd_optimal_tack_completed(void){
    //safety check
    if(measurements_filtered.opt_tack_going == true){
        measurements_filtered.opt_tack_going = false;

        //copy in the whole alpha_p the last average value of alpha
        fill_fvector(measurements_filtered.alpha_p, measurements_filtered.k, measurements_filtered.alpha_sns);

        //start filling app_p with new values from the first position
        measurements_filtered.oldestValue = 0;

        //copy in the whole twd_p the last average value of twd
        fill_fvector(measurements_filtered.twd_p, measurements_filtered.k_twd, measurements_filtered.twd_sns);

        //start filling twd_p with new values from the first position
        measurements_filtered.oldestValueTwd = 0;

        //reset the original window size for alpha and twd
        measurements_filtered.alpha_window_length = measurements_filtered.k;
        measurements_filtered.twd_window_length = measurements_filtered.k_twd;
    }

}

/**
 * Fill a float vector with a constant value
*/
void fill_fvector(float* vector, uint16_t vector_size, float value){
    for(uint16_t i = 0; i < vector_size; i++)
        vector[i] = value;
}

/**
 * Set if alpha angle must be computed using the twd provided by the moving average
 * filter @see get_twd_sns() or using a fixed value as twd.
 *
 * @param use_fixed_twd 0 if you want to use twd from moving filter
 */
void cd_use_fixed_twd(int32_t use_fixed_twd){

    //check if the user has decided to use or not to use a fixed true wind direction
    if((use_fixed_twd == 0 && user_params.use_fixed_twd == true) ||
       (use_fixed_twd == 1 && user_params.use_fixed_twd == false)){

        //update and print info on QGC
        user_params.use_fixed_twd = (use_fixed_twd == 0) ? false : true;
        print_fixed_twd_info();
    }
}

/**
 * Set if alpha angle must be computed using the twd provided by the moving average
 * filter @see get_twd_sns() or using a fixed value as twd.
 *
 * @param fixed_twd_r mean twd, [rad], in our sensor convention.
 */
void cd_set_mean_twd(float fixed_twd_r){

    //make sure fixed_twd_r is in [-pi, pi]
    if(fixed_twd_r > M_PI_F)
        fixed_twd_r = M_PI_F;
    else if(fixed_twd_r < - M_PI_F)
        fixed_twd_r = -M_PI_F;


    //check if the user has changed the fixed_twd
    if(fixed_twd_r != user_params.fixed_twd_r){

        //update and print info on QGC
        user_params.fixed_twd_r = fixed_twd_r;
        print_fixed_twd_info();
    }
}

/**
 * Print on QGC usefull info if the user has just decided to use or not to use a fixed true wind
 * direction to compute alpha.
 */
void print_fixed_twd_info(void){


    if(user_params.use_fixed_twd == false){
        //new value for user_params.use_fixed_twd, send a msg to QGC
        smq_send_log_info("Use TWD from moving avg filt.");
    }
    else if(user_params.use_fixed_twd == true){
        //new value for user_params.use_fixed_twd, send a msg to QGC
        sprintf(txt_msg, "Use fixed TWD = %0.1f [deg]", (double)(user_params.fixed_twd_r * rad2deg));
        smq_send_log_info(txt_msg);
    }
}
