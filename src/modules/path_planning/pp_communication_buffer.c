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
 * @file pp_communication_buffer.h
 *
 * API to tell the boat it should tack or jybe
 *
 * @author Marco Tranzatto <marco.tranzatto@gmail.com>
 */

#include "pp_communication_buffer.h"
#include <drivers/drv_hrt.h>
#include <stdio.h>
#include <string.h>

#define M_PI_F 3.14159265358979323846f

#define SGN(X) ((X) < (0.0f) ? (-1.0f) : (1.0f))

#define MIN_TS_GO_DOWNWIND_US 150000 //150 milliseconds

//state variables
static struct path_planning_s pp;
static bool pp_updated = false; //has pp been updated ?

static struct mi_ack_s mi_ack;
static bool mi_ack_updated = false;

static bool manual_mode = false; //is remote control in manual mode?, true, if in manual mode
static uint8_t last_haul = HAUL_PORT;//dummy initial guess

static bool change_alpha_star = false;//use it only after reaced last grid line

//int32_t boat_ned[3];//boat NED coordinates
//data from autonomous_sailing app
static struct boat_guidance_debug_s boat_guidance_debug;

#if USE_GRID_LINES == 1
static float alpha_star_vel_r_s = 0.2f;//velocity of changing alpha_star when reached last grid line
static uint64_t last_change_alpha_star = 0;
static uint64_t now = 0;
static float downwind_alpha_star_abs = 2.7925268f;
static char txt_msg[70]; ///used to send messages to QGC
#endif

//uint64_t last_update = 0;



/**
 * Set Failsafe state
 * Communicate to Autonomous Sailing app, that we need to go into failsafe mode
 */
bool cb_set_failsafe(bool state) {

	pp.failsafe = state;

	pp_updated = true;

	return true;
}

/**
 * Get the mode we are in at the moment (manual/autonomous)
 *
 * @return true, if we are in autonomous mode
 */
bool cb_is_autonomous_mode(void) {
	if(manual_mode == true) {
		//we are in autonomous mode => return true
		return false;
	} else {
		//we are in manual mode => return false
		return true;
	}
}

/**
 * Store the current Position of the boat in NED-Coordinates in the Pathplanning Topic
 * => This is used for debugging in QGround Control! (Added by Jonas Wirz)
 *
 * @param north, east position of the boat in NED-Frame [m]
 */
bool cb_new_position(float north, float east) {
	pp.ned_east = east;
	pp.ned_north = north;

	pp_updated = true;

	return true;
}

/**
 * Store the current Heading of the boat in Pathplanning Topic
 * => This is used for debugging in QGround Control! (Added by Jonas Wirz)
 *
 * @param heading: Current heading of the boat in Compass Frame [rad]
 */
bool cb_new_heading(float heading) {
	pp.heading = heading;

	pp_updated = true;

	return true;
}

/**
 * Return the current Heading of the boat from the Pathplanning Topic
 * Note: The heading is calculated in navigator.c
 * The heading is a convex combination from the COG and the Yaw-Angle dependent on the update rate
 *
 * @return heading in Compass Frame [rad]
 */
float cb_get_heading(void) {

	return pp.heading;
}

/**
 * Store the current Reference Heading of the boat in Pathplanning Topic => For DEBUG purposes
 * => This is used for debugging in QGround Control! (Added by Jonas Wirz)
 *
 * @param ref_heading: The reference heading in compass frame [rad]
 */
bool cb_new_refheading(float ref_heading) {
	pp.ref_heading = ref_heading;

	pp_updated = true;

	return true;
}

/**
 * Store the current Heading of the boat in Pathplanning Topic
 * => This is used for debugging in QGround Control! (Added by Jonas Wirz)
 *
 * @param wind in compass frame [rad]
 */
bool cb_new_wind(float wind) {
	pp.wind = wind;

	pp_updated = true;

	return true;
}

/* @brief send new mission_id */
bool cb_new_mission(int id) {

	pp.mi_id = id;

	pp_updated = true;
	return true;
}

/* @brief send new target position */
bool cb_new_target(Point tar) {

	pp.tar_lat = tar.lat;
	pp.tar_lon = tar.lon;
	pp.tar_num++;

	pp_updated = true;
	return true;
}

/* @brief increase obstacle number */
bool cb_new_obstacle(Point obs) {

	pp.obs_lat = obs.lat;
	pp.obs_lon = obs.lon;
	pp.obs_num++;

	pp_updated = true;
	return true;
}

bool cb_new_obs_ack(bool ack) {

	mi_ack.obs_ack = ack;

	mi_ack_updated = true;
	return true;
}

bool cb_new_tar_ack(bool ack) {

	mi_ack.tar_ack = ack;

	mi_ack_updated = true;
	return true;
}

/**
 * Tell autonomous_sailing app to start a tack or jybe maneuver.
 * You should call this funcion only if the boat is not doing another maneuver.
 * Use @see cb_is_maneuver_completed() to ask if the boat is doing a meneuver.
 *
 * @param  new_alpha_star   new alpha_star after the meneuver
 * @return                  true if the command was sent to autonomous_sailing app
 */
bool cb_do_maneuver(float new_alpha_star){
    bool res;

    //we can command a new maneuver only if the boat is not perfoming another one yet
    if(cb_is_maneuver_completed() == true){

        //set new alpha_star
        cb_set_alpha_star(new_alpha_star);

        //send do_maneuver command to autonomous_sailing app
        pp.do_maneuver = 1;

        //give a new Id for this new maneuver
        if(pp.id_maneuver == 255)
            pp.id_maneuver = 0;
        else
            pp.id_maneuver = pp.id_maneuver + 1;

        //remember we have to send the meneuver command to autonomous_sailing app
        pp_updated = true;
        res = true;
    }
    else
        res = false;

    return res;
}

/**
 * Ask if an already started maneuver has been completed.
 *
 * @return      true if the boat is not doing or has to do any maneuver
*/
bool cb_is_maneuver_completed(void){
    return (pp.do_maneuver == 1) ? false : true;
}

/**
 * Based on the data publisehd by autonomous_sailing app see if an
 * already started maneuver has just been completed.
 * If the remote control is set to manual mode, read the current haul
 * and set alpha_star to have the same haul. The magnitude of
 * alpha_star is not changed.
 *
 * @param boat_guidance_debug_sub subscription to boat_guidance_debug
*/
void cb_new_as_data(int boat_guidance_debug_sub){

    //copy new BGUD data
    orb_copy(ORB_ID(boat_guidance_debug), boat_guidance_debug_sub,
             &boat_guidance_debug);

    // check if we sent a do_maneuver command
    if(cb_is_maneuver_completed() == false){
        // check if the maneuver has the same id of the sent one and it is completed
        if(boat_guidance_debug.maneuver_completed == 1 &&
           boat_guidance_debug.id_maneuver == pp.id_maneuver){

            //maneuver is completed
            pp.do_maneuver = 0;
            pp_updated = true;
        }
    }
    //if the remote control is in manual mode, set the sign of alpha_star based on the actual haul
    if(manual_mode){
        //get the most updated haul from data provided by autonomous_sailing app
        uint8_t haul_tmp = cb_get_haul();
        //if we have changed haul since the last time, update alpha_star
        if(haul_tmp != last_haul){
            float temp_alpha_star = (haul_tmp == HAUL_PORT) ?
                                     -fabsf(pp.alpha_star) : fabsf(pp.alpha_star);
            //set the new alpha_star
            cb_set_alpha_star(temp_alpha_star);
        }
        //update last_haul
        last_haul = haul_tmp;
    }
}

/**
 * If either at least one parameter in path-planning topic
 * has been changed or when have to reach "sail downwind" position,
 * publish it.
*/
void pp_cb_publish_if_updated(void){

    //do we need to go in the "downwind position" by changing alpha_star ?
    #if USE_GRID_LINES == 1
    if(change_alpha_star){
        go_downwind();
    }
    #endif //USE_GRID_LINES == 1

    //if path_planning topic has been updated, publish it
    //Make sure this happens not too often, otherwise the processor load is too high
    //uint64_t utime = hrt_absolute_time();

    if (pp_updated == true /*&& last_update-utime > 0.25e-6*/) {
    	//last_update = time;

        pp.timestamp = hrt_absolute_time();
        pp_th_publish(&pp);
        pp_updated = false;
    }
    if (mi_ack_updated == true) {

        mi_ack.timestamp = hrt_absolute_time();
        pp_th_publish_mi_ack(&mi_ack);
        mi_ack_updated = false;
    }
}

/**
 * Set a new reference for alpha.
 * Available only if the boat is not (or will not starting) doing a maneuver.
 *
 * @param new_alpha_star    new alpha star in Dumas' convention, [rad]
*/
bool cb_set_alpha_star(float new_alpha_star){

    bool res = true;

    //make sure -pi <= alpha <= pi
    if(new_alpha_star > M_PI_F)
        new_alpha_star = M_PI_F;
    else if(new_alpha_star < -M_PI_F)
        new_alpha_star = -M_PI_F;

    // set new alpha if the boat is not maneuvering
    if(cb_is_maneuver_completed() == true){
        pp.alpha_star = new_alpha_star;
        pp_updated = true;
    }
    else
        res = false;

    return res;
}

/**
 * Init pp_communication_buffer.
*/
void pp_cb_init(void){

    //clean memory
    memset(&pp, 0, sizeof(pp));
    memset(&boat_guidance_debug, 0, sizeof(boat_guidance_debug));
    memset(&mi_ack, 0, sizeof(mi_ack));

    //default starting maneuver id = 255
    pp.id_maneuver = 255;
    pp.id_cmd = PP_NORMAL_CMD;
    //default alpha_star = 45 deg
    cb_set_alpha_star(M_PI_F / 4.0f);

    //Added 19.06.15, commented 02.09.2015
    pp_updated = true;
    mi_ack_updated = true;
    pp_cb_publish_if_updated();
}

/**
 * Get current alpha_star value.
 *
 * @return actual alpha_star value
*/
float cb_get_alpha_star(void){
    return pp.alpha_star;
}

/**
 * Check if the remote control is in manual mode.
*/
void cb_new_rc_data(const struct pp_structs_topics_s *strs_p){

    if(strs_p->rc_channels.channels[RC_MODE_INDEX] == RC_MANUAL_MODE){
        manual_mode = true;
        //make sure that when we will switch to autonomous, we'll use a TWD from moving filter
        cb_use_fixed_twd(false);
        change_alpha_star = false;
    }
    else{

        #if USE_GRID_LINES == 1
        /* if we have just switched from manual to autonomous,
         * print the next grid line we want to reach, if there is at
         * least one.
         * Remember: if we have just switched, manual_mode is not
         * yet updated to false, but since we are here, the mode is
         * set to autonomous.
        */
        if(manual_mode == true){
            float next_grid;
            if(gh_get_next_gridline(&next_grid)){
                //send a message to QGC
                sprintf(txt_msg, "Next grid: %0.1f [m]", (double) next_grid);
                smq_send_log_info(txt_msg);
            }
        }
        #endif //USE_GRID_LINES == 1

        //update manual_mode variable
        manual_mode = false;
    }



}

/**
 * Get the current haul of the boat
 *
 * @return HAUL_PORT if sailing at port haul, HAUL_STARBOARD otherwise
*/
uint8_t cb_get_haul(void){
    return (boat_guidance_debug.alpha < 0.0f) ?
            HAUL_PORT : HAUL_STARBOARD;
}

/**
 * Get the last alpha angle provided by autonomous_sailing app.
 * In Duma's convention ?? (most probably, JW)
 *
 * @return alpha angle in rads, range [-pi, pi]
*/
float cb_get_alpha(void){
    return boat_guidance_debug.alpha;
}

/**
 * Get the last true wind direction (TWD) and speed (TWS) provided
 * by autonomous_sailing app. (In Sensor convention)
 *
 * @param  *twd_p <-- true wind direction [rad] in our sensor convention
 * @param  *tws_p <-- true wind speed [m/s]
*/
void cb_get_tw_info(float *twd_p, float *tws_p){
    *twd_p = boat_guidance_debug.twd_mean;
    *tws_p = boat_guidance_debug.tws_mean;
}


/**
 * Tell the boat to tack as soon as possibile.
 * Change the sign of alpha_star, but not it's magnitude.
*/
bool cb_tack_now(void){
    return cb_do_maneuver(-cb_get_alpha_star());
}

/**
 * Tell autonomous_sailing app to use (or not to use) a fixed
 * true wind direction to compute alpha.
 * The fixed true wind direction used is the last true wind direction mean
 * read.
 *
 * @param use_fixed_wind    true if as app has to use a fixed TWD
*/
void cb_use_fixed_twd(bool use_fixed_twd){

    if(use_fixed_twd == true && pp.id_cmd != PP_SAIL_DOWNWIND_CMD){
        pp.id_cmd = PP_SAIL_DOWNWIND_CMD;
        pp_updated = true;
    }
    else if(use_fixed_twd == false && pp.id_cmd == PP_SAIL_DOWNWIND_CMD){
        pp.id_cmd = PP_NORMAL_CMD;
        pp_updated = true;
    }
}

#if USE_GRID_LINES == 1

/**
 * Change alpha_star in order to sail downwind.
 * In order to not overload the communication between path_planning
 * and autonomous_sailing app, alpha_star is changed with a min
 * priod of MIN_TS_GO_DOWNWIND_US.
*/
void go_downwind(){
    uint64_t time_elapsed;

    now = hrt_absolute_time();
    time_elapsed = now - last_change_alpha_star;

    //check if we can update alpha_star
    if(time_elapsed >= MIN_TS_GO_DOWNWIND_US){

        float delta_alpha;
        float new_alpha_star;

        //update alpha_star based on the current haul and alpha_star velocity
        delta_alpha = alpha_star_vel_r_s * (time_elapsed / 1e6f);

        new_alpha_star = pp.alpha_star + SGN(pp.alpha_star) * delta_alpha;

        //check if we don't have to increase alpha_star anymore
        if(fabsf(new_alpha_star) >= downwind_alpha_star_abs){
            new_alpha_star = SGN(new_alpha_star) * downwind_alpha_star_abs;
            change_alpha_star = false;
        }

        cb_set_alpha_star(new_alpha_star);

        last_change_alpha_star = now;
    }
}

/**
 * Set the velocity that should be used to change alpha_star when the boat
 * has reached the last grid line.
 *
 * @param  vel_r_s  positive velocity in rad/s
*/
void cb_set_alpha_star_vel(float vel_r_s){
    alpha_star_vel_r_s = vel_r_s;
}


/**
 * If the last grid line has been reached, calling this function will
 * make autonomous_sailing app sailing downwind
*/
void cb_reached_last_griline(void){

    //start changing alpha_star
    change_alpha_star = true;
    last_change_alpha_star = hrt_absolute_time();
    //tell autonomous_sailing app we want to sail downwind using a fixed wind direction
    cb_use_fixed_twd(true);
}

/**
 * Set what should be the alpha_star value when sailing downwind after
 * the last grid line has been reached.
 *
 * @param  alpha_star   alpha star value [rad]
*/
void cb_set_downwind_alpha_star(float alpha_star){
    //safety check
    if(alpha_star > M_PI_F)
        alpha_star = M_PI_F;
    else if(alpha_star < -M_PI_F)
        alpha_star = -M_PI_F;

    downwind_alpha_star_abs = fabsf(alpha_star);
}

#endif //USE_GRID_LINES == 1
