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

#define M_PI_F 3.14159265358979323846f

#define SGN(X) ((X) < (0.0f) ? (-1.0f) : (1.0f))

//state variables
static struct path_planning_s pp;
static bool pp_updated = false;//has pp been updated ?
static bool manual_mode = false; //is remote control in manual mode?
static uint8_t haul_current = HAUL_PORT;//dummy initial guess
static float alpha_star_vel_r_s = 0.2f;//velocity of changing alpha_star when reached last grid line
static bool change_alpha_star = false;//use it only after reaced last grid line
static uint64_t last_change_alpha_star = 0;
static uint64_t now = 0;

static char txt_msg[70]; ///used to send messages to QGC

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
 * @param strs_p    struct containing boat_guidance_debug_s struct
*/
void cb_new_as_data(const struct structs_topics_s *strs_p){

    // check if we sent a do_maneuver command
    if(cb_is_maneuver_completed() == false){
        // check if the maneuver has the same id of the sent one and it is completed
        if(strs_p->boat_guidance_debug.maneuver_completed == 1 &&
           strs_p->boat_guidance_debug.id_maneuver == pp.id_maneuver){

            //maneuver is completed
            pp.do_maneuver = 0;
            pp_updated = true;
        }
    }
    //if the remote control is in manual mode, set the sign of alpha_star based on the actual haul
    if(manual_mode){
        //get the most updated haul from data provided by autonomous_sailing app
        uint8_t haul_tmp = (strs_p->boat_guidance_debug.alpha < 0.0f) ?
                            HAUL_PORT : HAUL_STARBOARD;
        //if we have changed haul since the last time, update alpha_star
        if(haul_tmp != haul_current){
            float temp_alpha_star = (haul_tmp == HAUL_PORT) ?
                                     -fabsf(pp.alpha_star) : fabsf(pp.alpha_star);
            //set the new alpha_star
            cb_set_alpha_star(temp_alpha_star);
        }
    }

    //save the current haul
    haul_current = (strs_p->boat_guidance_debug.alpha < 0.0f) ?
                    HAUL_PORT : HAUL_STARBOARD;
}

/**
 * If either at least one parameter in path-planning topic
 * has been changed, publish it.
*/
void cb_publish_pp_if_updated(void){
    //has pp struct been updated or must alpha_star be changed?
    if(pp_updated == true || change_alpha_star){

        if(change_alpha_star){
            go_downwind();
        }

        th_publish_path_planning(&pp);
        pp_updated = false;
    }
}

/**
 * Update X and Y coordinate in path_planning topic.
 *
 * @param x_m       X coordinate in the race frame, [m]
 * @param y_m       Y coordinate in the race frame, [m]
 */
void cb_set_race_coordinates(float x_m, float y_m){
    pp.x_race_m = x_m;
    pp.y_race_m = y_m;
    pp_updated = true;
}

/**
 * Set a new reference for alpha.
 * Available only if the boat is not (or will not starting) doing a maneuver.
 *
 * @param new_alpha_star    new alpha star in Dumas' convention, [rad]
*/
bool cb_set_alpha_star(float new_alpha_star){

    bool res = true;

    //make sure abs(alpha) <= pi
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
void cb_init(void){
    //clean memory
    memset(&pp, 0, sizeof(pp));
    //default starting maneuver id = 255
    pp.id_maneuver = 255;
    //default alpha_star = 45 deg
    cb_set_alpha_star(M_PI_F / 4.0f);
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
void cb_new_rc_data(const struct structs_topics_s *strs_p){

    if(strs_p->rc_channels.channels[RC_MODE_INDEX] == RC_MANUAL_MODE)
        manual_mode = true;
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
                sprintf(txt_msg, "Next grid: 0.1%f [m]", (double) next_grid);
                smq_send_log_info(txt_msg);
            }
        }
        #endif

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
    return haul_current;
}

/**
 * Get the last computed X coordinate of the boat in the race frame.
 *
 * @return X coordinate in meters
*/
float cb_get_x_race_m(void){
    return pp.x_race_m;
}

/**
 * Tell the boat to tack as soon as possibile.
 * Change the sign of alpha_star, but not it's magnitude.
*/
bool cb_tack_now(void){
    return cb_do_maneuver(-cb_get_alpha_star());
}


#if USE_GRID_LINES == 1

/**
 * Change alpha_star in order to sail downwind.
*/
void go_downwind(){
    //update alpha_star based on the current haul and alpha_star velocity
    now = hrt_absolute_time();//time in micro seconds
    //TODO FIX THIS PROBLEM, OTHERWISE THE SYSTEN WILL ALWAYS CRUSH!!
    double delta_alpha = (double)alpha_star_vel_r_s * ((now - last_change_alpha_star) / 1e6);
    last_change_alpha_star = now;

    float new_alpha_star;

    new_alpha_star = pp.alpha_star + SGN(pp.alpha_star) * (float)delta_alpha;

    cb_set_alpha_star(new_alpha_star);

    //check if we don't have to increase alpha_star anymore
    if(fabsf(new_alpha_star) >= M_PI_F){
        change_alpha_star = false;
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

    //TODO send special command first time!
}

#endif //USE_GRID_LINES == 1
