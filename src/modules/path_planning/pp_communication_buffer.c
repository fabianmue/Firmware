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

static struct path_planning_s pp;

static bool doing_maneuver = false; //is boat doing a maneuver ?
static bool send_maneuver_cmd = false;//do we have to send a do_maneuver command?
static bool pp_updated = false;//has pp been updated ?

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
    if(doing_maneuver == false){

        //set new alpha_star
        cb_set_alpha_star(new_alpha_star);

        //do_maneuver
        pp.do_maneuver = true;

        //remember we have to send the meneuver command to autonomous_sailing app
        send_maneuver_cmd = true;
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
    /*
     * Return true only if the boat is not doing a maneuver AND
     * if we do not have to send a do_maneuver command to
     * autonomous_sailing app.
    */
    if(doing_maneuver == false && send_maneuver_cmd == false)
        return true;
    else
        return false;
}

/**
 * Based on the data publisehd by autonomous_sailing app see if an
 * already started maneuver has just been completed.
 *
 * @param strs_p    struct containing boat_guidance_debug_s struct
*/
void cb_new_as_data(const struct structs_topics_s *strs_p){

    // If we sent a do_maneuver command, check if the maneuver is completed
    if(doing_maneuver == true && send_maneuver_cmd == false){
        //is the maneuver completed?
        if(strs_p->boat_guidance_debug.maneuver_completed == 1){
            doing_maneuver == false;

            pp.do_maneuver = false;
            pp_updated = true;
        }
    }
}

/**
 * If either at least a set function in this module has been called,
 * or the @see cd_do_maneuver() has been called, publish path_planning topic.
*/
void cb_publish_pp_if_updated(void){
    //has pp struct been updated? If so, publish it
    if(pp_updated == true){

        th_publish_path_planning(&pp);
        pp_updated = false;

        //have we just sent a do_maneuver_command?
        if(send_maneuver_cmd == true){
            //if so, the boat will starting maneuvering soon
            send_maneuver_cmd = false;
            doing_maneuver = true;
            smq_send_log_info("Starting maneuver.");
        }
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

    //set new alpha if the boat is not maneuvering
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
