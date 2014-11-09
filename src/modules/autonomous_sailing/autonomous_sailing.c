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
 * @file autonomous_sailing.c
 *
 * Main app for controlling the sail boat using data from 200WX weather station.
 *
 * @author Marco Tranzatto <marco.tranzatto@gmail.com>
 */

#include <nuttx/config.h>
#include <unistd.h>
#include <stdio.h>
#include <errno.h>
#include <poll.h>


#include "as_settings.h"
#include "navigation.h"
#include "parameters.c"


//Include topics necessary
#include "topics_handler.h"


//#include <uORB/uORB.h>
//#include <uORB/topics/actuator_controls.h>
//#include <uORB/topics/actuator_armed.h>

//#include <uORB/topics/wind_sailing.h>
//#include <uORB/topics/vehicle_global_position.h>
//#include <uORB/topics/vehicle_bodyframe_meas.h>
//#include <uORB/topics/vehicle_attitude.h>


// To be able to use the "parameter function" from Q ground control:
#include <systemlib/param/param.h>
#include <systemlib/systemlib.h>

#define DAEMON_PRIORITY SCHED_PRIORITY_MAX - 10 ///daemon priority


//Thread management variables
static bool thread_should_exit = false;		/**< daemon exit flag */
static bool thread_running = false;			/**< daemon status flag */
static int daemon_task;						/**< Handle of daemon task / thread */


/**
 * autonomous sailing app start / stop handling function
 *
 * @ingroup apps
 */
__EXPORT int autonomous_sailing_main(int argc, char *argv[]);

/** @brief Mainloop of daemon. */
int as_daemon_thread_main(int argc, char *argv[]);

/** @brief Print the correct usage. */
static void usage(const char *reason);

/** @brief Iinitialize actuators. */
bool actuators_init(struct published_fd_s *pubs_p,
                    struct structs_topics_s *strs_p);

/** @brief Subscribe to appropriate topics. */
bool as_subscriber(struct subscribtion_fd_s *subs_p);

/** @brief Autonomous sailing controller for the rudder. */
float as_rudder_controller();

/** @brief Convert GPS data in position in Race frame coordinate*/
void navigation_module();

/** @brief Decide the next control action to be implemented*/
void guidance_module();

/** @brief Initialize parameters*/
void param_init(struct pointers_param_qgc *pointers_p,
                struct parameters_qgc *params_p);

/** @brief Check if one or more parameters have been updated and perform appropriate actions*/
void param_check_update(struct pointers_param_qgc *pointers_p,
                        struct parameters_qgc *params_p);

//pointers to params from QGroundControl
struct pointers_param_qgc pointers_param;


static void usage(const char *reason)
{
	if (reason)
		warnx("%s\n", reason);
    errx(1, "usage: autonomous_sailing {start|stop|status} [-p <additional params>]\n\n");
}



/**
 * The daemon app only briefly exists to start
 * the background job. The stack size assigned in the
 * Makefile does only apply to this management task.
 *
 * The actual stack size should be set in the call
 * to task_create().
 */
int autonomous_sailing_main(int argc, char *argv[]){

	if (argc < 1)
		usage("missing command");

	if (!strcmp(argv[1], "start")) {

		if (thread_running) {
			warnx("daemon already running\n");
			/* this is not an error */
			exit(0);
		}

		thread_should_exit = false;
        daemon_task = task_spawn_cmd("autonomous_sailing",
									SCHED_DEFAULT,
                                    DAEMON_PRIORITY,
								    4096,
								    as_daemon_thread_main,
					 (argv) ? (const char **)&argv[2] : (const char **)NULL);
		exit(0);
	}

	if (!strcmp(argv[1], "stop")) {
		thread_should_exit = true;
		exit(0);
	}

	if (!strcmp(argv[1], "status")) {
		if (thread_running) {
			warnx("\trunning\n");
		} else {
			warnx("\tnot started\n");
		}
		exit(0);
	}

	usage("unrecognized command");
	exit(1);
}

/**
* Main thread of this app.
*
* DESCRIVERE FUNZIONAMENTO
*/
int as_daemon_thread_main(int argc, char *argv[]){

    //pool return value
    int poll_ret;

    //file descriptors of subscribed topics
    struct subscribtion_fd_s subs;
    //file descriptors of published topics
    struct published_fd_s pubs;
    //structs of interested toics
    struct structs_topics_s strs;
    //parameters from QGroundControl
    struct parameters_qgc params;


//    // boat's coordinates in Race frame
//    int32_t x_cm_race = 0; ///x position of the boat in Race frame, in centimeters.
//    int32_t y_cm_race = 0; ///y position of the boat in Race frame, in centimeters.

//    // next target's coordinates in Race frame, for now set them far away from 0
//    int32_t target_x_cm_race = 100000; ///x position of next target in Race frame, in centimeters.
//    int32_t target_y_cm_race = 100000; ///y position of next target in Race frame, in centimeters.

    //paramters from QGroundControl
//    float rudder_servo;
//    float sail_servo;

//    float p_gain;
//    float i_gain;

//    int32_t lat0;
//    int32_t lon0;
//    int32_t alt0;

//    float epsilon;

    warnx(" starting\n");

    //subscribe to interested topics
    as_subscriber(&subs);

    //initialize parameters
    param_init(&pointers_param, &params);

	// try to initiliaze actuators
    if(!actuators_init(&pubs, &strs)){
		// something went wrong
		thread_should_exit = true;
        warnx(" problem in initializing actuators\n");
	}

    // polling management
    struct pollfd fds[] = {
            { .fd = subs.att_sub ,   .events = POLLIN },
            { .fd = subs.gps_sub ,   .events = POLLIN },
            { .fd = subs.wsai_sub,   .events = POLLIN },
            { .fd = subs.bfme_sub,   .events = POLLIN }
    };

    //set reference of NED frame before starting
    set_ref0(&(params.lat0), &(params.lon0), &(params.alt0));

    thread_running = true;

    while(!thread_should_exit){

        poll_ret = poll(fds, (sizeof(fds) / sizeof(fds[0])), 1000);

        // handle the poll result
        if(poll_ret == 0) {
            // this means none of our providers is giving us data
            warnx(" got no data within a second\n");
        }
        else{
            /* this is undesirable but not much we can do - might want to flag unhappy status */
            if (poll_ret < 0) {
                warn("POLL ERR %d, %d", poll_ret, errno);
                continue;
            }
            else{
                // evrything is ok, at least so far (i.e. pool_ret > 0)
                if(fds[0].revents & POLLIN){
                    // new Attitude values

                    //prova
                    orb_copy(ORB_ID(vehicle_attitude), subs.att_sub, &(strs.att));

                }
                if(fds[1].revents & POLLIN){
                    // new vehicle_global_position value

                    //copy GPS data
                    orb_copy(ORB_ID(vehicle_global_position), subs.gps_sub, &(strs.gps_filtered));

                    //do navigation module
                    navigation_module();

                }
                if(fds[2].revents & POLLIN){
                    // new WSAI values

                    // copy new data
                    orb_copy(ORB_ID(wind_sailing), subs.wsai_sub, &(strs.wsai));
                    // rudder control
                    as_rudder_controller();

                }
                if(fds[3].revents & POLLIN){
                    // new BFME values

                    // copy new data
                    orb_copy(ORB_ID(vehicle_bodyframe_meas), subs.bfme_sub, &(strs.bfme));

                }

                //check if any parameter has been updated
                param_check_update(&pointers_param, &params);
            }
        }

        // Send out commands:
        orb_publish(ORB_ID_VEHICLE_ATTITUDE_CONTROLS, pubs.actuator_pub, &(strs.actuators));
        // actuators.control[0] -> output channel 1
        // actuators.control[2] -> output channel 3
        // actuators.control[3] -> output channel 4
	}

    // kill all outputs
    for (unsigned i = 0; i < NUM_ACTUATOR_CONTROLS; i++)
        strs.actuators.control[i] = 0.0f;
    orb_publish(ORB_ID_VEHICLE_ATTITUDE_CONTROLS, pubs.actuator_pub, &(strs.actuators));

    warnx(" exiting.\n");

	thread_running = false;

	return 0;

}

/**
 * Subscribe each fd to the correspondent topic.
 *
 * @return                 True on succes.
*/
bool as_subscriber(struct subscribtion_fd_s *subs_p){

    subs_p->att_sub = orb_subscribe(ORB_ID(vehicle_attitude));
    subs_p->gps_sub = orb_subscribe(ORB_ID(vehicle_global_position));
    subs_p->bfme_sub = orb_subscribe(ORB_ID(vehicle_bodyframe_meas));
    subs_p->wsai_sub = orb_subscribe(ORB_ID(wind_sailing));

    if(subs_p->att_sub == -1){
        warnx(" error on subscribing on Attitude Topic \n");
        return false;
    }

    if(subs_p->gps_sub == -1){
        warnx(" error on subscribing on GPS Topic \n");
        return false;
    }

    if(subs_p->bfme_sub == -1){
        warnx(" error on subscribing on Body Frame Measurements Topic \n");
        return false;
    }

    if(subs_p->wsai_sub == -1){
        warnx(" error on subscribing on Wind Sailing Topic \n");
        return false;
    }

    warnx(" subscribed to all topics \n");

    return true;
}

/**
* Initialize parameters.
*
*/
void param_init(struct pointers_param_qgc *pointers_p,
                struct parameters_qgc *params_p){

    //initialize pointer to parameters
    pointers_p->sail_pointer    = param_find("AS_SAIL");
    pointers_p->rudder_pointer  = param_find("AS_RUDDER");

    pointers_p->p_gain_pointer  = param_find("AS_P_GAIN");
    pointers_p->i_gain_pointer  = param_find("AS_I_GAIN");

    pointers_p->lat0_pointer    = param_find("AS_LAT0");
    pointers_p->lon0_pointer    = param_find("AS_LON0");
    pointers_p->alt0_pointer    = param_find("AS_ALT0");

    pointers_p->epsilon_pointer = param_find("AS_EPSI");

    //get parameters
    param_get(pointers_p->sail_pointer, &(params_p->sail_servo));
    param_get(pointers_p->rudder_pointer, &(params_p->rudder_servo));

    param_get(pointers_p->p_gain_pointer, &(params_p->p_gain));
    param_get(pointers_p->i_gain_pointer, &(params_p->i_gain));

    param_get(pointers_p->lat0_pointer, &(params_p->lat0));
    param_get(pointers_p->lon0_pointer, &(params_p->lon0));
    param_get(pointers_p->alt0_pointer, &(params_p->alt0));

    param_get(pointers_p->epsilon_pointer, &(params_p->epsilon));
}

/** Check if any paramter has been updated, if so take appropriate actions
 *
*/
void param_check_update(struct pointers_param_qgc *pointers_p,
                        struct parameters_qgc *params_p){

    float app_f;
    int32_t app_i;

    //check sail_servo
    param_get(pointers_p->sail_pointer, &app_f);
    if(params_p->sail_servo != app_f){
        params_p->sail_servo = app_f;
    }

    //check rudder_servo
    param_get(pointers_p->rudder_pointer, &app_f);
    if(params_p->rudder_servo != app_f){
        params_p->rudder_servo = app_f;
    }

    //check p_gain
    param_get(pointers_p->p_gain_pointer, &app_f);
    if(params_p->p_gain != app_f){
        params_p->p_gain = app_f;
    }

    //check i_gain
    param_get(pointers_p->i_gain_pointer, &app_f);
    if(params_p->i_gain != app_f){
        params_p->i_gain = app_f;
    }

    //check lat0
    param_get(pointers_p->lat0_pointer, &app_i);
    if(params_p->lat0 != app_i){
        params_p->lat0 = app_i;
        //update NED origin
        set_ref0(&(params_p->lat0), &(params_p->lon0), &(params_p->alt0));
    }

    //check lon0
    param_get(pointers_p->lon0_pointer, &app_i);
    if(params_p->lon0 != app_i){
        params_p->lon0 = app_i;
        //update NED origin
        set_ref0(&(params_p->lat0), &(params_p->lon0), &(params_p->alt0));
    }

    //check alt0
    param_get(pointers_p->alt0_pointer, &app_i);
    if(params_p->alt0 != app_i){
        params_p->alt0 = app_i;
        //update NED origin
        set_ref0(&(params_p->lat0), &(params_p->lon0), &(params_p->alt0));
    }

    //check epsilon
    param_get(pointers_p->epsilon_pointer, &app_f);
    if(params_p->epsilon != app_f){
        params_p->epsilon = app_f;
    }
}

/**
* Initialize actuators.
*
* @return 							true if everything is OK
*/
bool actuators_init(struct published_fd_s *pubs_p,
                    struct structs_topics_s *strs_p){

	bool right_init = true;
	// initialize actuators arming structure
	struct actuator_armed_s armed;
	armed.armed = true;
	armed.ready_to_arm = true;
	armed.timestamp = hrt_absolute_time();

	// advertise and publish actuators arming initial structure
	orb_advert_t armed_pub = orb_advertise(ORB_ID(actuator_armed), &armed);
	orb_publish(ORB_ID(actuator_armed), armed_pub, &armed);

	//initialize actuators struct
    memset(&(strs_p->actuators), 0, sizeof(strs_p->actuators));
	// set actuator controls values to zero
	for (unsigned i = 0; i < NUM_ACTUATOR_CONTROLS; i++) {
        strs_p->actuators.control[i] = 0.0f;
	}

	// advertise that this controller will publish actuator values
    pubs_p->actuator_pub = orb_advertise(ORB_ID_VEHICLE_ATTITUDE_CONTROLS, &(strs_p->actuators));

	//check if everything was ok
    if(armed_pub == ERROR || armed_pub == -1 || pubs_p->actuator_pub == ERROR || pubs_p->actuator_pub == -1)
		right_init = false; //something went wrong


	return right_init;
}

float as_rudder_controller(){


}

/**
 * Compute from GPS position the boat's position in Race frame. Set up the next target position.
 *
*/
void navigation_module(){

//    int32_t north_cm;
//    int32_t east_cm;
//    int32_t down_cm;


//    //compute boat position in NED frame
//    geo_to_ned(gps_p, &north_cm, &east_cm, &down_cm);
}
