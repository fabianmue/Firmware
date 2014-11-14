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
#include "path_planning_data.h"


//Include topics necessary
#include "topics_handler.h"



// To be able to use the "parameter function" from Q ground control:
#include <systemlib/param/param.h>
#include <systemlib/systemlib.h>

#define DAEMON_PRIORITY SCHED_PRIORITY_MAX - 10 ///daemon priority

#define TIMEOUT_1SEC 1000


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

/** @brief Convert GPS data in position in Race frame coordinate*/
void navigation_module(const struct structs_topics_s *strs_p,
                       struct local_position_race_s *lp_p);

/** @brief Give next optimal action to be implemented*/
void path_planning();

/** @brief Implement next control action*/
void guidance_module(struct reference_actions_s *ref_act_p,
                     const struct apparent_angle_s *apparent_angle_p);

/** @brief Initialize parameters*/
void param_init(struct pointers_param_qgc *pointers_p,
                struct parameters_qgc *params_p,
                struct apparent_angle_s *apparent_angle_p);

/** @brief Check if one or more parameters have been updated and perform appropriate actions*/
void param_check_update(struct pointers_param_qgc *pointers_p,
                        struct parameters_qgc *params_p,
                        struct apparent_angle_s *apparent_angle_p);

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

    //local position in the Race frame
    struct local_position_race_s local_pos_r = {.x_race_cm = 0, .y_race_cm = 0};

    //optimal path parameters
    struct reference_actions_s ref_act = {.alpha_star = 30.0, .should_tack = false};

    //apparent wind angle history
    struct apparent_angle_s apparent_angle;

    warnx(" starting\n");

    //subscribe to interested topics
    as_subscriber(&subs);

    //initialize parameters
    param_init(&pointers_param, &params, &apparent_angle);

	// try to initiliaze actuators
    if(!actuators_init(&pubs, &strs)){
		// something went wrong
		thread_should_exit = true;
        warnx(" problem in initializing actuators\n");
	}

    // polling management
    struct pollfd fds[] = {
            { .fd = subs.att ,                  .events = POLLIN },
            { .fd = subs.gps ,                  .events = POLLIN },
            { .fd = subs.wind_sailing,          .events = POLLIN },
            { .fd = subs.boat_weather_station,  .events = POLLIN }
    };

    //set reference of NED frame before starting
    set_ref0(&(params.lat0), &(params.lon0), &(params.alt0));

    thread_running = true;

    while(!thread_should_exit){

        poll_ret = poll(fds, (sizeof(fds) / sizeof(fds[0])), TIMEOUT_1SEC);

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
                    orb_copy(ORB_ID(vehicle_attitude), subs.att, &(strs.att));

                }
                if(fds[1].revents & POLLIN){
                    // new vehicle_global_position value

                    //copy GPS data
                    orb_copy(ORB_ID(vehicle_global_position), subs.gps, &(strs.gps_filtered));

                    //do navigation module
                    navigation_module(&strs, &local_pos_r);

                    //do optimal path planning
                    path_planning();

                }
                if(fds[2].revents & POLLIN){
                    // new WSAI values, copy new data
                    orb_copy(ORB_ID(wind_sailing), subs.wind_sailing, &(strs.wind_sailing));

//                    //update apparent wind angle history by substituing oldest value
//                    apparent_angle.app_angle_p[apparent_angle.oldest_value] = strs.wsai.angle_apparent;
//                    //update index of oldest value
//                    apparent_angle.oldest_value++;
//                    if(apparent_angle.oldest_value == apparent_angle.window_size)
//                        apparent_angle.oldest_value = 0;
//                    //compute apparent wind angle mean
//                    apparent_angle.app_angle_mean = 0.0f;
//                    for(int i = 0; i < apparent_angle.window_size; i++){
//                        apparent_angle.app_angle_mean += apparent_angle.app_angle_p[i];
//                    }

                }
                if(fds[3].revents & POLLIN){
                    // new boat_weather_station values

                    // copy new data
                    orb_copy(ORB_ID(boat_weather_station), subs.boat_weather_station, &(strs.boat_weather_station));

                }

            }
        }

        //check if any parameter has been updated
        param_check_update(&pointers_param, &params, &apparent_angle);

        //always perfrom guidance module to control the boat
        guidance_module(&ref_act, &apparent_angle);

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

    subs_p->att = orb_subscribe(ORB_ID(vehicle_attitude));
    subs_p->gps = orb_subscribe(ORB_ID(vehicle_global_position));
    subs_p->boat_weather_station = orb_subscribe(ORB_ID(boat_weather_station));
    subs_p->wind_sailing = orb_subscribe(ORB_ID(wind_sailing));

    if(subs_p->att == -1){
        warnx(" error on subscribing on vehicle_attitude Topic \n");
        return false;
    }

    if(subs_p->gps == -1){
        warnx(" error on subscribing on vehicle_global_position Topic \n");
        return false;
    }

    if(subs_p->boat_weather_station == -1){
        warnx(" error on subscribing on boat_weather_station Topic \n");
        return false;
    }

    if(subs_p->wind_sailing == -1){
        warnx(" error on subscribing on wind_sailing Topic \n");
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
                struct parameters_qgc *params_p,
                struct apparent_angle_s *apparent_angle_p){

    //initialize pointer to parameters
    pointers_p->sail_pointer    = param_find("AS_SAIL");
    pointers_p->rudder_pointer  = param_find("AS_RUDDER");

    pointers_p->p_gain_pointer  = param_find("AS_P_GAIN");
    pointers_p->i_gain_pointer  = param_find("AS_I_GAIN");

    pointers_p->lat0_pointer    = param_find("AS_LAT0");
    pointers_p->lon0_pointer    = param_find("AS_LON0");
    pointers_p->alt0_pointer    = param_find("AS_ALT0");

    pointers_p->epsilon_pointer = param_find("AS_EPSI");

    pointers_p->moving_window_pointer = param_find("AS_WIN");

    //get parameters
    param_get(pointers_p->sail_pointer, &(params_p->sail_servo));
    param_get(pointers_p->rudder_pointer, &(params_p->rudder_servo));

    param_get(pointers_p->p_gain_pointer, &(params_p->p_gain));
    param_get(pointers_p->i_gain_pointer, &(params_p->i_gain));

    param_get(pointers_p->lat0_pointer, &(params_p->lat0));
    param_get(pointers_p->lon0_pointer, &(params_p->lon0));
    param_get(pointers_p->alt0_pointer, &(params_p->alt0));

    param_get(pointers_p->epsilon_pointer, &(params_p->epsilon));

    param_get(pointers_p->moving_window_pointer, &(params_p->moving_window));

    //create array for apparent wind angle history
    apparent_angle_p->window_size = params_p->moving_window;
    apparent_angle_p->oldest_value = 0;
    apparent_angle_p->app_angle_p = malloc(sizeof(float) * apparent_angle_p->window_size);

    for(int i = 0; i < apparent_angle_p->window_size; i++)
        apparent_angle_p->app_angle_p[i] = 0.0f;
}

/** Check if any paramter has been updated, if so take appropriate actions
 *
*/
void param_check_update(struct pointers_param_qgc *pointers_p,
                        struct parameters_qgc *params_p,
                        struct apparent_angle_s *apparent_angle_p){

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

    //check moving window
    param_get(pointers_p->moving_window_pointer, &app_i);
    if(params_p->moving_window != app_i){
        params_p->moving_window = app_i;

        //delete old apparent wind angle history
        free(apparent_angle_p->app_angle_p);

        //create array for apparent wind angle history
        apparent_angle_p->window_size = params_p->moving_window;
        apparent_angle_p->oldest_value = 0;
        apparent_angle_p->app_angle_p = malloc(sizeof(float) * apparent_angle_p->window_size);

        for(int i = 0; i < apparent_angle_p->window_size; i++)
            apparent_angle_p->app_angle_p[i] = 0.0f;
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

/** Implement refernce actions provided by optimal path planning*/
void guidance_module(struct reference_actions_s *ref_act_p,
                     const struct apparent_angle_s *apparent_angle_p){


}

/**
 * Compute from vehicle_global_position topic the boat's position in Race frame. Set up the next target position.
 *
*/
void navigation_module(const struct structs_topics_s *strs_p,
                       struct local_position_race_s *lp_p){

    int32_t north_cm;
    int32_t east_cm;
    int32_t down_cm;


    //compute boat position in NED frame w.r.t. lat0 lon0 alt0 set by set_ref0()
    geo_to_ned(&(strs_p->gps_filtered), &north_cm, &east_cm, &down_cm);

    //TODO NED to Race frame.
    lp_p->x_race_cm = east_cm;
    lp_p->y_race_cm = north_cm;

}

/** Retrieve data from pre-computed path planning and give the next references*/
void path_planning(){

}
