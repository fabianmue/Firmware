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

//navigation module
#include "navigation.h"

//parameters from QGroundControl
#include "parameters.h"

//path planning data computed offline
#include "path_planning.h"

//guidance module
#include "guidance_module.h"

//Include topics necessary
#include "topics_handler.h"

//controller data functions
#include "controller_data.h"

//hardware in the loop simulation
#include "hil_simulation.h"

//settings
#include "settings.h"

// To be able to use the "parameter function" from Q ground control:
#include <systemlib/param/param.h>
#include <systemlib/systemlib.h>

#define DAEMON_PRIORITY SCHED_PRIORITY_MAX - 10 ///daemon priority

#if SIMULATION_FLAG == 1 //defined in parameter.h
    #define TIMEOUT_POLL 400 //ms between every simulation
#else
    #define TIMEOUT_POLL 1000 //normal usage set to 1 sec the timeout
#endif

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

/** @brief Subscribe/Advertise appropriate topics. */
bool as_topics(struct subscribtion_fd_s *subs_p,
               struct published_fd_s *pubs_p,
               struct structs_topics_s *strs_p);

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
    //local copy of parameters from QGroundControl
    struct parameters_qgc params;

    //optimal path parameters
    struct reference_actions_s ref_act = {.alpha_star = 0.5f, .should_tack = false};

    warnx(" starting\n");

    //initialize controller data structures
    init_controller_data();

    //initialize grid lines in Race frame
    init_grids();

    //init message module
    init_msg_module();

    //subscribe/advertise interested topics
    as_topics(&subs, &pubs, &strs);

    //initialize local copy of parameters from QGroundControl
    param_init(&params, &strs);

	// try to initiliaze actuators
    if(!actuators_init(&pubs, &strs)){
		// something went wrong
		thread_should_exit = true;
        warnx(" problem in initializing actuators\n");
	}

    // polling management
    struct pollfd fds[] = {
            { .fd = subs.gps_raw,                   .events = POLLIN },
            { .fd = subs.gps_filtered,              .events = POLLIN },
            { .fd = subs.wind_sailing,              .events = POLLIN },
            { .fd = subs.parameter_update,          .events = POLLIN },
            { .fd = subs.att,                       .events = POLLIN },
            { .fd = subs.boat_weather_station,      .events = POLLIN }
    };

    thread_running = true;

    while(!thread_should_exit){

        poll_ret = poll(fds, (sizeof(fds) / sizeof(fds[0])), TIMEOUT_POLL);

        // handle the poll result
        if(poll_ret == 0){
            // this means none of our providers is giving us data
            warnx(" got no data within a second\n");
        }
        else{
            /* this is undesirable but not much we can do - might want to flag unhappy status */
            if (poll_ret < 0) {
                warnx("POLL ERR %d, %d", poll_ret, errno);
                continue;
            }
            else{
                // everything is ok, at least so far (i.e. pool_ret > 0)
                if(fds[0].revents & POLLIN){
                    // new vehicle_gps_position data
                    orb_copy(ORB_ID(vehicle_gps_position), subs.gps_raw, &(strs.gps_raw));

                    #if SAVE_DEBUG_VALUES == 1
                    //save time when this action is performed
                    strs.debug_values.timestamp = hrt_absolute_time();
                    strs.debug_values.float_val_2 = strs.debug_values.timestamp / 1e3;
                    strs.debug_updated = true;
                    #endif

                    #if SIMULATION_FLAG == 0
                    //update course over ground in control data
                    update_cog(strs.gps_raw.cog_rad);
                    #endif

                }
                if(fds[1].revents & POLLIN){
                    // new vehicle_global_position data

                    //copy GPOS data
                    orb_copy(ORB_ID(vehicle_global_position), subs.gps_filtered, &(strs.gps_filtered));

                    //look into optimal path planning maps for reference actions
                    path_planning(&ref_act, &strs);

                    #if SAVE_DEBUG_VALUES == 1
                    //save time when this action is performed
                    strs.debug_values.timestamp = hrt_absolute_time();
                    strs.debug_values.float_val_1 = strs.debug_values.timestamp / 1e3;
                    strs.debug_updated = true;
                    #endif

                }
                if(fds[2].revents & POLLIN){
                    // new WSAI values, copy new data
                    orb_copy(ORB_ID(wind_sailing), subs.wind_sailing, &(strs.wind_sailing));

                    //update apparent wind direction in control data
                    update_app_wind(strs.wind_sailing.angle_apparent);

                    #if SAVE_DEBUG_VALUES == 1
                    //save time when this action is performed
                    strs.debug_values.timestamp = hrt_absolute_time();
                    strs.debug_values.float_val_3 = strs.debug_values.timestamp / 1e3;
                    strs.debug_updated = true;
                    #endif

                    #if SIMULATION_FLAG == 0
                    //update true wind direction in control data
                    update_twd(strs.wind_sailing.angle_true);
                    #endif
                }
                if(fds[3].revents & POLLIN){
                    // parameters updated
                    // read from param to clear updated flag
                    orb_copy(ORB_ID(parameter_update), subs.parameter_update, &(strs.update));

                    //update param
                    param_update(&params, &strs, true);
                }
                if(fds[4].revents & POLLIN){
                    // attitude updated
                    orb_copy(ORB_ID(vehicle_attitude), subs.att, &(strs.att));
                }
                if(fds[5].revents & POLLIN){
                    // boat_weather_station updated
                    orb_copy(ORB_ID(boat_weather_station), subs.boat_weather_station, &(strs.boat_weather_station));
                }
            }
        }

        #if SIMULATION_FLAG == 1
            //we're simulating the gps position, cog and twd with parameters from QGroundControl
            #if HIL_SIM == 1
                float cog, twd;
                get_data_hil(&cog, &twd);
                update_cog(cog);
                update_twd(twd);
            #else
                update_cog(params.cog_sim);
                update_twd(params.twd_sim);
            #endif
            //look into optimal path planning maps for reference actions
            path_planning(&ref_act, &strs);
            //fine cancella
        #endif

        //always perfrom guidance module to control the boat
        guidance_module(&ref_act, &params, &strs);

        //publish out commands
        orb_publish(ORB_ID_VEHICLE_ATTITUDE_CONTROLS, pubs.actuator_pub, &(strs.actuators));

        //publish debug value for post-processing
        orb_publish(ORB_ID(boat_guidance_debug), pubs.boat_guidance_debug_pub, &(strs.boat_guidance_debug));

        #if SAVE_DEBUG_VALUES == 1
        //publish debug data if updated
        if(strs.debug_updated){
            orb_publish(ORB_ID(debug_values), pubs.debug_values, &(strs.debug_values));
            strs.debug_updated = false;
        }
        #endif

        #if SIMULATION_FLAG == 1
        //strs.airspeed.true_airspeed_m_s = ref_act.alpha_star;
        orb_publish(ORB_ID(airspeed), pubs.airspeed, &(strs.airspeed));
        //fine cancella
        #endif
	}

    // kill all outputs
    for(unsigned i = 0; i < NUM_ACTUATOR_CONTROLS; i++)
        strs.actuators.control[i] = 0.0f;
    orb_publish(ORB_ID_VEHICLE_ATTITUDE_CONTROLS, pubs.actuator_pub, &(strs.actuators));

    warnx(" exiting.\n");

	thread_running = false;

	return 0;

}

/**
 * Subscribe/Advertise each fd to the correspondent topic.
 *
 * @return                 True on succes.
*/
bool as_topics(struct subscribtion_fd_s *subs_p,
               struct published_fd_s *pubs_p,
               struct structs_topics_s *strs_p){

    subs_p->gps_raw = orb_subscribe(ORB_ID(vehicle_gps_position));
    subs_p->gps_filtered = orb_subscribe(ORB_ID(vehicle_global_position));
    subs_p->wind_sailing = orb_subscribe(ORB_ID(wind_sailing));
    subs_p->parameter_update = orb_subscribe(ORB_ID(parameter_update));
    subs_p->att = orb_subscribe(ORB_ID(vehicle_attitude));
    subs_p->boat_weather_station = orb_subscribe(ORB_ID(boat_weather_station));

    if(subs_p->gps_raw == -1){
        warnx(" error on subscribing on vehicle_gps_position Topic \n");
        return false;
    }

    if(subs_p->gps_filtered == -1){
        warnx(" error on subscribing on vehicle_global_position Topic \n");
        return false;
    }

    if(subs_p->wind_sailing == -1){
        warnx(" error on subscribing on wind_sailing Topic \n");
        return false;
    }

    if(subs_p->parameter_update == -1){
        warnx(" error on subscribing on parameter_update Topic \n");
        return false;
    }

    if(subs_p->att == -1){
        warnx(" error on subscribing on vehicle_attitude Topic \n");
        return false;
    }

    if(subs_p->boat_weather_station == -1){
        warnx(" error on subscribing on boat_weather_station Topic \n");
        return false;
    }

    warnx(" subscribed to all topics \n");

    //advertise topics
    memset(&(strs_p->boat_guidance_debug), 0, sizeof(strs_p->boat_guidance_debug));
    pubs_p->boat_guidance_debug_pub = orb_advertise(ORB_ID(boat_guidance_debug), &(strs_p->boat_guidance_debug));

    #if SAVE_DEBUG_VALUES == 1
    // advertise debug_values topic
    memset(&(strs_p->debug_values), 0, sizeof(strs_p->debug_values));
    strs_p->debug_values.timestamp = hrt_absolute_time();
    pubs_p->debug_values = orb_advertise(ORB_ID(debug_values), &(strs_p->debug_values));
    strs_p->debug_updated = false;
    #endif

    #if SIMULATION_FLAG == 1
    memset(&(strs_p->airspeed), 0, sizeof(strs_p->airspeed));
    pubs_p->airspeed = orb_advertise(ORB_ID(airspeed), &(strs_p->airspeed));
    #endif
    return true;
}

/**
* Initialize actuators.
*
* @return   true if everything is OK
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



