/****************************************************************************
 *
 *   Copyright (c) 2012, 2013 PX4 Development Team. All rights reserved.
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
 * @file path_planning.c
 * Doing pathplanning for the boat.
 *
 * The main Software-Structure for reaching a target can be explained as follows:
 *
 * -------------------------------                                 --------------------------------
 * | MODULE "autonomous_sailing" |                                 | MODULE "path_planning"       |
 * |                             |       -------------------       |                              |
 * | this module is the HELSMAN. |       |  SHARED MEMEORY |       | this module is the NAVIGATOR.|
 * | it controls the sails and   | POLL  |                 | PUSH  | it calculates the optimum    |
 * | the rudder in order to      |------>|  heading        |<------| heading and gives orders     |
 * | track a given heading.      |       |  tack/gybe      |       | to the helsman.              |
 * |                             |       -------------------       |                              |
 * -------------------------------                                 --------------------------------
 *
 * @author Jonas Wirz <wirzjo@student.ethz.ch>
 * @author Marco Tranzatto <marco.tranzatto@gmail.com>
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <poll.h>
#include <errno.h>

#include <nuttx/config.h>
#include <nuttx/sched.h>

#include <systemlib/systemlib.h>
#include <systemlib/err.h>

#include "pp_config.h"
#include "pp_topics_handler.h"
#include "pp_gridlines_handler.h"
#include "pp_navigation_module.h"
#include "pp_parameters.h"

#include "pp_navigator.h"
#include "pp_failsafe.h"

#include "kalman_tracker/kt_tracker.h"

#include "parser_sensorboard/ps_sensorboard.h"

static bool thread_should_exit = false;		/**< daemon exit flag */
static bool thread_running = false;			/**< daemon status flag */
static int daemon_task;						/**< Handle of daemon task / thread */

//thread priority
#define DAEMON_PRIORITY SCHED_PRIORITY_MAX - 20 ///daemon priority (-20) (5 was working once for logging)
//#define DAEMON_PRIORITY SCHED_PRIORITY_MAX - 25 ///daemon priority


/**
 * daemon management function.
 */
__EXPORT int path_planning_main(int argc, char *argv[]);

/**
 * Mainloop of daemon.
 */
int pp_thread_main(int argc, char *argv[]);

/**
 * Print the correct usage.
 */
static void usage(const char *reason);

static void
usage(const char *reason)
{
	if (reason)
		warnx("%s\n", reason);
	errx(1, "usage: daemon {start|stop|status} [-p <additional params>]\n\n");
}

/**
 * The daemon app only briefly exists to start
 * the background job. The stack size assigned in the
 * Makefile does only apply to this management task.
 *
 * The actual stack size should be set in the call
 * to task_create().
 *
 * Note: This code is standard and should not be changed.
 *       It is directly copied from the Pixhawk Firmware Example.
 *
 */
int path_planning_main(int argc, char *argv[])
{
	if (argc < 1)
		usage("missing command");

	if (!strcmp(argv[1], "start")) {

		if (thread_running) {
			warnx("daemon already running\n");
			/* this is not an error */
			exit(0);
		}

		thread_should_exit = false;
        daemon_task = task_spawn_cmd("path_planning",
					 SCHED_DEFAULT,
                     DAEMON_PRIORITY,
                     4096, //was 4096 (5000 was working with SDLog)
					 pp_thread_main,
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
 * This is the main Thread that is created as a background Process by the Pixhawk Firmware
 *
 */
int pp_thread_main(int argc, char *argv[]) {

	//**THREAD IS STARTING
    warnx("path_planning starting\n");

	//**HANDLE TOPICS
	struct pp_subscribtion_fd_s subs;   // File-Descriptors of subscribed topics
	struct pp_structs_topics_s strs;    // Struct of Interested Topics

    pp_th_subscribe(&subs,&strs);       //Subscribe to interested Topics
    pp_th_advertise();                  //Advertise Topics

	//**POLL FOR CHANGES IN SUBSCRIBED TOPICS
    struct pollfd fds[] = {			 // Polling Management
            { .fd = subs.boat_guidance_debug,       .events = POLLIN }, //MUST BE THE FIRST ONE!
            { .fd = subs.vehicle_global_position,   .events = POLLIN },
            { .fd = subs.parameter_update,          .events = POLLIN },
            { .fd = subs.rc_channels,               .events = POLLIN },
            { .fd = subs.vehicle_attitude,          .events = POLLIN },
            { .fd = subs.mission_planning,			.events = POLLIN }
    };

    int poll_return;				//Return Value of the polling.

    //** INIT FUNCTIONS

    //init pp_paramters module
    pp_param_QGC_init();

    //init communication_buffer module
    pp_cb_init();

    //init pp_send_msg_qgc module
    smq_init_msg_module();

    #if USE_GRID_LINES == 1
    gh_init_grids();
    #endif //USE_GRID_LINES == 1

    //init the Navigator
	#if USE_GRID_LINES == 0
    nav_init();
	#endif


    //init communication with sensorboard
	#if LDEBUG_SENSORBOARD == 0
    sb_init();
	#endif


    //init the failsafe-mode
	#if USE_FAILSAFE == 1
    fs_init();
	#endif


    //init the Kalman tracker
	#if LDEBUG_KALMANTRACKER == 1
    tr_init();
	#endif

	//**SET THE THREAD-STATUS TO RUNNING
	thread_running = true;

	// MAIN THREAD-LOOP
	while (!thread_should_exit) {

		//**POLL FOR CHANGES IN THE SUBSCRIBED TOPICS
		poll_return = poll(fds, (sizeof(fds) / sizeof(fds[0])), TIMEOUT_POLL);

		if(poll_return == 0) {
			//No Topic contains changed data <=> no new Data available
			warnx("No new Data available\n");
		} else {
			//New Data is available

			if(poll_return < 0) {
				//An error occured during polling
				warnx("POLL ERR %d, %d", poll_return, errno);
				continue;
			} else {
				//Everything is OK and new Data is available
                if(fds[0].revents & POLLIN){
                    //update pp_communication_buffer with this information
                    cb_new_as_data(subs.boat_guidance_debug);
                }
                if(fds[1].revents & POLLIN){
                    //copy new GPOS data
                    orb_copy(ORB_ID(vehicle_global_position), subs.vehicle_global_position,
                             &(strs.vehicle_global_position));
                    //compute boat position in race frame using pp_navigation_module
                    n_navigation_module(&(strs.vehicle_global_position));
                    #if USE_GRID_LINES == 1
                    gh_gridlines_handler();
                    #endif //USE_GRID_LINES == 1


					#if LDEBUG_KALMANTRACKER == 1
                    //Provide the Kalman Tracker with the velocity of the boat
                    kt_set_velocity(strs.vehicle_global_position.vel_n, strs.vehicle_global_position.vel_e);
					#endif


                }
                if(fds[2].revents & POLLIN){
                    //copy new parameters from QGC
                    orb_copy(ORB_ID(parameter_update), subs.parameter_update,
                             &(strs.parameter_update));
                    //update param
                    pp_param_QGC_get(true);
                }
                if(fds[3].revents & POLLIN){
                    // copy commands from remote control
                    orb_copy(ORB_ID(rc_channels), subs.rc_channels, &(strs.rc_channels));
                    //update pp_communication_buffer with this information
                    cb_new_rc_data(&strs);

                    //Tell the state to the failsafe-mode
					#if USE_FAILSAFE == 1
                    fs_check_rc_signal(&strs);
					#endif
                }
                if(fds[4].revents & POLLIN){
                	//New data in vehicle_attitude topic
                	orb_copy(ORB_ID(vehicle_attitude), subs.vehicle_attitude, &(strs.vehicle_attitude));

                	//Set the new value for the yaw-angle
					#if USE_GRID_LINES == 0
                	yaw_update(&strs);
					#endif
                }
                if(fds[5].revents & POLLIN){
                	//New data in vehicle_attitude topic
                	orb_copy(ORB_ID(mission_planning), subs.mission_planning, &(strs.mission_planning));

                	//Set the new value for the mission
                	mission_update(strs.mission_planning);
                }
			}
		}

        // Call the navigator to calculate a new reference Heading
        // Note: The navigator is called in every loop, but it is only executed in a regular
        // time-based interval.
		#if USE_GRID_LINES == 0
        nav_navigator();
		#endif


        // Communicate with the Sensorboard
		#if LDEBUG_SENSORBOARD == 0
		sb_handler();
		#endif


        // Call the Failsafe state-machine
		#if USE_FAILSAFE == 1
        fs_state_machine();
		#endif


        // Call the Kalman Tracker Update-Function
		#if LDEBUG_KALMANTRACKER == 1
        tr_handler();
		#endif


        // Warning: path_planning and mi_ack topic should be published only ONCE for every loop iteration.
        // Use pp_communication_buffer to change topic's values.
        pp_cb_publish_if_updated();

	} //END OF MAIN THREAD-LOOP

	//**MANAGE THE KILLING OF THE THREAD
    warnx("path_planning exiting.\n");
	thread_running = false;

	return 0;
} //END OF pp_main_thread
