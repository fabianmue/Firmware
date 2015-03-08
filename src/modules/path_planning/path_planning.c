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


/* TODO:
 * - add logging of variables
 * - add parameters from QGroundControl
 * - add Potentialfield Method
 * - solve missing Reference Problem to PRIORITY VARIABLES
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

static bool thread_should_exit = false;		/**< daemon exit flag */
static bool thread_running = false;			/**< daemon status flag */
static int daemon_task;						/**< Handle of daemon task / thread */

//thread priority
#define DAEMON_PRIORITY SCHED_PRIORITY_MAX - 20 ///daemon priority


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
                     4096,
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
	struct subscribtion_fd_s subs;   //File-Descriptors of subscribed topics
	struct structs_topics_s strs;    //Struct of Interested Topics


    th_subscribe(&subs,&strs);       //Subscribe to interested Topics
    th_advertise(&strs);	 		 //Advertise Topics


	//**POLL FOR CHANGES IN SUBSCRIBED TOPICS
    struct pollfd fds[] = {			 // Polling Management
            { .fd = subs.boat_guidance_debug,       .events = POLLIN },//MUST BE THE FIRST ONE!
            { .fd = subs.vehicle_global_position,   .events = POLLIN },
            { .fd = subs.parameter_update,          .events = POLLIN },
            { .fd = subs.rc_channels,               .events = POLLIN }
    };

    int poll_return;				//Return Value of the polling.


    //** INIT FUNCTIONS

	//nav_init();	//Init a Navigator

    //init pp_paramters module
    p_param_init();

    //init communication_buffer module
    cb_init();

    //init pp_send_msg_qgc module
    smq_init_msg_module();

    #if USE_GRID_LINES == 1
    gh_init_grids();
    #endif

	//**SET THE THREAD-STATUS TO RUNNING
	thread_running = true;


	/**MAIN THREAD-LOOP
	 * This is the main Thread Loop. It loops until the Process is killed.*/
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
                    //copy new BGUD data
                    orb_copy(ORB_ID(boat_guidance_debug), subs.boat_guidance_debug,
                             &(strs.boat_guidance_debug));
                    //update pp_communication_buffer with this information
                    cb_new_as_data(&strs);
                }
                if(fds[1].revents & POLLIN){
                    //copy new GPOS data
                    orb_copy(ORB_ID(vehicle_global_position), subs.vehicle_global_position,
                             &(strs.vehicle_global_position));
                    //compute boat position in race frame using pp_navigation_module
                    n_navigation_module(&strs);
                    #if USE_GRID_LINES == 1
                    gh_gridlines_handler();
                    #endif
                }
                if(fds[2].revents & POLLIN){
                    //copy new parameters from QGC
                    orb_copy(ORB_ID(parameter_update), subs.parameter_update,
                             &(strs.parameter_update));
                    //update param
                    p_param_update(true);
                }
                if(fds[3].revents & POLLIN){
                    // copy commands from remote control
                    orb_copy(ORB_ID(rc_channels), subs.rc_channels, &(strs.rc_channels));
                    //update pp_communication_buffer with this informaion
                    cb_new_rc_data(&strs);
                }
			}
		}

        /* Warning: path_planning topic should be published only ONCE for every loop iteration.
         * Use pp_communication_buffer to change topic's values.
        */
        cb_publish_pp_if_updated();


	} //END OF MAIN THREAD-LOOP


	//**MANAGE THE KILLING OF THE THREAD
    warnx("path_planning exiting.\n");
	thread_running = false;

	return 0;
} //END OF pp_main_thread
