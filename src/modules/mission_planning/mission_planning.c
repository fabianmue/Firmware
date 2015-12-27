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

/*
 * @file mission_planning.c: doing mission planning for the boat.
 *
 * The main Software-Structure for reaching a target can be explained as follows:
 *
 * -------------------------------                                 --------------------------------		  --------------------------------
 * | MODULE "autonomous_sailing" |                                 | MODULE "path_planning"       |       | MODULE "mission_planning"    |
 * |                             |       -------------------       |                              |       |                              |
 * | this module is the HELSMAN. |       |  SHARED MEMEORY |       | this module is the NAVIGATOR.|       | this module sets the way-    |
 * | it controls the sails and   | POLL  |                 | PUSH  | it calculates the optimum    | PUSH  | points to fulfill a certain  |
 * | the rudder in order to      |------>|  heading        |<------| heading and gives orders     |<------| mission. It receives data    |
 * | track a given heading.      |       |  tack/gybe      |       | to the helsman.              |       | from a ground station via    |
 * |                             |       -------------------       |                              |       | SD card or telemetry.        |
 * -------------------------------                                 --------------------------------		  --------------------------------
 *
 * @author Jonas Wirz <wirzjo@student.ethz.ch>
 * @author Marco Tranzatto <marco.tranzatto@gmail.com>
 * @author Fabian Müller <fabianmu@student.ethz.ch>
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

#include "mp_topics_handler.h"
#include "mp_communication_buffer.h"
#include "mp_params.h"

#include "mp_send_msg_qgc.h"

/***********************************************************************************/
/*****  V A R I A B L E S  *********************************************************/
/***********************************************************************************/

static bool thread_should_exit = false;		/**< daemon exit flag */
static bool thread_running = false;			/**< daemon status flag */
static int daemon_task;						/**< Handle of daemon task / thread */

//thread priority
#define DAEMON_PRIORITY SCHED_PRIORITY_MAX - 25 ///daemon priority (-25)
#define TIMEOUT_POLL 1000

/***********************************************************************************/
/*****  F U N C T I O N   D E C L A R A T I O N S  *********************************/
/***********************************************************************************/

// print the correct usage.
static void usage(const char *reason);

 // daemon management function.
__EXPORT int mission_planning_main(int argc, char *argv[]);

// mainloop of daemon
int mp_thread_main(int argc, char *argv[]);

/***********************************************************************************/
/*****  F U N C T I O N   D E F I N I T I O N S  ***********************************/
/***********************************************************************************/

static void usage(const char *reason) {

	if (reason) {
		warnx("%s\n", reason);
	}
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
 * argv[1] = "start", "stop" or "status"
 * rest of the arguments are passed to daemon task creation
 *
 */
int mission_planning_main(int argc, char *argv[])
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
		// create thread
        daemon_task = task_spawn_cmd("mission_planning",
        		SCHED_DEFAULT,
        		DAEMON_PRIORITY,
        		4096,
        		mp_thread_main,
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
int mp_thread_main(int argc, char *argv[]) {

	// thread is starting
    warnx("mission_planning starting\n");

	// handle topics
	struct mp_subscribtion_fd_s subs;   // file-descriptors of subscribed topics
	struct mp_structs_topics_s strs;    // struct of Interested Topics

    mp_th_subscribe(&subs, &strs);      // subscribe to interested topics
    mp_th_advertise();                  // advertise topics

	// poll for changes in subscribed topics
    struct pollfd fds[] = {			 // polling management
            { .fd = subs.parameter_update,          .events = POLLIN },
            { .fd = subs.mi_ack,          			.events = POLLIN }
    };

    int poll_return;				// return value of the polling.

    // init mission planning parameters
    mp_param_init();

    // init communication buffer
    mp_cb_init();

    // init msg to QGC module
    mp_init_msg_module();

	thread_running = true;

    // main_thread loop - loops until thread is killed
	while (!thread_should_exit) {

		// poll for changes in subscribed topics
		poll_return = poll(fds, (sizeof(fds) / sizeof(fds[0])), TIMEOUT_POLL);

		if (poll_return == 0) {

			// no topic contains changed data
			warnx("no new data\n");
		} else if (poll_return < 0) {

			// error during polling
			warnx("polling error %d, %d", poll_return, errno);
			continue;
		} else {

			// new data
            if(fds[0].revents & POLLIN) {

                // copy new parameters from QGC
                orb_copy(ORB_ID(parameter_update), subs.parameter_update, &(strs.parameter_update));
                // update parameters
                mp_param_update();
            }
            if(fds[1].revents & POLLIN) {

                // copy new parameters from QGC
                orb_copy(ORB_ID(mi_ack), subs.mi_ack, &(strs.mi_ack));
                // send next waypoint/obstacle parameters
                mp_tf_mi_data(&strs);
            }
		}

        // warning: mission_planning topic should be published only ONCE for every loop iteration
        mp_cb_publish_if_updated();

	}

	// kill the thread
    warnx("mission_planning exiting\n");
	thread_running = false;

	return 0;
}
