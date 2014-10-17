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


//Include topics necessary
#include <uORB/uORB.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/actuator_armed.h>


// To be able to use the "parameter function" from Q ground control:
#include <systemlib/param/param.h>

#include <systemlib/systemlib.h>


//Thread management variables
static bool thread_should_exit = false;		/**< daemon exit flag */
static bool thread_running = false;			/**< daemon status flag */
static int daemon_task;						/**< Handle of daemon task / thread */


/**
 * daemon management function.
 */
__EXPORT int autonomous_sailing_main(int argc, char *argv[]);

/**
 * mainloop of daemon.
 */
int as_daemon_thread_main(int argc, char *argv[]);

/**
 * Print the correct usage.
 */
static void usage(const char *reason);

/**
* initializes actuators.
*
* @param act_pointerp				pointer to actuators struct actuator_controls_s
* @param actuator_pub_pointer a 	pointer to orb_metadata for the actuators topic
* @return 							true if everything is OK
*/
bool actuators_init(struct actuator_controls_s *act_pointer, orb_advert_t *actuator_pub_pointer);

/**
 * autonomous sailing controller for the rudder.
 */
float as_rudder_controller();

/*
 * Define the QGroundControl parameters here:
 * Warning: name can not be too long!!!
 */


/**
 * Sails position
 *
 * ?????.
 * Default value for sails position (must be converted into degrees) 0 = max sheet out, 0.56 = max sheet in.
 *
 * @min 0 (max sheet out)
 * @max 0.56 (max sheet in)
 */
PARAM_DEFINE_FLOAT(ASAIL_SAIL, 0.5f);

/**
 * Default heading angle wrt relative wind.
 *
 *
 * @min -?
 * @max ?
 */
PARAM_DEFINE_FLOAT(ASAIL_RUDDER, 50.0f);

/**
 * Proportional gain.
 *
 *
 * @min 0
 * @max ?
 */
PARAM_DEFINE_FLOAT(ASAIL_PGAIN, 0.03f);



param_t servo_sail_pointer;		/**< pointer to param ASAIL_SAIL*/
param_t servo_rudder_pointer;   /**< pointer to param ASAIL_RUDDER*/
param_t pgain_pointer;          /**< pointer to param ASAIL_PGAIN*/




static void usage(const char *reason)
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
		daemon_task = task_spawn_cmd("daemon",
									SCHED_DEFAULT,
									SCHED_PRIORITY_MAX,	
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

	struct actuator_controls_s actuators; 
	orb_advert_t actuator_pub;

	warnx("[as_daemon_thread_main] starting\n");

	thread_running = true;

	// try to initiliaze actuators
	if(!actuators_init(&actuators, &actuator_pub)){
		// something went wrong
		thread_should_exit = true;
		warnx("[as_daemon_thread_main] Problem in initializing actuators\n");
	}

	while (!thread_should_exit) {

		//prova
		/*actuators.timestamp = hrt_absolute_time();
		actuators.control[0] = 0.3;
		actuators.control[3] = 0.5;

		orb_publish(ORB_ID_VEHICLE_ATTITUDE_CONTROLS, actuator_pub, &actuators);

		sleep(1);*/
	}

	warnx("[as_daemon_thread_main] exiting.\n");

	thread_running = false;

	return 0;

}

bool actuators_init(struct actuator_controls_s *act_pointer, orb_advert_t *actuator_pub_pointer){

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
	memset(act_pointer, 0, sizeof(*act_pointer));
	// set actuator controls values to zero
	for (unsigned i = 0; i < NUM_ACTUATOR_CONTROLS; i++) {
		act_pointer->control[i] = 0.0f;
	}

	// advertise that this controller will publish actuator values
	*actuator_pub_pointer = orb_advertise(ORB_ID_VEHICLE_ATTITUDE_CONTROLS, act_pointer);

	//check if everything was ok
	if(armed_pub == ERROR || armed_pub == -1 || *actuator_pub_pointer == ERROR || *actuator_pub_pointer == -1)
		right_init = false; //something went wrong


	return right_init;
}

float as_rudder_controller() {


}
