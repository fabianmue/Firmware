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

#include <uORB/topics/wind_sailing.h>
#include <uORB/topics/vehicle_gps_position.h>
#include <uORB/topics/vehicle_bodyframe_meas.h>
#include <uORB/topics/vehicle_attitude.h>


// To be able to use the "parameter function" from Q ground control:
#include <systemlib/param/param.h>
#include <systemlib/systemlib.h>

#define DAEMON_PRIORITY SCHED_PRIORITY_MAX - 10 ///daemon priority

#define deg2rad 0.017453292519f // pi / 180


//Thread management variables
static bool thread_should_exit = false;		/**< daemon exit flag */
static bool thread_running = false;			/**< daemon status flag */
static int daemon_task;						/**< Handle of daemon task / thread */


/** @brief Daemon management function. */
__EXPORT int autonomous_sailing_main(int argc, char *argv[]);

/** @brief Mainloop of daemon. */
int as_daemon_thread_main(int argc, char *argv[]);

/** @brief Print the correct usage. */
static void usage(const char *reason);

/** @brief Iinitialize actuators. */
bool actuators_init(struct actuator_controls_s *act_pointer, orb_advert_t *actuator_pub_pointer);

/** @brief Subscribe to appropriate topics. */
bool as_subscriber(int *att_fd_pointer, int *gps_fd_pointer,
                   int *bfme_fd_pointer, int* wsai_fd_pointer);

/** @brief Autonomous sailing controller for the rudder. */
float as_rudder_controller(const struct wind_sailing_s  *wsai_raw_pointer,
                           struct actuator_controls_s *act_pointer);

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
 * Default heading angle w.r.t. relative wind, in degrees.
 *
 *
 * @min -90
 * @max 90
 */
PARAM_DEFINE_FLOAT(ASAIL_RUDDER, 30.0f);

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

    // file descriptor and struct for actuator_controls topic
	struct actuator_controls_s actuators; 
	orb_advert_t actuator_pub;

    //pool return value
    int poll_ret;

    // file descriptor and struct for attitude topic
    int att_pub_fd;
    struct vehicle_attitude_s att_raw;

    // file descriptor and struct for vehicle_gps_position topic
    int gps_pub_fd;
    struct vehicle_gps_position_s gps_raw;

    // file descriptor and struct for wind_sailing topic
    int wsai_pub_fd;
    struct wind_sailing_s wsai_raw;

    // file descriptor and struct for vehicle_bodyframe_meas topic
    int bfme_pub_fd;
    struct vehicle_bodyframe_meas_s bfme_raw;

    warnx(" starting\n");

    as_subscriber(&att_pub_fd, &gps_pub_fd, &bfme_pub_fd, &wsai_pub_fd);

	// try to initiliaze actuators
	if(!actuators_init(&actuators, &actuator_pub)){
		// something went wrong
		thread_should_exit = true;
        warnx(" problem in initializing actuators\n");
	}

    // polling management
    struct pollfd fds[] = {
            { .fd = att_pub_fd ,   .events = POLLIN },
            { .fd = gps_pub_fd ,   .events = POLLIN },
            { .fd = wsai_pub_fd,   .events = POLLIN },
            { .fd = bfme_pub_fd,   .events = POLLIN }
    };

    //prova
    //att_pub_fd = orb_subscribe(ORB_ID(vehicle_attitude));

    //prova
//    struct pollfd fds[] = {
//            { .fd = wsai_pub_fd,   .events = POLLIN }
//    };

    thread_running = true;

    while(!thread_should_exit){

        poll_ret = poll(fds, 4, 1000);

        // handle the poll result
        if(poll_ret == 0) {
            // this means none of our providers is giving us data
            warnx(" got no data within a second\n");
        }
        else{
            if(poll_ret < 0) {
                // this is seriously bad - should be an emergency
                warnx(" terrible error!\n");
            }
            else{
                // evrything is ok, at least so far (i.e. pool_ret > 0)
                if(fds[0].revents & POLLIN){
                    // new Attitude values

                    //prova
                    orb_copy(ORB_ID(vehicle_attitude), att_pub_fd, &att_raw);

                }
                if(fds[1].revents & POLLIN){
                    // new GPS values

                    //prova
                    orb_copy(ORB_ID(vehicle_gps_position), gps_pub_fd, &gps_raw);

                }
                if(fds[2].revents & POLLIN){//ripristina 2 e NON 0
                    // new WSAI values

                    // copy new data
                    orb_copy(ORB_ID(wind_sailing), wsai_pub_fd, &wsai_raw);
                    // rudder control
                    as_rudder_controller(&wsai_raw, &actuators);

                }
                if(fds[3].revents & POLLIN){
                    // new BFME values

                    // copy new data
                    orb_copy(ORB_ID(vehicle_bodyframe_meas), bfme_pub_fd, &bfme_raw);

                }
            }
        }

        // Send out commands:
        //ripristina
        orb_publish(ORB_ID_VEHICLE_ATTITUDE_CONTROLS, actuator_pub, &actuators);
        // actuators.control[0] -> output channel 1
        // actuators.control[2] -> output channel 3
        // actuators.control[3] -> output channel 4
	}

    // kill all outputs
    for (unsigned i = 0; i < NUM_ACTUATOR_CONTROLS; i++)
        actuators.control[i] = 0.0f;
    orb_publish(ORB_ID_VEHICLE_ATTITUDE_CONTROLS, actuator_pub, &actuators);

    warnx(" exiting.\n");

	thread_running = false;

	return 0;

}

/**
 * Subscribe each fd to the correspondent topic.
 *
 * @param att_fd_pointer   Pointer to int that will contain file descriptor for ATT topic.
 * @param gps_fd_pointer   Pointer to int that will contain file descriptor for GPS topic.
 * @param bfme_fd_pointer  Pointer to int that will contain file descriptor for BFME topic.
 * @param wsai_fd_pointer  Pointer to int that will contain file descriptor for WSAI topic.
 *
 * @return                 True on succes.
*/
bool as_subscriber(int *att_fd_pointer, int *gps_fd_pointer,
                   int *bfme_fd_pointer, int* wsai_fd_pointer){

    *att_fd_pointer = orb_subscribe(ORB_ID(vehicle_attitude));
    *gps_fd_pointer = orb_subscribe(ORB_ID(vehicle_gps_position));
    *bfme_fd_pointer = orb_subscribe(ORB_ID(vehicle_bodyframe_meas));
    *wsai_fd_pointer = orb_subscribe(ORB_ID(wind_sailing));

    if(*att_fd_pointer == -1){
        warnx(" error on subscribing on Attitude Topic \n");
        return false;
    }

    if(*gps_fd_pointer == -1){
        warnx(" error on subscribing on GPS Topic \n");
        return false;
    }

    if(*bfme_fd_pointer == -1){
        warnx(" error on subscribing on Body Frame Measurements Topic \n");
        return false;
    }

    if(*wsai_fd_pointer == -1){
        warnx(" error on subscribing on Wind Sailing Topic \n");
        return false;
    }

    warnx(" subscribed to all topics \n");

    return true;
}

/**
* Initialize actuators.
*
* @param act_pointerp				pointer to actuators struct actuator_controls_s
* @param actuator_pub_pointer a 	pointer to orb_metadata for the actuators topic
* @return 							true if everything is OK
*/
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

float as_rudder_controller(const struct wind_sailing_s  *wsai_raw_pointer,
                           struct actuator_controls_s *act_pointer){


}
