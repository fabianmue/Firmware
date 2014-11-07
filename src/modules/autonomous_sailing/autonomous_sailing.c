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
#include <uORB/uORB.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/actuator_armed.h>

#include <uORB/topics/wind_sailing.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_bodyframe_meas.h>
#include <uORB/topics/vehicle_attitude.h>


// To be able to use the "parameter function" from Q ground control:
#include <systemlib/param/param.h>
#include <systemlib/systemlib.h>

#define DAEMON_PRIORITY SCHED_PRIORITY_MAX - 10 ///daemon priority

//#define deg2rad 0.017453292519f // pi / 180


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

/** @brief Convert GPS data in position in Race frame coordinate*/
void navigation_module(const struct vehicle_global_position_s *gps_p,
                       int32_t *x_cm_race_p, int32_t *y_cm_race_p,
                       int32_t *target_x_cm_race_p, int32_t *target_y_cm_race_p);

/** @brief Decide the next control action to be implemented*/
void guidance_module();

/** @brief Initialize parameters*/
void param_init(float *sail_servo_p, float *rudder_servo_p,
                float *p_gain_p, float *i_gain_p,
                int32_t *lat0_p, int32_t *lon0_p, int32_t *alt0_p,
                float *epsilon_p);

/** @brief Check if one or more parameters have been updated and perform appropriate actions*/
void param_check_update(float *sail_servo_p, float *rudder_servo_p,
                        float *p_gain_p, float *i_gain_p,
                        int32_t *lat0_p, int32_t *lon0_p, int32_t *alt0_p,
                        float *epsilon_p);



param_t sail_pointer;         /**< pointer to param AS_SAIL*/
param_t rudder_pointer;       /**< pointer to param AS_RUDDER*/

param_t p_gain_pointer;       /**< pointer to param AS_P_GAIN*/
param_t i_gain_pointer;       /**< pointer to param AS_I_GAIN*/

param_t lat0_pointer;         /**< pointer to param AS_LAT0*/
param_t lon0_pointer;         /**< pointer to param AS_LON0*/
param_t alt0_pointer;         /**< pointer to param AS_ALT0*/

param_t epsilon_pointer;      /**< pointer to param AS_EPSI*/



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

    // file descriptor and struct for vehicle_global_position topic
    int gps_pub_fd;
    struct vehicle_global_position_s gps_raw;

    // file descriptor and struct for wind_sailing topic
    int wsai_pub_fd;
    struct wind_sailing_s wsai_raw;

    // file descriptor and struct for vehicle_bodyframe_meas topic
    int bfme_pub_fd;
    struct vehicle_bodyframe_meas_s bfme_raw;

    // boat's coordinates in Race frame
    int32_t x_cm_race = 0; ///x position of the boat in Race frame, in centimeters.
    int32_t y_cm_race = 0; ///y position of the boat in Race frame, in centimeters.

    // next target's coordinates in Race frame, for now set them far away from 0
    int32_t target_x_cm_race = 100000; ///x position of next target in Race frame, in centimeters.
    int32_t target_y_cm_race = 100000; ///y position of next target in Race frame, in centimeters.

    //paramters from QGroundControl
    float rudder_servo;
    float sail_servo;

    float p_gain;
    float i_gain;

    int32_t lat0;
    int32_t lon0;
    int32_t alt0;

    float epsilon;

    warnx(" starting\n");

    //subscribe to interested topics
    as_subscriber(&att_pub_fd, &gps_pub_fd, &bfme_pub_fd, &wsai_pub_fd);

    //initialize parameters
    param_init(&rudder_servo, &sail_servo,
               &p_gain, &i_gain,
               &lat0, &lon0, &alt0,
               &epsilon);

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

    //set reference of NED frame before starting
    set_ref0(&lat0, &lon0, &alt0);

    thread_running = true;

    while(!thread_should_exit){

        poll_ret = poll(fds, (sizeof(fds) / sizeof(fds[0])), 1000);

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

                    //copy GPS data
                    orb_copy(ORB_ID(vehicle_global_position), gps_pub_fd, &gps_raw);

                    //do navigation module
                    navigation_module(&gps_raw,
                                      &x_cm_race, &y_cm_race,
                                      &target_x_cm_race, &target_y_cm_race);

                }
                if(fds[2].revents & POLLIN){
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
    *gps_fd_pointer = orb_subscribe(ORB_ID(vehicle_global_position));
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
* Initialize parameters.
*
*/
void param_init(float *sail_servo_p, float *rudder_servo_p,
                float *p_gain_p, float *i_gain_p,
                int32_t *lat0_p, int32_t *lon0_p, int32_t *alt0_p,
                float *epsilon_p){

    //initialize pointer to parameters
    sail_pointer      = param_find("AS_SAIL");
    rudder_pointer    = param_find("AS_RUDDER");

    p_gain_pointer    = param_find("AS_P_GAIN");
    i_gain_pointer    = param_find("AS_I_GAIN");

    lat0_pointer      = param_find("AS_LAT0");
    lon0_pointer      = param_find("AS_LON0");
    alt0_pointer      = param_find("AS_ALT0");

    epsilon_pointer   = param_find("AS_EPSI");

    param_get(sail_pointer, sail_servo_p);
    param_get(rudder_pointer, rudder_servo_p);

    param_get(p_gain_pointer, p_gain_p);
    param_get(i_gain_pointer, i_gain_p);

    param_get(lat0_pointer, lat0_p);
    param_get(lon0_pointer, lon0_p);
    param_get(alt0_pointer, alt0_p);

    param_get(epsilon_pointer, epsilon_p);
}

/** Check if any paramter has been updated, if so take appropriate actions
 *
*/
void param_check_update(float *sail_servo_p, float *rudder_servo_p,
                        float *p_gain_p, float *i_gain_p,
                        int32_t *lat0_p, int32_t *lon0_p, int32_t *alt0_p,
                        float *epsilon_p){

    float app_f;
    int32_t app_i;

    //check sail_servo
    param_get(sail_pointer, &app_f);
    if(*sail_servo_p != app_f){
        *sail_servo_p = app_f;
    }

    //check rudder_servo
    param_get(rudder_pointer, &app_f);
    if(*rudder_servo_p != app_f){
        *rudder_servo_p = app_f;
    }

    //check p_gain
    param_get(p_gain_pointer, &app_f);
    if(*p_gain_p != app_f){
        *p_gain_p = app_f;
    }

    //check i_gain
    param_get(i_gain_pointer, &app_f);
    if(*i_gain_p != app_f){
        *i_gain_p = app_f;
    }

    //check lat0
    param_get(lat0_pointer, &app_i);
    if(*lat0_p != app_i){
        *lat0_p = app_i;
        //update NED origin
        set_ref0(lat0_p, lon0_p, alt0_p);
    }

    //check lon0
    param_get(lon0_pointer, &app_i);
    if(*lon0_p != app_i){
        *lon0_p = app_i;
        //update NED origin
        set_ref0(lat0_p, lon0_p, alt0_p);
    }

    //check alt0
    param_get(alt0_pointer, &app_i);
    if(*alt0_p != app_i){
        *alt0_p = app_i;
        //update NED origin
        set_ref0(lat0_p, lon0_p, alt0_p);
    }

    //check epsilon
    param_get(epsilon_pointer, &app_f);
    if(*epsilon_p != app_f){
        *epsilon_p = app_f;
    }
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

/**
 * Compute from GPS position the boat's position in Race frame. Set up the next target position.
 *
 * @param gps_p                 pointer to gps struct.
 * @param x_cm_race_p           pointer to the value to be set with boat x coordinate in Race frame.
 * @param y_cm_race_p           pointer to the value to be set with boat y coordinate in Race frame.
 * @param target_x_cm_race_p    pointer to the value to be set with target x coordinate in Race frame.
 * @param target_y_cm_race_p    pointer to the value to be set with target y coordinate in Race frame.
*/
void navigation_module(const struct vehicle_global_position_s *gps_p,
                       int32_t *x_cm_race_p, int32_t *y_cm_race_p,
                       int32_t *target_x_cm_race_p, int32_t *target_y_cm_race_p){

    int32_t north_cm;
    int32_t east_cm;
    int32_t down_cm;


    //compute boat position in NED frame
    geo_to_ned(gps_p, &north_cm, &east_cm, &down_cm);
}
