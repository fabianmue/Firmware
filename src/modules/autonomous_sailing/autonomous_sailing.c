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

//prova
#include <dataman/dataman.h>

// To be able to use the "parameter function" from Q ground control:
#include <systemlib/param/param.h>
#include <systemlib/systemlib.h>

#define DAEMON_PRIORITY SCHED_PRIORITY_MAX - 10 ///daemon priority

#if SIMULATION_FLAG == 1 //defined in parameter.h
    #define TIMEOUT_POLL 200 //ms between every simulation
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


//cancella
void prova_waypoints(struct structs_topics_s *strs_p);

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
            { .fd = subs.boat_weather_station,      .events = POLLIN },
            { .fd = subs.mission, .events = POLLIN }//prova
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

                    #if SIMULATION_FLAG == 0
                    //update course over ground in control data
                    update_cog(strs.gps_raw.cog_rad);
                    #endif

                }
                if(fds[1].revents & POLLIN){
                    // new vehicle_global_position data

                    //copy GPS data
                    orb_copy(ORB_ID(vehicle_global_position), subs.gps_filtered, &(strs.gps_filtered));

                    //look into optimal path planning maps for reference actions
                    path_planning(&ref_act, &strs, &params);

                }
                if(fds[2].revents & POLLIN){
                    // new WSAI values, copy new data
                    orb_copy(ORB_ID(wind_sailing), subs.wind_sailing, &(strs.wind_sailing));

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
                if(fds[6].revents & POLLIN){//prova
                    // mission updated
                    orb_copy(ORB_ID(offboard_mission),
                             subs.mission,
                             &(strs.mission));

                    prova_waypoints(&strs);
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
            path_planning(&ref_act, &strs, &params);
            //fine cancella
        #endif

        //always perfrom guidance module to control the boat
        guidance_module(&ref_act, &params, &strs, &pubs);

        // Send out commands
        orb_publish(ORB_ID_VEHICLE_ATTITUDE_CONTROLS, pubs.actuator_pub, &(strs.actuators));

        #if SIMULATION_FLAG == 1
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
    subs_p->mission = orb_subscribe(ORB_ID(offboard_mission));//prova
    strs_p->current_offboard = -1;//prova

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

    if(subs_p->mission == -1){//prova
        warnx(" error on subscribing on mission Topic \n");
        return false;
    }

    warnx(" subscribed to all topics \n");

    //advertise topics
    memset(&(strs_p->boat_guidance_debug), 0, sizeof(strs_p->boat_guidance_debug));
    pubs_p->boat_guidance_debug_pub = orb_advertise(ORB_ID(boat_guidance_debug), &(strs_p->boat_guidance_debug));

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


void prova_waypoints(struct structs_topics_s *strs_p){

    /* determine current index */
    if (strs_p->mission.current_seq >= 0 && strs_p->mission.current_seq < (int)strs_p->mission.count) {
        strs_p->current_offboard = strs_p->mission.current_seq;
    } else {
        /* if less items available, reset to first item */
        if (strs_p->current_offboard >= (int)strs_p->mission.count) {
            strs_p->current_offboard = 0;

        /* if not initialized, set it to 0 */
        } else if (strs_p->current_offboard < 0) {
            strs_p->current_offboard = 0;
        }
        /* otherwise, just leave it */
    }

    /* Check mission feasibility, for now do not handle the return value,
     * however warnings are issued to the gcs via mavlink from inside the MissionFeasiblityChecker */
    //dm_item_t dm_current = DM_KEY_WAYPOINTS_OFFBOARD(strs_p->mission.dataman_id);

//    _missionFeasiblityChecker.checkMissionFeasible(_navigator->get_vstatus()->is_rotary_wing,
//            dm_current, (size_t) strs_p->mission.count, _navigator->get_geofence(),
//            _navigator->get_home_position()->alt);

    struct mission_s mission_state;

    /* lock MISSION_STATE item */
    dm_lock(DM_KEY_MISSION_STATE);

    /* read current state */
    int read_res = dm_read(DM_KEY_MISSION_STATE, 0, &mission_state, sizeof(mission_state));

    //strs_p->airspeed.true_airspeed_m_s = (float)read_res;//cancella

    if (read_res == sizeof(mission_state)) {
        /* data read successfully, check dataman ID and items count */

        if (mission_state.dataman_id == strs_p->mission.dataman_id && mission_state.count == strs_p->mission.count) {
            /* navigator may modify only sequence, write modified state only if it changed */
            if (mission_state.current_seq != strs_p->current_offboard) {
                if (dm_write(DM_KEY_MISSION_STATE, 0, DM_PERSIST_POWER_ON_RESET, &mission_state, sizeof(mission_state)) != sizeof(mission_state)) {
                    //warnx("ERROR: can't save mission state");
                    //mavlink_log_critical(_navigator->get_mavlink_fd(), "ERROR: can't save mission state");
                }
            }
        }

    } else {
        /* invalid data, this must not happen and indicates error in offboard_mission publisher */
        mission_state.dataman_id = strs_p->mission.dataman_id;
        mission_state.count = strs_p->mission.count;
        mission_state.current_seq = strs_p->current_offboard;

        //warnx("ERROR: invalid mission state");
        //mavlink_log_critical(_navigator->get_mavlink_fd(), "ERROR: invalid mission state");

        /* write modified state only if changed */
        if (dm_write(DM_KEY_MISSION_STATE, 0, DM_PERSIST_POWER_ON_RESET, &mission_state, sizeof(mission_state)) != sizeof(mission_state)) {
            //warnx("ERROR: can't save mission state");
            //mavlink_log_critical(_navigator->get_mavlink_fd(), "ERROR: can't save mission state");
        }
    }


}
