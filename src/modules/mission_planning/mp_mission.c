/*
 * mp_mission.c
 *
 * This file provides functions for setting a competition Race-Field in WRSC 20xx. The file defines "Missions"
 *
 * O is a buoy
 *
 *   |
 *   | Wind direction from true North
 *   V
 *
 *   O1----------------O2
 *   |                 |
 *   |                 |
 *   Finish            |
 *   |                 |
 *   |                 |
 *   |                 |
 *   O3-----Start -----O4
 *
 *  x
 *  ^
 *  |
 *  |--> y
 *
 *  Created on: 22.05.2015
 *      Author: Jonas Wirz 		<wirzjo@student.ethz.ch>
 *      		Fabian Müller 	<fabianmu@student.ethz.ch>
 */


#include <stdbool.h>
#include <math.h>
#include "mp_mission.h"

#include <drivers/drv_hrt.h>

#include "../path_planning/pp_send_msg_qgc.h"


/***********************************************************************************/
/*****  V A R I A B L E S  *********************************************************/
/***********************************************************************************/

extern frame comp_frame;

static uint64_t last_call;

mission last_mission;

/***********************************************************************************/
/*****  P U B L I C    F U N C T I O N S  ******************************************/
/***********************************************************************************/

/*
 * set the configuration of the race-Field
 *
 * @param O1long: longitude of buoy 1 [deg]
 * @param O1lat: latitude of buoy 1 [deg]
 * @param dist: distances between two buoys [m]
 * @param rotation: rotation of the frame around O1 [deg] (0 deg = y axis)
 */
void mp_set_racefield(float O1long, float O1lat, float dist, float rotation) {

	comp_frame.dist = dist;
	comp_frame.rotation = DEG2RAD * rotation;
	comp_frame.O1.northx = 0;
	comp_frame.O1.easty = 0;

	float sqrt2 = (float) sqrt(2);

	comp_frame.O2.northx = comp_frame.O1.northx + dist*sinf(comp_frame.rotation);
	comp_frame.O2.easty = comp_frame.O1.easty + dist*cosf(comp_frame.rotation);
	comp_frame.O3.northx = comp_frame.O1.northx - dist*sinf(comp_frame.rotation-PI/2);
	comp_frame.O3.easty = comp_frame.O1.easty + dist*cosf(comp_frame.rotation-PI/2);
	comp_frame.O4.northx = comp_frame.O1.northx - sqrt2*dist*sinf(comp_frame.rotation-PI/4);
	comp_frame.O4.easty = comp_frame.O1.easty + sqrt2*dist*cosf(comp_frame.rotation-PI/4);

}

/*
 * start a new mission
 *
 * @param: mission : mission that is started
 *
 */
bool mp_start_mission(mission new_mission) {

	if(last_mission.isactive == true) {

		smq_send_log_info("current mission still active, new mission could not be started!");
		return false;
	}
	else {

		/* TODO: start mission (start thread) */
		new_mission.isactive = true;
		last_mission = new_mission;
		smq_send_log_info("new mission started!");

		switch(new_mission.type) {
			case 0:
				// triangular course
				break;

			case 1:
				// station-keeping
				break;

			case 2:
				// area-scanning
				break;

			case 3:
				// fleet-race
				break;

			case 4:
				// unconstrained
				break;

			default:
				// nothing to do here
				break;

		return true;
		}
	}
}

void mp_stop_mission(void) {

	/* TODO: stop mission (stop thread) */
	last_mission.isactive = false;
	smq_send_log_info("mission stopped!");

};

bool mp_handler(void) {

	uint64_t curr_time = hrt_absolute_time();
	uint64_t difference = curr_time - last_call;

	if(difference > 2e6) {

		//Do not call the mission planner every time in the main-loop, but only every two seconds
		last_call = curr_time;

	}

	return true;

}
