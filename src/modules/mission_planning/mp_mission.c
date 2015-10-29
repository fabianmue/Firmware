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

mission last_mission;

/***********************************************************************************/
/*****  F U N C T I O N   D E F I N I T I O N S  ***********************************/
/***********************************************************************************/

// start a new mission
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
	nav_queue_init();

	last_mission.isactive = false;
	smq_send_log_info("mission stopped!");

};
