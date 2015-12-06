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

#include <drivers/drv_hrt.h>

#include "mp_mission.h"
#include "mp_params.h"

#include "../pp_send_msg_qgc.h"

/***********************************************************************************/
/*****  V A R I A B L E S  *********************************************************/
/***********************************************************************************/

static struct {
	int cur_task;
	int queue_size;
	mission queue[MAX_ELEM];
} mission_queue = {
	.cur_task = NULL,
	.queue_size = 0
};

/***********************************************************************************/
/*****  F U N C T I O N   D E F I N I T I O N S  ***********************************/
/***********************************************************************************/

void mp_start_mission(mission new_mission) {

	smq_send_log_info("new mission started");

	nav_queue_init();


	// set obstacles
	int ob_count = 0;
	while (*new_mission.obstacles) {

		// convert lat-lon obstacle to NED
		NEDpoint ob;

		// set obstacle
		nav_set_obstacle_ned(ob_count, ob);
	}



	// set waypoints
	while (*new_mission.waypoints) {

		// convert lat-lon waypoint to NED
		NEDpoint wp;

		// set target
		nav_set_target_ned(wp);
	}

	switch (new_mission.type) {
		case 0:
			// unconstrained

		case 1:

		case 2:

		case 3:

		case 4:
	}

};
