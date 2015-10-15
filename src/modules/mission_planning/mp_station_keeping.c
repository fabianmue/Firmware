/*
 * mp_station_keeping.c
 *
 *  Created on: 09.10.2015
 *      Author: Fabian
 */

#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <mp_mission.h>

/***********************************************************************************/
/*****  V A R I A B L E S  *********************************************************/
/***********************************************************************************/

static struct {
	uint8_t curtask; // current task number
} state = {
	.curtask = 0
};

static bool stationkeeping_isinside = false;

static uint64_t countdown_ms = 300e6; // countdown in milliseconds

/***********************************************************************************/
/*****  FUNCTION DECLARATIONS  *****************************************************/
/***********************************************************************************/

bool mi_station_keeping(uint8_t tasknum);

/***********************************************************************************/
/*****  P U B L I C    F U N C T I O N S  ******************************************/
/***********************************************************************************/

bool mi_station_keeping(uint8_t tasknum) {

	if(tasknum == state.curtask) {
		return false;
	}

	// assign the current task-Number
	state.curtask = tasknum;

	// inform QGroundControl that a new mission is started
	smq_send_log_info("New Mission started! (JW)");

	NEDpoint wp1, wp2, wp3, wp4; // initiate waypoints inside and outside race area

	/*      O1--------------------O2
	 *      |                      |
	 *      |                      |
	 *      |                      |
	 * ---------->wp1<---->wp2--------->wp3
	 *      |                      |
	 *      |                      |
	 *      |                      |
	 *      O3--------------------O4
	 *
	 *  x
	 *  ^
	 *  |
	 *  |--> y
	 */

	mi_init();

	wp1.easty = O1.easty + config.dist/3;
	wp1.northx = (O1.northx + O3.northx)/2;

	wp2.easty = O1.easty + 2*config.dist/3;
	wp2.northx = (O1.northx + O3.northx)/2;

	wp3.easty = O2.easty + config.dist/2;
	wp3.northx = (O1.northx + O3.northx)/2;

	if(countdown > 0) {

		nav_set_target_ned(wp1);
		nav_set_target_ned(wp2);

	}

	nav_set_target_ned(wp3);

	return true;

}

