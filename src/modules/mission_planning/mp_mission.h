/*
 * mp_mission.h
 *
 *  Created on: 25.05.2015
 *      Author: Jonas
 *
 *      09.10.2015: moved from path_planning to mission_planning app (Fabian Müller)
 */

#ifndef MP_MISSION_H_
#define MP_MISSION_H_

#include <stdint.h>
#include <stdbool.h>
#include "mp_read_params.h"


/***********************************************************************************/
/*****  V A R I A B L E S  *********************************************************/
/***********************************************************************************/


typedef struct mission_s {
	uint8_t type; 	// mission type; 0 = triangular course, 1 = station-keeping, 2 = area-scanning, 3 = fleet-race, 4 = unconstrained
	NEDpoint waypoints[100];
	bool isactive;
} mission;


/***********************************************************************************/
/*****  F U N C T I O N   D E C L A R A T I O N S  *********************************/
/***********************************************************************************/


/* @brief start new mission */
bool mp_start_mission(mission new_mission);

/* @brief stop current mission */
void mp_stop_mission(void);


#endif /* MP_MISSION_H_ */
