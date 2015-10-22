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

#define DEG2RAD      0.0174532925199433f
#define PI           3.14159265358979323846f

/***********************************************************************************/
/*****  V A R I A B L E   D E C L A R A T I O N S **********************************/
/***********************************************************************************/

typedef struct NEDpoint_s{
	float northx;	//north component
	float easty;	//east component
	float downz;	//down component
} NEDpoint;

typedef struct frame_s {
	NEDpoint O1; 	// position of the first buoy (topleft)
	NEDpoint O2;
	NEDpoint O3;
	NEDpoint O4;
	float dist;  	// distance between two buoys [m]
	float rotation; // rotation of the field around buoy O1 [rad]
} frame;

typedef struct mission_s {
	uint8_t type; 	// mission type; 0 = triangular course, 1 = station-keeping, 2 = area-scanning, 3 = fleet-race, 4 = unconstrained
	NEDpoint waypoints[100];
	bool isactive;
} mission;

/***********************************************************************************/
/*****  F U N C T I O N   D E C L A R A T I O N S  *********************************/
/***********************************************************************************/

/* @brief set race field parameters */
void mp_set_racefield(float O1long, float O1lat, float dist, float rotation);

/* @brief start new mission */
bool mp_start_mission(mission new_mission);

/* @brief stop current mission */
void mp_stop_mission(void);

/* @brief mission planning handler */
bool mp_handler(void);

#endif /* MP_MISSION_H_ */
