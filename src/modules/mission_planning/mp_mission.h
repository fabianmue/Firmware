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

#include "mp_params.h"
#include "mp_topics_handler.h"

#define MAX_NUM_FR 5
#define MAX_NUM_MI 5

#define MI_SEL_DEBUG 0

/***********************************************************************************/
/*****  V A R I A B L E S  *********************************************************/
/***********************************************************************************/

typedef struct waypoint_s {
	double latitude;
	double longitude;
} waypoint;

typedef struct obstacle_s {
	waypoint center;
	double radius;
} obstacle;

typedef struct buoy_s {
	obstacle body;
	int rotation;
} buoy;

typedef struct frame_s {
	int id;
	buoy buoys[MAX_NUM_BU];
	int buoy_count;
} frame;

typedef struct mission_s {
	int id;
	waypoint waypoints[MAX_NUM_WP];
	int waypoint_count;
	obstacle obstacles[MAX_NUM_OB];
	int obstacle_count;
} mission;

/***********************************************************************************/
/*****  F U N C T I O N   D E C L A R A T I O N S  *********************************/
/***********************************************************************************/

void mp_mi_handler(int id, int wp_ack, int ob_ack);

void mp_list_reset(void);

int mp_fr_convert(rd_frame *fr);

int mp_mi_uc_convert(rd_mission_uc *rd_mi_uc);

int mp_mi_tr_convert(rd_mission_tr *rd_mi_tr);

int mp_mi_sk_convert(rd_mission_sk *rd_mi_sk);

int mp_mi_as_convert(rd_mission_as *rd_mi_as);

frame* mp_get_frame_by_id(int id);

mission* mp_get_mission_by_id(int id);

int mp_add_fr_to_list(frame fr);

int mp_add_mi_to_list(mission mi);

#endif /* MP_MISSION_H_ */
