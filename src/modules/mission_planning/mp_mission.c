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

#include "mp_send_msg_qgc.h"
#include "mp_communication_buffer.h"
#include "../path_planning/pp_navigation_helper.h"

/***********************************************************************************/
/*****  V A R I A B L E S  *********************************************************/
/***********************************************************************************/

mission cur_mission;
bool mi_is_set = false;
frame cur_frame;
bool fr_is_set = false;

char buffer_mi[50];

static struct {
	int size;
	frame list[MAX_NUM_FR];
} fr_list = {
	.size = 0
};

static struct {
	int size;
	mission queue[MAX_NUM_MI];
} mi_queue = {
	.size = 0
};

/***********************************************************************************/
/*****  F U N C T I O N   D E F I N I T I O N S  ***********************************/
/***********************************************************************************/

void mp_mi_handler(int id) {

	bool found = false;

	// check if id exists in mi_queue
	for (int i = 0; i < mi_queue.size; i++) {

		if (mi_queue.queue[i].id == id) {

			// set current mission
			cur_mission = mi_queue.queue[i];
			mi_is_set = true;
			found = true;
		}
	}

	if (found == false) {

		mi_is_set = false;
		sprintf(buffer_mi, "failed to start mission (id %d not found)\n", id);
		printf(buffer_mi);
		mp_send_log_info(buffer_mi);
		return;
	}

	// try to set frame
	mp_init_mi();
	if (fr_is_set != true) {

		// frame setting failed - reset
		sprintf(buffer_mi, "failed to start mission %s (id %d, failed to set frame)\n", cur_mission.name, id);
		printf(buffer_mi);
		mp_send_log_info(buffer_mi);
		mi_is_set = false;
		return;
	}

	// do mission
	mp_execute_mi();
}

void mp_init_mi(void) {

	// check frame list for frame with fr_id of the mission
	for (int i = 0; i < fr_list.size; i++) {
		if (fr_list.list[i].id == cur_mission.fr_id) {
			cur_frame = fr_list.list[i];
			fr_is_set = true;
			sprintf(buffer_mi, "frame set to %s (id %d)\n", cur_frame.name, cur_frame.id);
			printf(buffer_mi);
			mp_send_log_info(buffer_mi);
			return;
		}
	}
	fr_is_set = false;
	sprintf(buffer_mi, "failed to set frame (id %d, not found)\n", cur_mission.fr_id);
	printf(buffer_mi);
	mp_send_log_info(buffer_mi);
}

void mp_execute_mi(void) {

	// advertise
	sprintf(buffer_mi, "mission %s started (id %d)\n", cur_mission.name, cur_mission.id);
	printf(buffer_mi);
	mp_send_log_info(buffer_mi);

	// reset navigation queue
	nav_queue_init();

	// set all obstacles
	for (int i = 0; i < MAX_NUM_OB; i++) {
		if (cur_mission.obstacles[i].center.latitude == 0 & cur_mission.obstacles[i].center.longitude == 0) {
			break;
		}
		Point geo;
		geo.lat = cur_mission.obstacles[i].center.latitude;
		geo.lon = cur_mission.obstacles[i].center.longitude;
		geo.alt = 0;
		NEDpoint obs = nh_geo2ned(geo);
		nav_set_obstacle_ned(i, obs);
		mp_cb_new_obstacle();	// msg to QGC
	}

	for (int j = 0; j < MAX_NUM_WP; j++) {
		if (cur_mission.waypoints[j].latitude == 0 & cur_mission.waypoints[j].longitude == 0) {
			break;
		}
		Point geo;
		geo.lat = cur_mission.waypoints[j].latitude;
		geo.lon = cur_mission.waypoints[j].longitude;
		geo.alt = 0;
		NEDpoint wp = nh_geo2ned(geo);
		nav_set_target_ned(wp);
		mp_cb_new_target(geo.lat, geo.lon);	// msg to QGC
	}

	mi_is_set = false;
	fr_is_set = false;
}

int mp_add_mi_to_queue(mission *mi) {

	mission mi_val;
	mi_val = *mi;

	// check if mission queue is full
	if (mi_queue.size == MAX_NUM_MI) {

		sprintf(buffer_mi, "failed to add mission %s to mission queue (max queue size reached)\n", mi_val.name);
		printf(buffer_mi);
		mp_send_log_info(buffer_mi);
		return -1;
	}
	// disp_mi(mi);
	mi_queue.queue[mi_queue.size] = mi_val;
	mi_queue.size++;
	sprintf(buffer_mi, "added mission %s to mission queue\n", mi_val.name);
	printf(buffer_mi);
	mp_send_log_info(buffer_mi);
	return 0;
}

int mp_add_fr_to_queue(frame *fr) {

	frame fr_val;
	fr_val = *fr;

	// check if frame list is full
	if (fr_list.size == MAX_NUM_FR) {

		sprintf(buffer_mi, "failed to add frame %s to frame list (max list size reached)\n", fr_val.name);
		printf(buffer_mi);
		mp_send_log_info(buffer_mi);
		return -1;
	}
	// disp_fr(fr);
	fr_list.list[fr_list.size] = fr_val;
	fr_list.size++;
	sprintf(buffer_mi, "added frame %s to frame list\n", fr_val.name);
	printf(buffer_mi);
	mp_send_log_info(buffer_mi);
	return 0;
}
