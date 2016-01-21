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
#include "mp_send_msg_qgc.h"
#include "mp_communication_buffer.h"

/***********************************************************************************/
/*****  V A R I A B L E S  *********************************************************/
/***********************************************************************************/

int32_t sel_id = 0, tf_done = 0;
mission *selected_mission = NULL;
int wp_tf_count = 0, ob_tf_count = 0;

char buffer_mi[60];

static struct {
	int size;
	frame list[MAX_NUM_FR];
} fr_list;

static struct {
	int size;
	mission list[MAX_NUM_MI];
} mi_list;

struct mp_published_fd_s *mp_pubs;

/***********************************************************************************/
/*****  F U N C T I O N   D E F I N I T I O N S  ***********************************/
/***********************************************************************************/

void mp_mi_handler(int id, int wp_ack, int ob_ack) {

	if (MI_SEL_DEBUG != 0) {
		id = 1;
	}

	if (selected_mission == NULL | id != sel_id) {

		// try to get selected mission from list
		selected_mission = mp_get_mission_by_id(id);
		if (selected_mission == NULL) {
			sprintf(buffer_mi, "mission with id %d not found\n", id);
			printf(buffer_mi);
			mp_send_log_info(buffer_mi);
			return;
		}

		// new id selected
		sprintf(buffer_mi, "mission with id %d found\n", id);
		printf(buffer_mi);
		mp_send_log_info(buffer_mi);

		sel_id = selected_mission->id;
		mp_cb_new_mission_id(sel_id);
		tf_done = 0;
		mp_param_QGC_set(tf_done);
		wp_tf_count = 0;
		ob_tf_count = 0;
	}

	if (id == sel_id & selected_mission != NULL) {

		// check if all waypoints have been transfered
		if (wp_tf_count >= selected_mission->waypoint_count & ob_tf_count >= selected_mission->obstacle_count) {
			tf_done = 1;
			mp_param_QGC_set(tf_done);
			return;
		}

		// transfer next waypoint
		if (wp_ack != 0 & wp_tf_count < selected_mission->waypoint_count) {
			mp_cb_new_waypoint(selected_mission->waypoints[wp_tf_count].latitude, selected_mission->waypoints[wp_tf_count].longitude);
			wp_tf_count++;

			sprintf(buffer_mi, "new wp sent to cb (lat = %3.8f, lon = %3.8f)", selected_mission->waypoints[wp_tf_count].latitude, selected_mission->waypoints[wp_tf_count].longitude);
			printf(buffer_mi);
			mp_send_log_info(buffer_mi);
		}

		// transfer next obstacle
		if (ob_ack != 0 & ob_tf_count < selected_mission->obstacle_count) {
			mp_cb_new_obstacle(selected_mission->obstacles[ob_tf_count].center.latitude, selected_mission->obstacles[ob_tf_count].center.longitude, selected_mission->obstacles[ob_tf_count].radius);
			ob_tf_count++;

			sprintf(buffer_mi, "new ob sent to cb (lat = %3.8f, lon = %3.8f)", selected_mission->obstacles[ob_tf_count].center.latitude, selected_mission->obstacles[ob_tf_count].center.longitude);
			printf(buffer_mi);
			mp_send_log_info(buffer_mi);
		}
	}
}

// @brief: reset frame and mission list
void mp_list_reset(void) {
	memset(&mi_list, 0, sizeof(mi_list));
	memset(&fr_list, 0, sizeof(fr_list));
}

// @brief: convert read frame and add to frame list
int mp_fr_convert(rd_frame *rd_fr) {
	frame fr;
	fr.id = rd_fr->id;
	for (int i = 0; i < rd_fr->buoy_count; i++) {
		fr.buoys[i].body.center.latitude = rd_fr->buoys[i].body.center.latitude;
		fr.buoys[i].body.center.longitude = rd_fr->buoys[i].body.center.longitude;
		fr.buoys[i].body.radius = rd_fr->buoys[i].body.radius;
		fr.buoys[i].rotation = rd_fr->buoys[i].rotation;
	}
	fr.buoy_count = rd_fr->buoy_count;
	int sc = mp_add_fr_to_list(fr);
	if (sc == 1) {
		return 1;
	}
	return 0;
}

// @brief: convert read unconstrained mission and add to mission list
int mp_mi_uc_convert(rd_mission_uc *rd_mi_uc) {
	mission mi;
	mi.id = rd_mi_uc->mi_base.id;
	for (int i = 0; i < rd_mi_uc->waypoint_count; i++) {
		mi.waypoints[i].latitude = rd_mi_uc->waypoints[i].latitude;
		mi.waypoints[i].longitude = rd_mi_uc->waypoints[i].longitude;
	}
	mi.waypoint_count = rd_mi_uc->waypoint_count;
	for (int i = 0; i < rd_mi_uc->obstacle_count; i++) {
		mi.obstacles[i].center.latitude = rd_mi_uc->obstacles[i].center.latitude;
		mi.obstacles[i].center.longitude = rd_mi_uc->obstacles[i].center.longitude;
		mi.obstacles[i].radius = rd_mi_uc->obstacles[i].radius;
	}
	mi.obstacle_count = rd_mi_uc->obstacle_count;
	int sc = mp_add_mi_to_list(mi);
	if (sc == 1) {
		return 1;
	}
	return 0;
}

// @brief: convert read triangular mission and add to mission list
int mp_mi_tr_convert(rd_mission_tr *rd_mi_tr) {
	mission mi;
	mi.id = rd_mi_tr->mi_base.id;


	int sc = mp_add_mi_to_list(mi);
	if (sc == 1) {
		return 1;
	}
	return 0;
}

// @brief: convert read station keeping mission and add to mission list
int mp_mi_sk_convert(rd_mission_sk *rd_mi_sk) {
	mission mi;
	mi.id = rd_mi_sk->mi_base.id;


	int sc = mp_add_mi_to_list(mi);
	if (sc == 1) {
		return 1;
	}
	return 0;
}

// @brief: convert read area scanning mission and add to mission list
int mp_mi_as_convert(rd_mission_as *rd_mi_as) {
	mission mi;
	mi.id = rd_mi_as->mi_base.id;


	int sc = mp_add_mi_to_list(mi);
	if (sc == 1) {
		return 1;
	}
	return 0;
}

// @brief: get frame from frame list by id
frame* mp_get_frame_by_id(int id) {
	for (int i = 0; i < fr_list.size; i++) {
		if (fr_list.list[i].id == id) {
			return &fr_list.list[i];
		}
	}
	return NULL;
}

// @brief: get mission from mission list by id
mission* mp_get_mission_by_id(int id) {

	for (int i = 0; i < mi_list.size; i++) {
		if (mi_list.list[i].id == id) {
			return &mi_list.list[i];
		}
	}
	return NULL;
}

// @brief: add frame to frame list if the list is not yet full
int mp_add_fr_to_list(frame fr) {

	// check if frame list is full
	if (fr_list.size < MAX_NUM_FR) {
		fr_list.list[fr_list.size] = fr;
		fr_list.size++;
		sprintf(buffer_mi, "added frame to list (id = %d)\n", fr.id);
		printf(buffer_mi);
		mp_send_log_info(buffer_mi);
		return 1;
	}
	sprintf(buffer_mi, "failed to add frame to list (id = %d, max list size reached)\n", fr.id);
	printf(buffer_mi);
	mp_send_log_info(buffer_mi);
	return 0;
}

// @brief: add mission to mission list if the list is not yet full
int mp_add_mi_to_list(mission mi) {

	// check if frame list is full
	if (mi_list.size < MAX_NUM_MI) {
		mi_list.list[mi_list.size] = mi;
		mi_list.size++;
		sprintf(buffer_mi, "added mission to list (id = %d)\n", mi.id);
		printf(buffer_mi);
		mp_send_log_info(buffer_mi);
		return 1;
	}
	sprintf(buffer_mi, "failed to add mission to list (id = %d, max list size reached)\n", mi.id);
	printf(buffer_mi);
	mp_send_log_info(buffer_mi);
	return 0;
}
