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

mission cur_mission;
int ob_tf_count = 0, wp_tf_count = 0;
bool mi_is_set = false;

char buffer_mi[50];

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

void mp_mi_handler(int id) {

	bool found = false;

	// check if id exists in mi_queue
	for (int i = 0; i < mi_list.size; i++) {
		if (mi_list.list[i].id == id) {

			// set current mission
			sprintf(buffer_mi, "mission found (id = %d)\n", id);
			printf(buffer_mi);
			mp_send_log_info(buffer_mi);

			cur_mission = mi_list.list[i];
			mi_is_set = true;
			found = true;
		}
	}

	if (found == false) {
		mi_is_set = false;
		memset(&cur_mission, 0, sizeof(cur_mission));
		sprintf(buffer_mi, "mission not found (id = %d)\n", id);
		printf(buffer_mi);
		mp_send_log_info(buffer_mi);
		return;
	}

	mp_tf_mi_init();
	switch (cur_mission.type) {
		case 0:
			break;
		default:
			break;
	};
}

void mp_tf_mi_init(void) {
	mp_cb_new_mission(cur_mission.id);
	ob_tf_count = 0;
	wp_tf_count = 0;
}

void mp_tf_mi_data(struct mp_structs_topics_s *strs) {
	if (mi_is_set == true) {
		if (ob_tf_count < cur_mission.obstacle_count & strs->mi_ack.obs_ack == 1) {
			mp_cb_new_obstacle(cur_mission.obstacles[ob_tf_count].center.latitude, cur_mission.obstacles[ob_tf_count].center.longitude);
			ob_tf_count++;
		}
		if (wp_tf_count < cur_mission.waypoint_count & strs->mi_ack.tar_ack == 1) {
			mp_cb_new_target(cur_mission.waypoints[wp_tf_count].latitude, cur_mission.waypoints[wp_tf_count].longitude);
			wp_tf_count++;
		}
	}
}

void mp_reset_lists(void) {

	memset(&mi_list, 0, sizeof(mi_list));
	memset(&fr_list, 0, sizeof(fr_list));
}

int mp_add_mi_to_list(mission *mi) {

	// check if mission queue is full
	if (mi_list.size < MAX_NUM_MI) {
		mi_list.list[mi_list.size] = *mi;
		mi_list.size++;
		sprintf(buffer_mi, "added mission %s to list\n", mi->name);
		printf(buffer_mi);
		mp_send_log_info(buffer_mi);
		return 1;
	}

	sprintf(buffer_mi, "failed to add mission %s to list (max queue size reached)\n", mi->name);
	printf(buffer_mi);
	mp_send_log_info(buffer_mi);
	return 0;
}

int mp_add_fr_to_list(frame *fr) {

	// check if frame list is full
	if (fr_list.size < MAX_NUM_FR) {
		fr_list.list[fr_list.size] = *fr;
		fr_list.size++;
		sprintf(buffer_mi, "added frame %s to list\n", fr->name);
		printf(buffer_mi);
		mp_send_log_info(buffer_mi);
		return 1;
	}

	sprintf(buffer_mi, "failed to add frame %s to list (max list size reached)\n", fr->name);
	printf(buffer_mi);
	mp_send_log_info(buffer_mi);
	return 0;
}
