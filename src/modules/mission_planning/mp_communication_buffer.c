/*
 * mp_communication_buffer.c
 *
 *  Created on: 04.12.2015
 *      Author: Fabian
 */

#include <drivers/drv_hrt.h>
#include <stdio.h>
#include <string.h>

#include "mp_communication_buffer.h"

// #include <path_planning/pp_navigation_helper.h>

/***********************************************************************************/
/*****  V A R I A B L E S  *********************************************************/
/***********************************************************************************/

static struct mission_planning_s mp;
static bool mp_updated = false;

/***********************************************************************************/
/*****  F U N C T I O N   D E F I N I T I O N S  ***********************************/
/***********************************************************************************/

bool mp_cb_sd_read(void) {

	mp.sd_read = 1;

	mp_updated = true;
	return true;
}

bool mp_cb_new_mission(int id) {

	mp.mi_id = id;

	mp_updated = true;
	return true;
}

bool mp_cb_new_target(float tar_lat, float tar_lon) {

	mp.tar_lat = tar_lat;
	mp.tar_lon = tar_lon;
	mp.tar_num++;

	mp_updated = true;
	return true;
}

bool mp_cb_new_obstacle(float obs_lat, float obs_lon) {

	mp.obs_lat = obs_lat;
	mp.obs_lon = obs_lon;
	mp.obs_num++;

	mp_updated = true;
	return true;
}

void mp_cb_publish_if_updated(void) {

    // if mission_planning topic has been updated, publish it
    // make sure this happens not too often, otherwise the processor load is too high
    if(mp_updated == true) {

        mp.timestamp = hrt_absolute_time();
        mp_th_publish(&mp);
        mp_updated = false;
    }
}

void mp_cb_init(void) {

    //clean memory
    memset(&mp, 0, sizeof(mp));

    mp_updated = true;
    mp_cb_publish_if_updated();
}
