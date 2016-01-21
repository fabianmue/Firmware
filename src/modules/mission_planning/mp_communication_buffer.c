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

/***********************************************************************************/
/*****  V A R I A B L E S  *********************************************************/
/***********************************************************************************/

static struct mission_planning_s mp;

static bool mp_updated = false;

/***********************************************************************************/
/*****  F U N C T I O N   D E F I N I T I O N S  ***********************************/
/***********************************************************************************/

bool mp_cb_new_mission_id(int id) {

	//
	mp.mi_id = id;

	mp_updated = true;
	return true;
}

bool mp_cb_new_waypoint(float wp_lat, float wp_lon) {

	//
	mp.wp_lat = wp_lat;
	mp.wp_lon = wp_lon;
	mp.wp_count++;

	mp_updated = true;
	return true;
}

bool mp_cb_new_obstacle(float ob_lat, float ob_lon, float ob_rad) {

	//
	mp.ob_lat = ob_lat;
	mp.ob_lon = ob_lon;
	mp.ob_rad = ob_rad;
	mp.ob_count++;

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
