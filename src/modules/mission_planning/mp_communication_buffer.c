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

bool mp_cb_new_target(float lat, float lon) {

	mp.tar_lat = lat;
	mp.tar_lon = lon;

	/*
	Point tar;
	tar.lat = lat;
	tar.lon = lon;
	tar.alt = 0;
	NEDpoint tar_ned = nh_geo2ned(tar);
	mp.tar_ned_north = tar_ned.northx;
	mp.tar_ned_east = tar_ned.easty;
	*/

	mp_updated = true;
	return true;
}

bool mp_cb_new_obstacle(void) {

	mp.ob_num++;

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

    mp.tar_lat = 0;
    mp.tar_lon = 0;

    /*
    mp.tar_ned_north = 0;
    mp.tar_ned_east = 0;
	*/

    mp_updated = true;
    mp_cb_publish_if_updated();
}
