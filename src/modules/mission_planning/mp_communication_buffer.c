/*
 * mp_communication_buffer.c
 *
 *  Created on: 04.12.2015
 *      Author: Fabian
 */

#include "mp_communication_buffer.h"
#include <drivers/drv_hrt.h>
#include <stdio.h>
#include <string.h>

//state variables
static struct mission_planning_s mp;
static bool mp_updated = false;

/**
 * send an int-value for debug purposes to QGC
 */
bool cb_new_target(float north, float east) {

	// TODO: convert NED to lat-lon
	mp.tar_lat = 0;
	mp.tar_lon = 0;
	mp.tar_ned_north = north;
	mp.tar_ned_east = east;

	mp_updated = true;
	return true;
}

void cb_publish_mp_if_updated(void) {

    // if mission_planning topic has been updated, publish it
    // make sure this happens not too often, otherwise the processor load is too high
    if(mp_updated == true) {

        mp.timestamp = hrt_absolute_time();
        th_publish_mission_planning(&mp);
        mp_updated = false;
    }
}

void cb_init(void) {

    //clean memory
    memset(&mp, 0, sizeof(mp));

    mp.tar_lat = 0;
    mp.tar_lon = 0;
    mp.tar_ned_north = 0;
    mp.tar_ned_east = 0;

    mp_updated = true;
    cb_publish_pp_if_updated();
}
