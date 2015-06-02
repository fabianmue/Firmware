/*
 * kt_topic_handler.c
 *
 *  Created on: 27.05.2015
 *      Author: Jonas
 */


#include "kt_topic_handler.h"

#include <stdint.h>
#include <string.h>
#include <uORB/uORB.h>
#include <uORB/topics/path_planning_kalman.h>


/***********************************************************************************/
/*****  V A R I A B L E S  *********************************************************/
/***********************************************************************************/

static struct path_planning_kalman_s ppk;

static orb_advert_t adv_ppk;	//Advert topic

static bool flag_update = false; //This flag is set to one, if any values are to be updated


/***********************************************************************************/
/*****  F U N C T I O N   P R O T O T Y P E S **************************************/
/***********************************************************************************/



/***********************************************************************************/
/*****  P U B L I C    F U N C T I O N S  ******************************************/
/***********************************************************************************/

/**
 * Init the use of the UORB-Topics
 *
 */
bool th_init(void) {

	//Subscribe to the Path Planning Kalman Topic
	/*sub_ppk = orb_subscribe(ORB_ID(path_planning_kalman));

	if(sub_ppk == -1){
		warnx(" error on subscribing on path_planning_kalman \n");
	    return false;
	}*/

    memset(&ppk, 0, sizeof(ppk));
    adv_ppk = orb_advertise(ORB_ID(path_planning_kalman), &ppk);

	return true;
}


/**
 * Update the values in the topic
 * NOTE: This function must be called in regular intervals
 *
 */
bool th_update(void) {

	if(flag_update == true) {
		//Update the Path Planning Kalman Topic
		orb_publish(ORB_ID(path_planning_kalman), adv_ppk, &ppk);

		//Reset the Flag
		flag_update = false;
	}

	return true;
}


/**
 * Set the number of currently tracked objects
 *
 * @param tracknum: Number of tracks currently tracked
 */
bool th_set_nroftracks(uint16_t tracknum) {

	//Update the value in the topic
	ppk.tracknum = tracknum;

	//Set the flag such that the topic is advertised
	flag_update = true;

	return true;

}


/**
 * Set the number of tracked objects newly added to the tracking-list
 *
 * @param newtracknum: Number of newly added tracks
 */
bool th_set_nrofnewtracks(uint16_t newtracknum) {

	//Update the value in the topic
	ppk.newtracknum = newtracknum;

	//Set the flag such that the topic is advertised
	flag_update = true;

	return true;
}


/**
 * Set the number of tracked objects that were refound in this step
 *
 * @param refoundnum: Number of refound tracks in this step
 */
bool th_set_nrofrefoundtracks(uint16_t refoundnum) {

	//Update the value in the topic
	ppk.refoundnum = refoundnum;

	//Set the flag such that the topic is advertised
	flag_update = true;

	return true;
}
