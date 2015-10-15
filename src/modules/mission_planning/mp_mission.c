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
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "mp_mission.h"

#include <drivers/drv_hrt.h>

#include "../pp_navigation_helper.h"
#include "../pp_navigator.h"
#include "../pp_send_msg_qgc.h"
#include "../pp_communication_buffer.h"

#include "../pp_config.h"



/***********************************************************************************/
/*****  V A R I A B L E S  *********************************************************/
/***********************************************************************************/

static struct {
	NEDpoint O1; 	// position of the first buoy (topleft)
	NEDpoint O2;
	NEDpoint O3;
	NEDpoint O4;
	float dist;  	// distance between two buoys [m]
	float rotation; // rotation of the field around buoy O1 [rad]
} comp_frame;

static struct {
	uint8_t type; // task type; 1 = triangular course, 2 = station-keeping, 3 = area-scanning, 4 = fleet-race
	bool isactive;
} curr_task;

/* buoys */
static NEDpoint O1;



/***********************************************************************************/
/*****  P U B L I C    F U N C T I O N S  ******************************************/
/***********************************************************************************/

/*
 * set the configuration of the race-Field
 *
 * @param O1long: longitude of buoy 1 [deg]
 * @param O1lat: latitude of buoy 1 [deg]
 * @param dist: distances between two buoys [m]
 * @param rotation: rotation of the frame around O1 [deg] (0 deg = y axis)
 */
void mi_set_racefield(float O1long, float O1lat, float dist, float rotation) {

	comp_frame.dist = dist;
	comp_frame.rotation = DEG2RAD * rotation;
	comp_frame.O1.northx = ;
	comp_frame.O1.easty = ;
	comp_frame.O2.northx = comp_frame.O1.northx + sin(comp_frame.rotation)*comp_frame.dist;
	comp_frame.O2.easty = comp_frame.O1.easty + cos(comp_frame.rotation)*comp_frame.dist;
	comp_frame.O3.northx = comp_frame.O1.northx + sin(comp_frame.rotation)*comp_frame.dist;;
	comp_frame.O3.easty = ;
	comp_frame.O4.northx = ;
	comp_frame.O4.easty = ;

	config.o1 = o1;

}

/*
 * set buoys
 */
void mi_init(void) {

	O1 = racefield.O1;
	O2.northx = O1.northx;
	O2.easty =  O1.easty + racefield.dist;
	O3.northx = O1.northx - racefield.dist;
	O3.easty = O1.easty;
	O4.northx = O1.northx - racefield.dist;
	O4.easty = O1.easty + racefield.dist;
}

/*
 * Set a new Competition Task
 *
 * @param: tasknum: Number of the task that should be loaded
 *
 */
bool mi_start_task(uint8_t task_type) {

	if(tasknum == state.curtask) {
		return false;
	}

	//Assign the current Task-Number
	state.curtask = tasknum;

	//Inform QGround Control that a new mission is started
	smq_send_log_info("New Mission started! (JW)");

	return true;

}


/*
 * Periodically called function by the main-program loop
 *
 */
bool mi_handler(void) {

	uint64_t curtime = hrt_absolute_time();

	/*if(stationkeeping_isinside == true && countdown_running == false) {
		countdown = true;
		countdown_running = true;
		countdown_ms = 0; //initialize the countdown

		lastcall = curtime;
	}*/

	//We start the timer, when we switch to autonomous mode
	/*if(cb_is_autonomous_mode() == true && countdown_running == false) {
		countdown = true;
		countdown_running = true;
		countdown_ms = 0;

		lastcall = curtime;
	}*/

	uint64_t difference = curtime-lastcall;

	if(difference > 2e6) {
		//Do not call the mission planner every time in the main-loop, but only every two seconds

		lastcall = curtime;

		if(countdown == true) {
			//A countdown is set

			countdown_ms += difference;

			if(countdown_ms >= countdown_ms_top) {
				//The countdown is over => set countdown-flag to false and start original mission
				countdown = false;
				countdown_ms = 0;

				//Check, what the action is, when the countdown is over (for each task)
				switch (state.curtask) {
					case 6: { //THIS IS THE STATION KEEPING TASK
						//We need to leave the square after 5 minutes

						NEDpoint wp4;
						wp4.northx = -25;
						wp4.easty = 25;

						nav_queue_init();		 //We reset the queue to zero-waypoints
						nav_set_target_ned(wp4); //We add a new target, that is OUTSIDE of the square

						break;
					}
					/*case 13: { //THIS IS THE FLEET-RACE
						//We switch now to Mission 14, which is the race-course (fleet-race)

						mi_set_new_task(14);
						state.curtask = 14;

						break;
					}*/
					default: {
						break;
					}
				}
			}
		}

	} //if lastcall

	return true;

}

/***********************************************************************************/
/*****  P R I V A T E    F U N C T I O N S  ****************************************/
/***********************************************************************************/

/*
 * Detect, if we are inside the area defined by the buoys
 * CORRECT!!!
 *
 * @return true, if the boat position is inside the area
 */
bool mi_isinside(NEDpoint boatpos) {

	float x = boatpos.northx;
	float y = boatpos.easty;

	bool inside = false;

	if(x < O1.northx && x > O3.northx && y > O1.easty && y < O2.easty) {
		inside = true;
	}

	//if(y > O1.easty && y < O2.easty) {
	//	inside = inside && true;
	//}

	if(inside == true && stationkeeping_isinside == false) {
		smq_send_log_info("[JW] inside the competition area");
	}

	if(stationkeeping_isinside == true && inside == false) {
		smq_send_log_info("[JW] just exited the competion area");
	}

	stationkeeping_isinside = inside;

	return inside;
}
