/*
 * mission.c
 *
 * This file provides functions for setting a competition Race-Field in WRSC 2015. The file defines "Missions"
 *
% O is a buoy
%
%   |
%   | Wind direction from true North
%   V
%
%   O1----------------O2
%   |                 |
%   |                 |
%   Finish            |
%   |                 |
%   |                 |
%   |                 |
%   O3-----Start -----O4
%
%
%  ^x (North)
%  |
%  |---->y (East)
%
 *
 *
 *
 *
 *  Created on: 22.05.2015
 *      Author: Jonas Wirz <wirzjo@student.ethz.ch>
 */


#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "mission.h"

#include <drivers/drv_hrt.h>

#include "../pp_navigation_helper.h"
#include "../pp_navigator.h"
#include "../pp_send_msg_qgc.h"
#include "../pp_communication_buffer.h"
#include "../pp_send_msg_qgc.h"


#include "../pp_config.h"






/***********************************************************************************/
/*****  V A R I A B L E S  *********************************************************/
/***********************************************************************************/

static struct {
	NEDpoint o1; //Position of the first buoy
	float dist;  //Distance between two buoys [m]
	float rotation; //Rotation of the field around buoy o1 [rad]
} config = {
	.dist = 45,
	.rotation = 0
};

static struct {
	uint8_t curtask; //Current Task number
} state = {
	.curtask = 0
};

static bool stationkeeping_isinside = false;
static bool countdown_running = false;


static uint64_t lastcall = 0; //Timestamp of the last call to the handler-function

static bool countdown = false; //Set true, if a countdown is needed
static uint64_t countdown_ms = 0; //Number of milliseconds for the countdown
static uint64_t countdown_ms_top = 278e6; //Time to reach in milliseconds (5min)

//static char txt_msg[150]; //Buffer for QGround Control Messages


//Calcualte the positions of the Buoys
static NEDpoint O1;
static NEDpoint O2;
static NEDpoint O3;
static NEDpoint O4;



/***********************************************************************************/
/*****  F U N C T I O N   P R O T O T Y P E S **************************************/
/***********************************************************************************/





/***********************************************************************************/
/*****  P U B L I C    F U N C T I O N S  ******************************************/
/***********************************************************************************/

void mi_init(void) {

	O1 = config.o1;

	//Set the buoys
	O2.northx = O1.northx;
	O2.easty =  O1.easty + config.dist;
	O3.northx = O1.northx - config.dist;
	O3.easty = O1.easty;
	O4.northx = O1.northx - config.dist;
	O4.easty = O1.easty + config.dist;

}



/*
 * Set the configuration of the race-Field
 *
 * @param dist: Distance between two buoys [m]
 * @param o1x, o1y: Position of buoy O1 in NED-Frame [m]
 * @param rotation: Rotation of the Frame around O1 [°]
 */
void mi_set_configuration(float dist, float o1x, float o1y, float rotation) {

	if(dist > 0) {
		config.dist = dist;
	}

	if(rotation > 0) {
		config.rotation = DEG2RAD * rotation;
	}

	NEDpoint o1;
	o1.northx = o1x;
	o1.easty = o1y;

	config.o1 = o1;

}


/*
 * Set a new Competition Task
 *
 * @param: tasknum: Number of the task that should be loaded
 *
 */
bool mi_set_new_task(uint8_t tasknum) {

	if(tasknum == state.curtask) {
		return false;
	}

	//Assign the current Task-Number
	state.curtask = tasknum;

	//Inform QGround Control that a new mission is started
	smq_send_log_info("New Mission started! (JW)");


	//Switch between the task-numbers
	//NOTE: There is another switch on this task-number in the handler-function
	switch(tasknum) {
		case 0: { //Do no changes <=> reset the counter and the waypoint-queue to zero
			mi_init();

			nav_queue_init();

			countdown_running = false;
			countdown = false;

			break;
		}
		case 1: { //Race-Task with triangular course

			//The course is: Start -> Q1 -> Q4 -> Finish

			NEDpoint start;
			NEDpoint end;

			start.northx = (O4.northx+O3.northx)/2;
			start.easty = (O4.easty+O3.easty)/2;

			end.northx = O3.northx + config.dist/4;
			end.easty = O3.easty;

			//Rotate around Point O1
			O3 = nh_rotate(O3, O1, config.rotation);
			O4 = nh_rotate(O4, O1, config.rotation);

			start = nh_rotate(start,O1,config.rotation);
			end = nh_rotate(end,O1,config.rotation);


			//Set the configuration
			nav_set_target_ned(start);
			nav_set_target_ned(O1);
			nav_set_target_ned(O4);
			nav_set_target_ned(end);



			//Set the obstacles
			NEDpoint obst1;
			NEDpoint obst2;

			obst1.northx = O1.northx - config.dist/2;
			obst1.easty = O1.easty + config.dist/2;
			obst1 = nh_rotate(obst1,O1,config.rotation);

			obst2.northx = O1.northx - config.dist;
			obst2.easty = O1.easty;
			obst2 = nh_rotate(obst2,O1,config.rotation);

			nav_set_obstacle_ned(0,obst1);
			nav_set_obstacle_ned(1,obst2);

			break;
		}
		case 2: { //Upwind course with obstacle in the middle

			//Course: O3 -> O1
			//Obstacle in Middle between O3 and O1

			//Config Obstacle
			NEDpoint obst1;
			obst1.northx = O1.northx - config.dist / 2;
			obst1.easty = O1.easty;

			//Rotate the Configuraiont
			O3 = nh_rotate(O3, O1, config.rotation);
			obst1 = nh_rotate(obst1, O1, config.rotation);

			nav_set_target_ned(O3);
			nav_set_target_ned(O1);
			nav_set_obstacle_ned(0, obst1);


			break;
		}
		case 3: { //Downwind course with obstacle in the middle

			//Course: O3 -> O1
			//Obstacle in the Middle between O3 and O1

			//Config Obstacle
			NEDpoint obst1;
			obst1.northx = O1.northx - config.dist / 2;
			obst1.easty = O1.easty;

			//Rotate the Configuraiont
			O3 = nh_rotate(O3, O1, config.rotation);
			obst1 = nh_rotate(obst1, O1, config.rotation);

			nav_set_target_ned(O1);
			nav_set_target_ned(O3);
			nav_set_obstacle_ned(0, obst1);



			break;
		}
		case 4: { //Beam-reach course with obstacle in the middle

			//Course: O1 -> O2
			//Obstacle in the Middle between O1 and O2

			//Config Obstacle
			NEDpoint obst1;
			obst1.northx = O1.northx;
			obst1.easty = O1.easty + config.dist/2;

			//Rotate the Configuraiont
			O2 = nh_rotate(O2, O1, config.rotation);
			obst1 = nh_rotate(obst1, O1, config.rotation);

			nav_set_target_ned(O1);
			nav_set_target_ned(O2);
			nav_set_obstacle_ned(0, obst1);


			break;
		}

		case 5: { //Competition race course without any obstacles

			//The course is: Start -> Q1 -> Q4 -> Finish

			//First we define a countdown. During the time of the countdown we sail between two buoys behind the start-line
			if(countdown == false) {
				countdown = true;
				countdown_ms = 60e6; //Set the countdown to 1min

				smq_send_log_info("Started Countdown");

				NEDpoint start1;
				NEDpoint start2;

				start1.northx = O3.northx - 6.0f;
				start1.easty = O3.easty + 6.0f;
				start2.northx = O4.northx - 6.0f;
				start2.easty = O4.easty - 6.0f;

				start1 = nh_rotate(start1, O1, config.rotation);
				start2 = nh_rotate(start2, O1, config.rotation);

				nav_set_target_ned(start1);
				nav_set_target_ned(start2);
			}


			//The countdown is over => we can start
			if(countdown_ms == 0) {
				NEDpoint end;

				end.northx = O3.northx + config.dist/4;
				end.easty = O3.easty-3;

				//Rotate around Point O1
				O3 = nh_rotate(O3, O1, config.rotation);
				O4 = nh_rotate(O4, O1, config.rotation);
				end = nh_rotate(end,O1,config.rotation);

				//Set the configuration
				nav_set_target_ned(O1);
				nav_set_target_ned(O4);
				nav_set_target_ned(end);


			}


			break;
		}

		case 6: {	//This is the COMPETITION STATION KEEPING CONTEST

			//countdown_ms = 300e6;	//The countdown for this mission is 5minutes
			//countdown = true; 		//Activate the countdown

			NEDpoint wp1, wp2, wp3; //Predefine Waypoints

			//O1--------------------O2
			//      wp1-->----wp2   |		wp4 (this is the final waypoint)
			//                 |    |
			//                 v    |
			//                 |    |
			//                wp3   |


			mi_init();


			wp1.easty = O1.easty + 10;
			wp1.northx = O1.northx - 10;

			wp2.easty = O2.easty - 10;
			wp2.northx = O2.northx - 10;

			wp3.easty = O2.easty - 10;
			wp3.northx = O2.northx - 20;



			//wp1 = nh_rotate(wp1, middle, config.rotation);
			//wp2 = nh_rotate(wp2, middle, config.rotation);
			//wp3 = nh_rotate(wp3, middle, config.rotation);

			nav_set_target_ned(wp1);
			nav_set_target_ned(wp2);
			nav_set_target_ned(wp3);

			NEDpoint obst;
			obst.northx = 0;
			obst.easty = 0;
			nav_set_obstacle_ned(0, obst);


			//We set the length of the countdown for this mission. Originally, 5min are needed, but we assume that
			//we need some time to leave the square. So, do a lucky guess here :)
			countdown_ms_top = 278e6;

			break;
		}

		case 7: {	//THIS IS THE AREA SCANNING CONTEST

			//Load a waypoint for every box
			NEDpoint wp;

			wp.northx=-2.25f;
			wp.easty=-2.25f;
			nav_set_target_ned(wp);

			wp.northx=-6.75f;
			wp.easty=-2.25f;
			nav_set_target_ned(wp);

			wp.northx=-11.25f;
			wp.easty=-2.25f;
			nav_set_target_ned(wp);

			wp.northx=-15.75f;
			wp.easty=-2.25f;
			nav_set_target_ned(wp);

			wp.northx=-20.25f;
			wp.easty=-2.25f;
			nav_set_target_ned(wp);

			wp.northx=-24.75f;
			wp.easty=-2.25f;
			nav_set_target_ned(wp);

			wp.northx=-29.25f;
			wp.easty=-2.25f;
			nav_set_target_ned(wp);

			wp.northx=-33.75f;
			wp.easty=-2.25f;
			nav_set_target_ned(wp);

			wp.northx=-38.25f;
			wp.easty=-2.25f;
			nav_set_target_ned(wp);

			wp.northx=-42.75f;
			wp.easty=-2.25f;
			nav_set_target_ned(wp);

			wp.northx=-42.75f;
			wp.easty=-6.75f;
			nav_set_target_ned(wp);

			wp.northx=-38.25f;
			wp.easty=-6.75f;
			nav_set_target_ned(wp);

			wp.northx=-33.75f;
			wp.easty=-6.75f;
			nav_set_target_ned(wp);

			wp.northx=-29.25f;
			wp.easty=-6.75f;
			nav_set_target_ned(wp);

			wp.northx=-24.75f;
			wp.easty=-6.75f;
			nav_set_target_ned(wp);

			wp.northx=-20.25f;
			wp.easty=-6.75f;
			nav_set_target_ned(wp);

			wp.northx=-15.75f;
			wp.easty=-6.75f;
			nav_set_target_ned(wp);

			wp.northx=-11.25f;
			wp.easty=-6.75f;
			nav_set_target_ned(wp);

			wp.northx=-6.75f;
			wp.easty=-6.75f;
			nav_set_target_ned(wp);

			wp.northx=-2.25f;
			wp.easty=-6.75f;
			nav_set_target_ned(wp);

			wp.northx=-2.25f;
			wp.easty=-11.25f;
			nav_set_target_ned(wp);

			wp.northx=-6.75f;
			wp.easty=-11.25f;
			nav_set_target_ned(wp);

			wp.northx=-11.25f;
			wp.easty=-11.25f;
			nav_set_target_ned(wp);

			wp.northx=-15.75f;
			wp.easty=-11.25f;
			nav_set_target_ned(wp);

			wp.northx=-20.25f;
			wp.easty=-11.25f;
			nav_set_target_ned(wp);

			wp.northx=-24.75f;
			wp.easty=-11.25f;
			nav_set_target_ned(wp);

			wp.northx=-29.25f;
			wp.easty=-11.25f;
			nav_set_target_ned(wp);

			wp.northx=-33.75f;
			wp.easty=-11.25f;
			nav_set_target_ned(wp);

			wp.northx=-38.25f;
			wp.easty=-11.25f;
			nav_set_target_ned(wp);

			wp.northx=-42.75f;
			wp.easty=-11.25f;
			nav_set_target_ned(wp);

			wp.northx=-42.75f;
			wp.easty=-15.75f;
			nav_set_target_ned(wp);

			wp.northx=-38.25f;
			wp.easty=-15.75f;
			nav_set_target_ned(wp);

			wp.northx=-33.75f;
			wp.easty=-15.75f;
			nav_set_target_ned(wp);

			wp.northx=-29.25f;
			wp.easty=-15.75f;
			nav_set_target_ned(wp);

			wp.northx=-24.75f;
			wp.easty=-15.75f;
			nav_set_target_ned(wp);

			wp.northx=-20.25f;
			wp.easty=-15.75f;
			nav_set_target_ned(wp);

			wp.northx=-15.75f;
			wp.easty=-15.75f;
			nav_set_target_ned(wp);

			wp.northx=-11.25f;
			wp.easty=-15.75f;
			nav_set_target_ned(wp);

			wp.northx=-6.75f;
			wp.easty=-15.75f;
			nav_set_target_ned(wp);

			wp.northx=-2.25f;
			wp.easty=-15.75f;
			nav_set_target_ned(wp);

			wp.northx=-2.25f;
			wp.easty=-20.25f;
			nav_set_target_ned(wp);

			wp.northx=-6.75f;
			wp.easty=-20.25f;
			nav_set_target_ned(wp);

			wp.northx=-11.25f;
			wp.easty=-20.25f;
			nav_set_target_ned(wp);

			wp.northx=-15.75f;
			wp.easty=-20.25f;
			nav_set_target_ned(wp);

			wp.northx=-20.25f;
			wp.easty=-20.25f;
			nav_set_target_ned(wp);

			wp.northx=-24.75f;
			wp.easty=-20.25f;
			nav_set_target_ned(wp);

			wp.northx=-29.25f;
			wp.easty=-20.25f;
			nav_set_target_ned(wp);

			wp.northx=-33.75f;
			wp.easty=-20.25f;
			nav_set_target_ned(wp);

			wp.northx=-38.25f;
			wp.easty=-20.25f;
			nav_set_target_ned(wp);

			wp.northx=-42.75f;
			wp.easty=-20.25f;
			nav_set_target_ned(wp);

			wp.northx=-42.75f;
			wp.easty=-24.75f;
			nav_set_target_ned(wp);

			wp.northx=-38.25f;
			wp.easty=-24.75f;
			nav_set_target_ned(wp);

			wp.northx=-33.75f;
			wp.easty=-24.75f;
			nav_set_target_ned(wp);

			wp.northx=-29.25f;
			wp.easty=-24.75f;
			nav_set_target_ned(wp);

			wp.northx=-24.75f;
			wp.easty=-24.75f;
			nav_set_target_ned(wp);

			wp.northx=-20.25f;
			wp.easty=-24.75f;
			nav_set_target_ned(wp);

			wp.northx=-15.75f;
			wp.easty=-24.75f;
			nav_set_target_ned(wp);

			wp.northx=-11.25f;
			wp.easty=-24.75f;
			nav_set_target_ned(wp);

			wp.northx=-6.75f;
			wp.easty=-24.75f;
			nav_set_target_ned(wp);

			wp.northx=-2.25f;
			wp.easty=-24.75f;
			nav_set_target_ned(wp);

			wp.northx=-2.25f;
			wp.easty=-29.25f;
			nav_set_target_ned(wp);

			wp.northx=-6.75f;
			wp.easty=-29.25f;
			nav_set_target_ned(wp);

			wp.northx=-11.25f;
			wp.easty=-29.25f;
			nav_set_target_ned(wp);

			wp.northx=-15.75f;
			wp.easty=-29.25f;
			nav_set_target_ned(wp);

			wp.northx=-20.25f;
			wp.easty=-29.25f;
			nav_set_target_ned(wp);

			wp.northx=-24.75f;
			wp.easty=-29.25f;
			nav_set_target_ned(wp);

			wp.northx=-29.25f;
			wp.easty=-29.25f;
			nav_set_target_ned(wp);

			wp.northx=-33.75f;
			wp.easty=-29.25f;
			nav_set_target_ned(wp);

			wp.northx=-38.25f;
			wp.easty=-29.25f;
			nav_set_target_ned(wp);

			wp.northx=-42.75f;
			wp.easty=-29.25f;
			nav_set_target_ned(wp);

			wp.northx=-42.75f;
			wp.easty=-33.75f;
			nav_set_target_ned(wp);

			wp.northx=-38.25f;
			wp.easty=-33.75f;
			nav_set_target_ned(wp);

			wp.northx=-33.75f;
			wp.easty=-33.75f;
			nav_set_target_ned(wp);

			wp.northx=-29.25f;
			wp.easty=-33.75f;
			nav_set_target_ned(wp);

			wp.northx=-24.75f;
			wp.easty=-33.75f;
			nav_set_target_ned(wp);

			wp.northx=-20.25f;
			wp.easty=-33.75f;
			nav_set_target_ned(wp);

			wp.northx=-15.75f;
			wp.easty=-33.75f;
			nav_set_target_ned(wp);

			wp.northx=-11.25f;
			wp.easty=-33.75f;
			nav_set_target_ned(wp);

			wp.northx=-6.75f;
			wp.easty=-33.75f;
			nav_set_target_ned(wp);

			wp.northx=-2.25f;
			wp.easty=-33.75f;
			nav_set_target_ned(wp);

			wp.northx=-2.25f;
			wp.easty=-38.25f;
			nav_set_target_ned(wp);

			wp.northx=-6.75f;
			wp.easty=-38.25f;
			nav_set_target_ned(wp);

			wp.northx=-11.25f;
			wp.easty=-38.25f;
			nav_set_target_ned(wp);

			wp.northx=-15.75f;
			wp.easty=-38.25f;
			nav_set_target_ned(wp);

			wp.northx=-20.25f;
			wp.easty=-38.25f;
			nav_set_target_ned(wp);

			wp.northx=-24.75f;
			wp.easty=-38.25f;
			nav_set_target_ned(wp);

			wp.northx=-29.25f;
			wp.easty=-38.25f;
			nav_set_target_ned(wp);

			wp.northx=-33.75f;
			wp.easty=-38.25f;
			nav_set_target_ned(wp);

			wp.northx=-38.25f;
			wp.easty=-38.25f;
			nav_set_target_ned(wp);

			wp.northx=-42.75f;
			wp.easty=-38.25f;
			nav_set_target_ned(wp);

			wp.northx=-42.75f;
			wp.easty=-42.75f;
			nav_set_target_ned(wp);

			wp.northx=-38.25f;
			wp.easty=-42.75f;
			nav_set_target_ned(wp);

			wp.northx=-33.75f;
			wp.easty=-42.75f;
			nav_set_target_ned(wp);

			wp.northx=-29.25f;
			wp.easty=-42.75f;
			nav_set_target_ned(wp);

			wp.northx=-24.75f;
			wp.easty=-42.75f;
			nav_set_target_ned(wp);

			wp.northx=-20.25f;
			wp.easty=-42.75f;
			nav_set_target_ned(wp);

			wp.northx=-15.75f;
			wp.easty=-42.75f;
			nav_set_target_ned(wp);

			wp.northx=-11.25f;
			wp.easty=-42.75f;
			nav_set_target_ned(wp);

			wp.northx=-6.75f;
			wp.easty=-42.75f;
			nav_set_target_ned(wp);

			wp.northx=-2.25f;
			wp.easty=-42.75f;
			nav_set_target_ned(wp);

			wp.northx = 0;
			wp.easty = 0;
			nav_set_target_ned(wp);

			break;
		}

		default: {
			//Nothing we can do about this
			break;
		}
	}


	//Everything is OK => return true
	return true;
}


/*
 * Periodically called function by the main-program loop
 *
 */
bool mi_handler(void) {

	uint64_t curtime = hrt_absolute_time();

	if(stationkeeping_isinside == true && countdown_running == false) {
		countdown = true;
		countdown_running = true;
		countdown_ms = 0; //initialize the countdown

		lastcall = curtime;
	}

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

