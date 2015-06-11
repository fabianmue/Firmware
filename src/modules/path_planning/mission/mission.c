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

#include "../pp_navigation_helper.h"
#include "../pp_navigator.h"
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


//static char txt_msg[150]; //Buffer for QGround Control Messages




/***********************************************************************************/
/*****  F U N C T I O N   P R O T O T Y P E S **************************************/
/***********************************************************************************/





/***********************************************************************************/
/*****  P U B L I C    F U N C T I O N S  ******************************************/
/***********************************************************************************/

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
 */
bool mi_set_new_task(uint8_t tasknum) {

	if(tasknum == state.curtask) {
		return false;
	}

	//Assign the current Task-Number
	state.curtask = tasknum;


	//Calcualte the positions of the Buoys
	NEDpoint O1 = config.o1;
	NEDpoint O2;
	NEDpoint O3;
	NEDpoint O4;

	O2.northx = O1.northx;
	O2.easty = O1.easty + config.dist;
	O3.northx = O1.northx - config.dist;
	O3.easty = O1.easty;
	O4.northx = O1.northx - config.dist;
	O4.easty = O1.easty + config.dist;

	//Inform QGround Control that a new mission is started
	smq_send_log_info("New Mission started! (JW)");



	switch(tasknum) {
		case 0: { //Do no changes
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
			nav_set_target_ned(0, start);
			nav_set_target_ned(1, O1);
			nav_set_target_ned(2, O4);
			nav_set_target_ned(3, end);



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

			nav_set_target_ned(0, O3);
			nav_set_target_ned(1, O1);
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

			nav_set_target_ned(0, O1);
			nav_set_target_ned(1, O3);
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

			nav_set_target_ned(0, O1);
			nav_set_target_ned(1, O2);
			nav_set_obstacle_ned(0, obst1);


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






/***********************************************************************************/
/*****  P R I V A T E    F U N C T I O N S  ****************************************/
/***********************************************************************************/

