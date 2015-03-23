/*
 * config.h
 *
 *	This file contains global definitions like constants, that are used in more than one c-File.
 *
 *  Created on: 04.03.2015
 *      Author: Jonas Wirz <wirzjo@student.ethz.ch>
 */

#ifndef CONFIG_H_
#define CONFIG_H_

#include <stdint.h>


/*DEBUG ON THE COMPUTERgit
 * If this flag is set to 1, the pathplanning module can be debugged using
 * an external C-Program simulating the parameter updates. This way it
 * is possible checking the algorithms and the inputs/outputs of the
 * different functions.
 * Note: With C_DEBUG = 1 it is NOT possible to compile for the Pixhawk!*/
#define C_DEBUG 0

#if C_DEBUG == 1 //Include some h-files for Computer Simulation (PC-Debugging)
	#include "PCDebug/pixhawk_variables.h"
#endif


/* PIXHAWK DEBUG FLAG
 * The development on the Pixhawk is on, if set to 1. It displays additional messages in QGround-Control then.
 */
#define P_DEBUG 1


//1 if you're testing path_planning app indoor, PLEASE SET INDOOR PARSER_200WX
#define SIMULATION_FLAG 1

//set if you want to use a path planning that uses grid lines
#define USE_GRID_LINES 1 ///1 if you want to use grid line system to tell the boat where to tack

/*DEBUG FLAG
 * If this flag is enabled (set to 1) the module is in Debug-Mode
 * and outputs comments onto the console.*/
#define DEBUG 1


/*POLLING TIMEOUT
 * Timeout, if no change in the polled topics is detected
 * Value is in MilliSeconds */
#define TIMEOUT_POLL 1000


/* CONVERT FROM DEGREES TO RADIANS
 * pi/180 <=> conversion from Degrees to Radians */
#define DEG2RAD      0.0174532925199433f 	//pi/180
#define RAD2DEG     57.2957795131f			//180/pi


/* DEFINITION FOR PI
 * pi = 3.14159... */
#define PI           3.14159265358979323846f //pi


/* DEFINITION FOR PI/2
 * pi/2 = 1.57079...*/
#define PIHALF	     1.57079632679f 		//pi/2


/* DEFINE MAXIMUM NUMBER OF TARGETS
 * Maximum number of targets (waypoints) */
#define MAXTARGETNUMBER 5


/* DEFINE MAXIMUM NUMBER OF OBSTACLES
 * Maximum number of obstacles */
#define MAXOBSTACLENUMBER 3


/* INITIAL TARGET
 * Position of the initial Target position [° E7-Format]*/
//#define HOMELAT 47.34879556f			//Note: This is a point outside of the testarea near Tiefenbrunnen ZH
//#define HOMELON 8.56092835f
//#define HOMEALT	405*1000.0f				//Note: The altitude value is in Millimeters

#define HOMELAT 4737857				//Note: This is the lower terrass at ETH Zurich (Physikstrasse)
#define HOMELON 8552777
#define HOMEALT	484.276f*1000.0f	//Note: The altitude value is in Millimeters



/* TARGET TOLERANCE
 * Radius of a circle around the target, inside which the target is counted as reached. This radius depends
 * on the GPS Accuracy. [m]*/
#define TARGETTOLERANCE 3


/* ALGORITHM FLAG
 * Chooses a path-planning algorithm
 * 1 = Cost-Function-Method
 * 2 = Potential-Field-Method
 *
 * DEFAULT = 1 = Cost-Function-Method */
extern uint8_t pp_algorithm;

//remote control usefull value
#define RC_MODE_INDEX 4             ///index of the type of mode in rc_channels struct
#define RC_MANUAL_MODE -1.0f        ///Rc_Ch4 == RC_MANUAL_MODE if manual mode selected


#endif /* CONFIG_H_ */
