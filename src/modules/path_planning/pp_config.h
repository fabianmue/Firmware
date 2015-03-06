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
 * Position of the initial Target position [rad]*/
#define HOMELAT 47.348877f * DEG2RAD	//Note: This is a point outside of the testarea near Tiefenbrunnen ZH
#define HOMELON 8.5604200f * DEG2RAD


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



#endif /* CONFIG_H_ */
