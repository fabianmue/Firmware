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


/*DEBUG ON THE COMPUTER
 * If this flag is set to 1, the pathplanning module can be debugged using
 * an external C-Program simulating the parameter updates. This way it
 * is possible checking the algorithms and the inputs/outputs of the
 * different functions.
 * Note: With C_DEBUG = 1 it is NOT possible to compile for the Pixhawk! */
#define C_DEBUG 0

#if C_DEBUG == 1 //Include some h-files for Computer Simulation (PC-Debugging)
	#include "PCDebug/pixhawk_variables.h"
#endif


/* PIXHAWK DEBUG FLAG
 * The development on the Pixhawk is on, if set to 1. It displays additional messages in QGround-Control then. */
#define P_DEBUG 0


//1 if you're testing path_planning app indoor, PLEASE SET INDOOR PARSER_200WX
#define SIMULATION_FLAG 0

//set if you want to use a path planning that uses grid lines
#define USE_GRID_LINES 0 ///1 if you want to use grid line system to tell the boat where to tack

/*DEBUG FLAG
 * If this flag is enabled (set to 1) the module is in Debug-Mode
 * and outputs comments onto the console.*/
#define DEBUG 0


/*LAKE DEBUG
 * The following variables are introduced in order to quickly switch between different Versions while on the lake
 *
 */
#define LDEBUG_POLARDIAGRAM 0 //Uses an easy Polardiagram that only contains the Upwind No-Go-Zone and has the same velocity everywhere else
#define LDEBUG_SENSORBOARD  1 //Deactivate the Communication with the Sensorbaord (1 <=> deactivate communication)
#define LDEBUG_STATICDATA   1 //Do not request new data from the sensorboard, but use predefined data
#define LDEBUG_KALMANTRACKER 0 //Activate the Kalman tracker (1 = tracker is activated)
#define LDEBUG_KALMANTRACKER_CMS 0 //Activate printf-messages onto console (1 = messages are activated)
#define LDEBUG_USEMISSION   1 //Use the Mission planner instead of manual input of target and obstacles (1 = enable/0 = disable)
#define LDEBUG_MISSIONHANDLER 1 //Use the Mission -Handler

#define LDEBUG_FAKEPOSITION 0 //Fake a NED position by QGround Control



/*FAILSAFE MODE
 * If this flag is enabled (set to 1) a failsafe mode is used. A circle is defined in which the boat
 * may have its position.
 * As soon as it leaves this circle the failsafe mode is activated and the boat should return to its
 * home position automatically. */
#define USE_FAILSAFE 0 //1, if failsafe mode is enabled


/*POLLING TIMEOUT
 * Timeout, if no change in the polled topics is detected
 * Value is in MilliSeconds */
#define TIMEOUT_POLL 1000


/*SENSOR CONFIGURATION
 * Configuration for the Sensor. Values like maximum measurement distance and stepsize are
 * defined here. */
#define SENSOR_STEPSIZE 2 //Stepsize the Sensor uses for making Distance Measurements [°]
#define SENSOR_MAXDIST 40 //Maximum measurement distance [m]
#define SENSOR_RANGE 90   //Range of the Sensor [°] (Note: to the left and to the right)


/* CONVERT FROM DEGREES TO RADIANS
 * pi/180 <=> conversion from Degrees to Radians */
#define DEG2RAD      0.0174532925199433f 	//pi/180
#define RAD2DEG      57.2957795131f			//180/pi


/* DEFINITION FOR PI
 * pi = 3.14159... */
#define PI           3.14159265358979323846f //pi


/* DEFINITION FOR PI/2
 * pi/2 = 1.57079...*/
#define PIHALF	     1.57079632679f 		//pi/2


/* DEFINE MAXIMUM NUMBER OF TARGETS
 * Maximum number of targets (waypoints) */
#define MAXTARGETNUMBER 110


/* DEFINE MAXIMUM NUMBER OF OBSTACLES
 * Maximum number of obstacles */
#define MAXOBSTACLENUMBER 9


/* INITIAL TARGET
 * Position of the initial Target position [° E7-Format]*/
//#define HOMELAT 473487956f			//Note: This is a point outside of the testarea near Tiefenbrunnen ZH
//#define HOMELON 85609284f
//#define HOMEALT	405*1000.0f				//Note: The altitude value is in Millimeters

//#define HOMELAT 473785680				//Note: This is the lower terrass at ETH Zurich (Physikstrasse)
//#define HOMELON 85528100
//#define HOMEALT	484.276f*1000.0f	//Note: The altitude value is in Millimeters

#define HOMELAT 601050000
#define HOMELON 199500000
#define HOMEALT 0



/* TARGET TOLERANCE
 * Radius of a circle around the target, inside which the target is counted as reached. This radius depends
 * on the GPS Accuracy. [m]*/
#define TARGETTOLERANCE 2


/* DOWNWIND COURSE
 * alpha-angle for which the boat is considered to sail on a downwind course [rad] */
#define DOWNWIND_COURSE 120*DEG2RAD


/* ALGORITHM FLAG
 * Chooses a path-planning algorithm
 * 1 = Cost-Function-Method
 * 2 = Potential-Field-Method
 *
 * DEFAULT = 1 = Cost-Function-Method */
extern uint8_t pp_algorithm;

//remote control useful value
#define RC_MODE_INDEX 4             ///index of the type of mode in rc_channels struct
#define RC_MANUAL_MODE -1.0f        ///Rc_Ch4 == RC_MANUAL_MODE if manual mode selected





#endif /* CONFIG_H_ */
