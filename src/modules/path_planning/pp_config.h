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



#endif /* CONFIG_H_ */
