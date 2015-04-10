/*
 * config.h
 *
 *  Created on: 10.04.2015
 *      Author: Jonas Wirz <wirzjo@student.ethz.ch>
 */

#ifndef CONFIG_H_
#define CONFIG_H_

/*POLLING TIMEOUT
 * Timeout, if no change in the polled topics is detected
 * Value is in MilliSeconds */
#define TIMEOUT_POLL 1000


/* CONVERT FROM DEGREES TO RADIANS
 * pi/180 <=> conversion from Degrees to Radians */
#define DEG2RAD      0.0174532925199433f 	//pi/180
#define RAD2DEG     57.2957795131f			//180/pi


#endif /* CONFIG_H_ */
