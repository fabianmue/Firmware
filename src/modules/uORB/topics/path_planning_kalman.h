/*
 * path_planning_kalman.h
 *
 * Topic for logging values of the Kalman Obstacle Tracker
 *
 *  Created on: 04.03.2015
 *      Author: Jonas Wirz <wirzjo@student.ethz.ch>
 */

#ifndef PATH_PLANNING_KALMAN_H_
#define PATH_PLANNING_KALMAN_H_

#include <stdint.h>
#include <stdbool.h>
#include "../uORB.h"


struct path_planning_kalman_s {
	uint64_t timestamp;   /**< Time of the last Pathplanning Update since System Start in Microseconds */

	uint16_t tracknum; 	  /**< Number of tracks currently available */
	uint16_t newtracknum; /**< Number of newly added tracks in the last step */
	uint16_t refoundnum;  /**< Number of refound tracks in the last step */
};


/* register this as object request broker structure */
ORB_DECLARE(path_planning_kalman);

#endif /* PATH_PLANNING_H_ */
