/*
 * path_planning.h
 *
 * Interface between the Modules "autonomous_sailing" and "path_planning"
 *
 *  Created on: 04.03.2015
 *      Author: Jonas Wirz <wirzjo@student.ethz.ch>
 */

#ifndef ESSC_LOG_H_
#define ESSC_LOG_H_

#include <stdint.h>
#include <stdbool.h>
#include "../uORB.h"


struct essc_log_s {
	float k; 				/**< Stepsize between to consecutive sail-angle changes [rad] */
    uint16_t windowsize;    /**< Size of the window for averaging the speed */
    float period; 			/**< Time between two consecutive changes in sail-angle [us] */
};


/* register this as object request broker structure */
ORB_DECLARE(essc_log);

#endif /* PATH_PLANNING_H_ */
