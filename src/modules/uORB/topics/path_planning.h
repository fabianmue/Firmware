/*
 * path_planning.h
 *
 * Interface between the Modules "autonomous_sailing" and "path_planning"
 *
 *  Created on: 04.03.2015
 *      Author: Jonas Wirz <wirzjo@student.ethz.ch>
 */

#ifndef PATH_PLANNING_H_
#define PATH_PLANNING_H_

#include <stdint.h>
#include <stdbool.h>
#include "../uORB.h"


struct path_planning_s {
	uint64_t timestamp;    /**< Time of the last Pathplanning Update since System Start in Microseconds */
    float    alpha_star;  /**< Alpha angle reference, in Dumas' convention [rad] */
    bool     do_maneuver; /**< True, iff boat should either tack or jybe */
    float    x_race_m;    /**< X coordinate in race frame, [m]*/
    float    y_race_m;    /**< Y coordinate in race frame, [m]*/
};


/* register this as object request broker structure */
ORB_DECLARE(path_planning);

#endif /* PATH_PLANNING_H_ */
