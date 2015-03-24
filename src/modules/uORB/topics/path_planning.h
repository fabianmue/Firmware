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
	uint64_t timestamp;   /**< Time of the last Pathplanning Update since System Start in Microseconds */
    float    alpha_star;  /**< Alpha angle reference, in Dumas' convention [rad] */
    bool     do_maneuver; /**< True, iff boat should either tack or jybe */
    uint8_t  id_maneuver; /**< ID last commanded maneuver */
    uint8_t  id_cmd; /**< ID command for autonomous_sailing app */

    float ned_north;	/**< North-Coordinate in NED-Frame of the Boat => for debugging*/
    float ned_east;		/**< East-Coordinate in NED-Frame of the Boat  => for debugging*/
    float heading; 		/**< Current heading of the boat known by the Navigator => for debugging*/
};


/* register this as object request broker structure */
ORB_DECLARE(path_planning);

#endif /* PATH_PLANNING_H_ */
