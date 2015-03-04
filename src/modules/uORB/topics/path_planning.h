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
	float    heading_ref;  /**< Heading Reference with respect to true North [0...2pi] [rad] */
	bool     tack; 		   /**< True, iff boat should tack (is set by pathplanning module and should
								be reset by Autonomous Sailing module)*/
	bool     gybe;         /**< True, iff boat should gybe (is set by pathplanning module and should
								be reset by Autonomous Sailing module)*/
};


/* register this as object request broker structure */
ORB_DECLARE(path_planning);

#endif /* PATH_PLANNING_H_ */
