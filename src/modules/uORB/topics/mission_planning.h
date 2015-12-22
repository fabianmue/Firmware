/*
 * mission_planning.h
 *
 *  Created on: 04.12.2015
 *      Author: Fabian
 */

#ifndef MISSION_PLANNING_H_
#define MISSION_PLANNING_H_

#include <stdint.h>
#include "../uORB.h"


struct mission_planning_s {

	uint64_t timestamp;   	// time of the last mission_planning update since system start in microseconds
	float tar_lat;			// latitude of current target
	float tar_lon;			// longitude of current target

	/*
    float tar_ned_north;   	// NED-north of current target
    float tar_ned_east;    	// NED-east of current target
	*/

    int ob_num;
};

/* register this as object request broker structure */
ORB_DECLARE(mission_planning);

#endif /* MISSION_PLANNING_H_ */
