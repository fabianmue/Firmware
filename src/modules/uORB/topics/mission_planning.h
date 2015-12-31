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

	int32_t mi_id;			// mission id
	float wp_lat;
	float wp_lon;
	int32_t wp_count;
	float ob_lat;
	float ob_lon;
	float ob_rad;
	int32_t ob_count;
};

/* register this as object request broker structure */
ORB_DECLARE(mission_planning);

#endif /* MISSION_PLANNING_H_ */
