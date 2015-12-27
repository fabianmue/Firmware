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
	int32_t sd_read;		// true, if mission was read from sd
	float tar_lat;
	float tar_lon;
	int32_t tar_num;
	float obs_lat;
	float obs_lon;
	int32_t obs_num;
};

/* register this as object request broker structure */
ORB_DECLARE(mission_planning);

#endif /* MISSION_PLANNING_H_ */
