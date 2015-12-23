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

	int mi_id;				// mission id

	int sd_read;

	float tar_lat;
	float tar_lon;
	int tar_num;
	float obs_lat;
	float obs_lon;
	int obs_num;

};

/* register this as object request broker structure */
ORB_DECLARE(mission_planning);

#endif /* MISSION_PLANNING_H_ */
