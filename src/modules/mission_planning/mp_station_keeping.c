/*
 * mp_station_keeping.c
 *
 *  Created on: 09.10.2015
 *      Author: Fabian
 */

#include <stdbool.h>
#include <stdlib.h>
#include <math.h>
#include "mp_mission.h"

/***********************************************************************************/
/*****  V A R I A B L E S  *********************************************************/
/***********************************************************************************/



/***********************************************************************************/
/*****  F U N C T I O N   D E C L A R A T I O N S  *********************************/
/***********************************************************************************/

bool mp_sk_start(mission new_mission);

bool mp_sk_isinside(NEDpoint boat_pos, frame comp_frame);

float mp_sk_perpdist(NEDpoint P1, NEDpoint P2, NEDpoint pos);

/***********************************************************************************/
/*****  P U B L I C    F U N C T I O N S  ******************************************/
/***********************************************************************************/

bool mp_sk_start(mission new_mission) {

	printf("station-keeping started");

	/*
	 * good strategy: move along a certain border of the competition frame, such that after the countdown has reached zero,
	 * 				  we can sail outside the frame as fast as possible
	 * 				  -> choose border such that the wind to the outside of the frame allows the fastest sailing as possible
	 *
	 * 		O1--------------------O2
	 *      |                      |
	 *      |                      |
	 *      |                      |
	 *      |                      |
	 *      |                      |
	 *      |                      |
	 *      |                      |
	 *      O3--------------------O4
	 *
	 */

	/* TODO: - implement countdown
	 * 		 - implement strategy
	 */

	return true;

}

bool mp_sk_isinside(NEDpoint boat_pos, frame comp_frame) {

	// calculate perpendicular distances from boat_pos to all borders (https://en.wikipedia.org/wiki/Distance_from_a_point_to_a_line)
	float dist_O1O2 = mp_sk_perpdist(comp_frame.O1, comp_frame.O2, boat_pos);
	float dist_O2O3 = mp_sk_perpdist(comp_frame.O2, comp_frame.O3, boat_pos);
	float dist_O3O4 = mp_sk_perpdist(comp_frame.O3, comp_frame.O4, boat_pos);
	float dist_O4O1 = mp_sk_perpdist(comp_frame.O4, comp_frame.O1, boat_pos);

	if (dist_O1O2 < comp_frame.dist & dist_O2O3 < comp_frame.dist & dist_O3O4 < comp_frame.dist & dist_O4O1 < comp_frame.dist) {

		return true;

	}
	else {

		return false;

	}

}

float mp_sk_perpdist(NEDpoint P1, NEDpoint P2, NEDpoint pos) {

	// calculate perpendicular distances from boat_pos to all borders (https://en.wikipedia.org/wiki/Distance_from_a_point_to_a_line)
	float dist = abs((P2.easty-P1.easty)*pos.northx-(P2.northx-P1.northx)*pos.easty \
			+P2.northx*P1.easty-P2.easty*P1.northx)/sqrt(pow(P2.easty-P1.easty,2)+pow(P2.northx-P1.northx,2));
	return dist;

}
