/*
 * pp_navigator.c
 *
 * This file contains a navigator. The navigator calculates a new heading reference and gives orders to the helsman.
 * In our case the helsman is the "autonomous_sailing module". The helsman polls the uORB topic "path_planning" for
 * changes and adjustes its control according to the orders.
 *
 *  Created on: 04.03.2015
 *      Author: Jonas Wirz <wirzjo@student.ethz.ch>
 */

#include "pp_navigator.h"
#include "pp_config.h"


/**Struct containing the status of the Navigator */
struct {
	Point position; 				//Last known Position (lat/lon) [rad]
	float heading_cur;				//Current Heading of the boat [rad]
	float heading_ref;				//Heading Reference for optimal path [rad]
	bool tack;						//true, iff Tack is in progress
	bool gybe;						//true, iff Gybe is in progress
} nav_state = {
		.heading_cur = 0,
		.heading_ref = 0,
		.tack = false,
		.gybe = false,
};




/**
 * Calculate a new optimal reference Heading
 */
void nav_navigate(void) {

	/** A new reference heading should only be calculated if the boat is not doing a maneuver */
	if(!nav_state.tack && !nav_state.gybe) {

		//TODO: Calculate the best Reference Heading here



		//A new Reference Heading is generated => send this data to the Helsman (autonomous_sailing module)
	}

} //end of nav_navigate



/**
 * The "autonomous_sailing module" has changed the topic "path_planning", because a maneuver was finished.
 * Therefore, the Helsman speaks to the Navigator => the Navigator has to listen.
 *
 * @param *strs_p: Pointer to the topics-struct
 */
void nav_listen2helsman(const struct structs_topics_s *strs_p) {

	//Get the state of the tack
	nav_state.tack = strs_p->path_planning.tack;

	//Get the state of the gybe
	nav_state.gybe = strs_p->path_planning.gybe;

} //end of nav_listen2helsman



/**
 * A new heading reference is available. Communicate this new information to the "autonomous_sailing module".
 * Therefore, speak to the Helsman.
 */
void nav_speak2helsman(void) {

	//Communicate the new data to the Helsman
	th_update_pathplanning(nav_state.heading_ref, nav_state.tack, nav_state.gybe);
}



/**
 * New information about the heading is available. Therefore, the state of the navigator needs to be updated
 *
 * @param *strs_p: Pointer to the topics-struct
 */
void nav_heading_update(const struct structs_topics_s *strs_p) {

	//TODO get the Heading Update and store it in the State-Struct

} //end of nav_heading_update



/**
 * New Information about the position is available. Therefore, the state-struct must be updated.
 *
 * @param *strs_p: Pointer to the topics-struct
 */
void nav_position_update(const struct structs_topics_s *strs_p) {

	nav_state.position.lat = strs_p->vehicle_global_position.lat * DEG2RAD;
	nav_state.position.lon = strs_p->vehicle_global_position.lon * DEG2RAD;

} //end of nav_heading_update




