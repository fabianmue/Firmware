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



/* Init the Struct containing the current state of the Navigator */
 struct nav_state_s state = {
		.heading_cur = 0,
		.heading_ref = 0,
		.wind_dir = 0,
		.tack = false,
		.gybe = false
};


/**Struct containing the Race-Field-Information */
struct nav_field_s field = {
		.NumberOfTargets = 1,
		.NumberOfObstacles = 1
};


/** Which algorithm should be used for pathplanning */
uint8_t pp_algorithm = 1;  	//Set Cost-Function-Method as the default algorithm





/**
 * Init a new Navigator by creating all necessary variables.
 *
 */
void nav_init(void) {

	//Set the initial Target-Position
	field.targets[0].lat = HOMELAT;
	field.targets[0].lat = HOMELON;

	//Set the initial Obstacle-Position
	field.obstacles[0].lat = 0;
	field.obstacles[0].lon = 0;

}



/**
 * Calculate a new optimal reference Heading
 */
void nav_navigate(void) {

	/** A new reference heading should only be calculated if the boat is not doing a maneuver */
	if(!state.tack && !state.gybe) {


		if(pp_algorithm == 1) {
			//TODO add the Cost-Function Reference Heading here
		}

		if(pp_algorithm == 2) {
			//TODO add the Potential-Field Reference Heading here
		}


		//****DECISION MAKING
		/* In the following section the decisions based on the optimal Heading are made. This means
		 * that a decision is made, iff the boat should tack or gybe. */
		float NewWind = nh_appWindDir(state.heading_ref,state.wind_dir); 		//New Apparent Winddirection
		float OldWind = nh_appWindDir(state.heading_cur,state.wind_dir);		//Current Apparent Winddirection


		/*Decide if we have to do a tack or a gybe
		* A maneuver is necessary, iff we change the hull. A change of hull is represented as a change of sign of the
		* apparent Wind direction.
		*/
		if(!((NewWind < 0 && OldWind < 0) || (NewWind > 0 && OldWind > 0))) {
			//A Maneuver is Necessary

			if(fabsf(NewWind) > PIHALF) {
				//A tack is necessary to reach the new optimal heading

				state.tack = true;
			} else {
				//A gybe is necessary to reach the new optimal heading

				state.gybe = true;
			}

		} //if no tack or gybe in progress



		/****COMMUNICATION
		* A new Reference Heading is generated => send this data to the Helsman (autonomous_sailing module) */
		nav_speak2helsman();
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
	state.tack = strs_p->path_planning.tack;

	//Get the state of the gybe
	state.gybe = strs_p->path_planning.gybe;

} //end of nav_listen2helsman



/**
 * A new heading reference is available. Communicate this new information to the "autonomous_sailing module".
 * Therefore, speak to the Helsman.
 */
void nav_speak2helsman(void) {

	//Communicate the new data to the Helsman
	th_update_pathplanning(state.heading_ref, state.tack, state.gybe);
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

	//TODO: check if we reached the target and possibly set the new target


	state.position.lat = ((float)(strs_p->vehicle_global_position.lat)) * DEG2RAD;
	state.position.lon = ((float)(strs_p->vehicle_global_position.lon)) * DEG2RAD;

} //end of nav_heading_update



/**
 * Set a new Obstacle
 * This functions is called by QGroundControl to set a new Value
 *
 * @param ObstNumber: The position of the obstacle in the Array of all Obstacles
 * @param ObstPos: The GPS-Position of the obstacle represented as a Point
 */
void nav_setObstacle(uint8_t ObstNumber, Point ObstPos) {
	field.obstacles[ObstNumber].lat = ObstPos.lat;
	field.obstacles[ObstNumber].lon = ObstPos.lon;

	field.NumberOfObstacles = ObstNumber;
}


/**
 * Set a new Target
 * This functions is called by QGroundControl to set a new Value
 *
 * @param TargetNumber: The position of the target in the Array of all Targets
 * @param TargetPos: The GPS-Position of the target represented as a Point
 */
void nav_setTarget(uint8_t TargetNumber, Point TargetPos) {
	field.targets[TargetNumber].lat = TargetPos.lat;
	field.targets[TargetNumber].lon = TargetPos.lon;

	field.NumberOfTargets = TargetNumber;
}




