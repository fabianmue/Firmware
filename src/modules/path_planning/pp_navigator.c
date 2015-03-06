/*
 * pp_navigator.c
 *
 * This file contains a navigator. The navigator calculates a new heading reference and gives orders to the helsman.
 * In our case the helsman is the "autonomous_sailing module". The helsman polls the uORB topic "path_planning" for
 * changes and adjustes its control according to the orders.
 *
 * Note: There are several Coordinate Systems:
 * 		 - Sensor Frame:  [-pi...0...pi] = [South..West..North..East..South]
 * 		 - Dumas' Frame:  [pi...0...-pi] = [South..West..North..East..South]
 * 		 - Compass Frame: [0...2pi]      = [North..East..South..West..North]
 *
 *  Created on: 04.03.2015
 *      Author: Jonas Wirz <wirzjo@student.ethz.ch>
 */

/* TODO:
 * - add parameters from QGroundControl
 * - add Potentialfield Method
 * - average WindSpeed and average WindDirection
 */



#include "pp_navigator.h"

#include "pp_cost_method.h"



/* Init the Struct containing the current state of the Navigator */
static struct nav_state_s state;


/**Struct containing the Race-Field-Information */
static struct nav_field_s field;


/** Which algorithm should be used for pathplanning */
uint8_t pp_algorithm = 1;  	//Set Cost-Function-Method as the default algorithm





/**
 * Init a new Navigator by creating all necessary variables.
 *
 */
void nav_init(void) {

	//Set the initial values for the Field
	field.NumberOfTargets = 1;
	field.NumberOfObstacles = 1;

	//Set the initial Target-Position
	field.targets[0].lat = HOMELAT;
	field.targets[0].lat = HOMELON;

	//Set the initial Obstacle-Position
	field.obstacles[0].lat = 0;
	field.obstacles[0].lon = 0;

	//Set the initial values for the State
	state.heading_cur = 0;
	state.heading_ref = 0;
	state.wind_dir = 0;
	state.maneuver = false;
	state.targetNum = 0;
}



/**
 * Calculate a new optimal reference Heading
 */
void nav_navigate(void) {

	/** A new reference heading should only be calculated if the boat is not doing a maneuver */
	if(!state.maneuver) {


		/****FIND A NEW REFERENCE HEADING
		 * Different algorithms can be used. */
		if(pp_algorithm == 1) {
			//Use Cost-Function-Method

			state.heading_ref = cm_NewHeadingReference(&state,&field);
		}

		if(pp_algorithm == 2) {
			//Use Potential-Field-Method

			//TODO add the Potential-Field Reference Heading here
		}


		//****DECISION MAKING
		/* In the following section the decisions based on the optimal Heading are made. In particular
		 * the the Navigator decides if the boat should tack */
		float NewWind = nh_appWindDir(state.heading_ref,state.wind_dir); 		//New Apparent Winddirection
		float OldWind = nh_appWindDir(state.heading_cur,state.wind_dir);		//Current Apparent Winddirection


		/*Decide if we have to do a tack or a gybe
		* A maneuver is necessary, iff we change the hull. A change of hull is represented as a change of sign of the
		* apparent Wind direction.
		*/
		if(!((NewWind < 0 && OldWind < 0) || (NewWind > 0 && OldWind > 0))) {
			//A Maneuver is Necessary

			state.maneuver = true;

		} //if boat should do a maneuver



		/****COMMUNICATION
		* A new Reference Heading is generated => send this data to the Helsman (autonomous_sailing module) */
		nav_speak2helsman();

	} //if no tack or gybe is in progress

} //end of nav_navigate



/**
 * The "autonomous_sailing module" has changed the topic "path_planning", because a maneuver was finished.
 * Therefore, the Helsman speaks to the Navigator => the Navigator has to listen.
 *
 * @param *strs_p: Pointer to the topics-struct
 */
void nav_listen2helsman(const struct structs_topics_s *strs_p) {

	//Get the state of the tack
    //state.tack = strs_p->path_planning.tack;

	//Get the state of the gybe
    //state.gybe = strs_p->path_planning.gybe;

} //end of nav_listen2helsman



/**
 * A new heading reference is available. Communicate this new information to the "autonomous_sailing module".
 * Therefore, speak to the Helsman.
 */
void nav_speak2helsman(void) {

	//Communicate the new data to the Helsman
	//TODO: The heading_reference must be converted to an alpha_star in Dumas' Frame!
    //th_update_pathplanning(state.heading_ref, state.tack, state.gybe);
}



/**
 * New information about the heading is available. Therefore, the state of the navigator needs to be updated
 *
 * @param *strs_p: Pointer to the topics-struct
 */
void nav_heading_wind_update(const struct structs_topics_s *strs_p) {

	/* Get the new alpha Value
	 * alpha = yaw-twd;
	 * alpha is either computed using the yaw-angle or the COG (Course over Ground) */
	float alpha = strs_p->boat_guidance_debug.alpha;
	float twd = strs_p->boat_guidance_debug.twd_mean;

	/* For the Pathplanning a compass-heading is needed (element of [0...2pi], with 0 = true North) */
	state.heading_cur = nh_dumas2compass(alpha + twd); //COG in Dumas' convention

} //end of nav_heading_update



/**
 * New Information about the position is available. Therefore, the state-struct must be updated.
 *
 * @param *strs_p: Pointer to the topics-struct
 */
void nav_position_update(const struct structs_topics_s *strs_p) {

	Point newPos;
	newPos.lat = ((float)(strs_p->vehicle_global_position.lat)) * DEG2RAD;
	newPos.lon = ((float)(strs_p->vehicle_global_position.lon)) * DEG2RAD;

	//Check, if we reached a target
	if(nh_dist(newPos,field.targets[state.targetNum]) <= TARGETTOLERANCE) {
		//We are inside the tolerance => target is counted as reached

		if(state.targetNum != (field.NumberOfTargets-1)) {
			//This is not the last target => set new Target

			state.targetNum += 1;
		} else {
			//This was the last target

			//TODO: Report to QGround Control

		}
	}


	//Update the state to the new Position
	state.position = newPos;

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




