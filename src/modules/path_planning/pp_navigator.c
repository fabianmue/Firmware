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
 * - add Potentialfield Method
 * - Winddirection => how's the definition? Wind from Nort = 0°/ Wind from South = 180° (Sensor-Frame)
 * - Calculate the distance to target and bearing to target only once!!!
 * - Add the center-line for Tactical cost
 */


#include "pp_config.h"
#include "pp_navigator.h"
#include "pp_cost_method.h"

#if C_DEBUG == 0
#include "pp_communication_buffer.h"
#include <drivers/drv_hrt.h>
#endif


/** Struct holding the main configuration variables of the navigator */
static struct {
	uint64_t period; 		//The period of calls to Pathplanning (time between two calls to nav_navigate())
} config = {
	.period = 1000
};


/** Init the Struct containing the current state of the Navigator */
static struct nav_state_s state;


/** Init the Struct containing the Race-Field-Information */
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
	field.targets[0].northx = 0;
	field.targets[0].easty = 0;

	//Set the initial Obstacle-Position
	field.obstacles[0].northx = 0;
	field.obstacles[0].easty = 0;

	//Set the initial Start-Line
	field.startline[0].northx = 0;	//Buoy1
	field.startline[0].easty = 0;
	field.startline[1].northx = 1;  //Buoy2
	field.startline[1].easty = 1;

	//Set the initial values for the State
	state.heading_cur = PI/2;
	state.heading_ref = 0;
	state.wind_dir = 0;
	state.maneuver = false;
	state.targetNum = 0;
	Point home;
	home.lat = HOMELAT;
	home.lon = HOMELON;
	state.position = nh_geo2ned(home);

	state.last_call = 0;
}



/**
 * Calculate a new optimal reference Heading
 */
void nav_navigator(void) {

	/** Check if new Inforamtion is available and update the state accordingly. */
	#if C_DEBUG == 0
	nav_listen2helsman();	//Check, if a maneuver is completed
	nav_heading_update();	//Check for a new Heading (alpha)
	nav_position_update();  //Check for a new Position update
	nav_wind_update();		//Check for new Wind measurements
	#endif



	/** Pathplanning is only done with a certain frequency
	 *  Therefore, check the systemtime.
	 *  Note: When the Computer-Debug-Mode is on Pathplanning is done in every loop!*/
	uint64_t systime = 0;
	#if C_DEBUG == 0
		systime = hrt_absolute_time();
	#endif
	if((systime-state.last_call >= config.period) || (C_DEBUG == 1)) {

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

	} //if do pathplanning with a predefined frequency

} //end of nav_navigate



/**
 * The "autonomous_sailing module" has changed the topic "path_planning", because a maneuver was finished.
 * Therefore, the Helsman speaks to the Navigator => the Navigator has to listen.
 *
 * @param *strs_p: Pointer to the topics-struct
 */
void nav_listen2helsman(void) {

	#if C_DEBUG == 0
	/* Check if Helsman has completed the maneuver */
	if(state.maneuver) {
		if(cb_is_maneuver_completed()) {
			//The ongoing maneuver is completed and therefore the state can be reseted.

			state.maneuver = false;
		}

	}
	#endif




} //end of nav_listen2helsman



/**
 * A new heading reference is available. Communicate this new information to the "autonomous_sailing module".
 * Therefore, speak to the Helsman.
 *
 * TODO: Check if the transformation from true heading to alpha is correct...
 */
void nav_speak2helsman() {

	/* Set the new alpha reference Value
	 * alpha = yaw-twd;
	 * alpha is either computed using the yaw-angle or the COG (Course over Ground) */
	float alpha_star = nh_sensor2dumas(nh_compass2sensor(state.heading_ref) - nh_compass2sensor(state.wind_dir));

	#if C_DEBUG == 0
	cb_set_alpha_star(alpha_star);
	#endif


	/* Tell the Helsman to tack as soon as possible, if pathplanning wants to tack */
	if(state.maneuver) {
		//A maneuver is necessary => tell the helsman to tack
		#if C_DEBUG == 0
		cb_do_maneuver(alpha_star);
		#endif
	}

    #if C_DEBUG == 1
		printf("New Heading Reference: %f / ",state.heading_ref*RAD2DEG);
		printf("New Alpha Star: %f",alpha_star*RAD2DEG);

		//Write a file with the shared-Memory Content for Matlab
		FILE *f = fopen("sharedMemory.txt", "a");
		if (f == NULL) {
			printf("Error opening file!\n");
		}

		fprintf(f,"%f, %d, ",state.heading_ref,state.maneuver);
		fclose(f);

	#endif
}



/**
 * New information about the heading is available. Therefore, the state of the navigator needs to be updated
 *
 * @param *strs_p: Pointer to the topics-struct
 */
void nav_heading_update(void) {

	/* Get the new alpha Value
	 * alpha = yaw-twd;
	 * alpha is either computed using the yaw-angle or the COG (Course over Ground) */
	float alpha = 0;
	#if C_DEBUG == 0
	alpha =  cb_get_alpha();
	#endif

	state.heading_cur = nh_dumas2compass(alpha + state.wind_dir);

} //end of nav_heading_update



/**
 * New information about the wind is available. Tell these values to the navigator.
 *
 * Note: This function updates the internal navigator-state
 */
void nav_wind_update(void) {

	/* Get the new Windspeed Value
	 * Note: The Wind is measured in Sensor-Frame! => convert it to Compass-Frame! */
	#if C_DEBUG == 0
	cb_get_tw_info(&(state.wind_dir),&(state.wind_speed));
	#endif

	state.wind_dir = nh_sensor2compass(state.wind_dir);


	/* Update the Centerline
	 * The Centerline always has the same direction as the mean wind. And starts at the next target.*/
	float dx = sinf(state.wind_dir);
	float dy = cosf(state.wind_dir);
	float norm = sqrtf(dx*dx+dy*dy);

	field.centerline.northx = dx/norm;
	field.centerline.easty = dy/norm;

} //end of nav_heading_update



/**
 * New Information about the position is available. Therefore, the state-struct must be updated.
 *
 * Meanwhile it is checked if the boat already reached one of its target positions and the new
 * waypoint (target) is set accordingly.
 *
 */
void nav_position_update(void) {

	/*Get the new NED-Position of the boat from the communication buffer
	 * Note: The NED-Position is in decimeters => convert to meters */
	int32_t ned[3];
	#if C_DEBUG == 0
	cb_get_boat_ned(ned);
	#endif

	NEDpoint newPos;
	newPos.northx = ned[0]/10.0f;
	newPos.easty = ned[1]/10.0f;
	newPos.downz = ned[2]; 			//Pathplanning does not need the down-value


	//Check, if we reached a target
	if(nh_ned_dist(newPos,field.targets[state.targetNum]) <= TARGETTOLERANCE) {
		//We are inside the tolerance => target is counted as reached

		if(state.targetNum != (field.NumberOfTargets-1)) {
			//This is not the last target => set new Target

			state.targetNum += 1;
		} else {
			//This was the last target

			//Don't know what to do here...just be happy?... maybe report to QGround Control?

		}
	}

	//Update the state to the new Position
	state.position = newPos;

} //end of nav_heading_update



/**
 * The start-line is defined by two buoys. Set the two buoys in GEO-Frame and
 * convert it to NED.
 *
 * @param buoy1: left buoy in GEO-Coordinates
 * @param buoy2: right buoy in GEO-Coordinates
 */
void nav_set_startline(Point buoy1, Point buoy2) {
	field.startline[0] = nh_geo2ned(buoy1);
	field.startline[1] = nh_geo2ned(buoy2);
}



/**
 * Set a new Obstacle
 * This functions is called by QGroundControl to set a new Value
 *
 * @param ObstNumber: The position of the obstacle in the Array of all Obstacles
 * @param ObstPos: The GPS-Position of the obstacle represented as a Point
 */
void nav_set_obstacle(uint8_t ObstNumber, Point ObstPos) {

	/* Convert to NED-Frame */
	field.obstacles[ObstNumber] = nh_geo2ned(ObstPos);

	field.NumberOfObstacles = ObstNumber;
}


/**
 * Set a new Target
 * This functions is called by QGroundControl to set a new Value
 *
 * @param TargetNumber: The position of the target in the Array of all Targets
 * @param TargetPos: The GPS-Position of the target represented as a Point
 */
void nav_set_target(uint8_t TargetNumber, Point TargetPos) {

	field.targets[TargetNumber] = nh_geo2ned(TargetPos);

	field.NumberOfTargets = TargetNumber;
}



/**
 * Set the configuration of the Navigator by QGround Control
 * This functions is called by QGroundControl to set a new Value
 *
 * @param period: Time between two calls to pathplanning [us]
 */
void nav_set_configuration(uint64_t period) {

	config.period = period;
}


/* FUNCTIONS FOR DEBUGGING */
#if C_DEBUG == 1

	/*
	 * Set a fake State for the navigator
	 *
	 * @param pos: Position in NED-Frame
	 * @param heading: heading of the boat in Compass-Frame in Degrees
	 */
	void DEBUG_nav_set_fake_state(NEDpoint pos, float heading) {

		state.heading_cur = DEG2RAD*heading;
		state.position = pos;

		//Set Environmental conditions
		state.wind_dir = 0;
		state.wind_speed = 3;

		//Always take the first Target
		state.targetNum = 0;

	}


	/*
	 * Set a fake Field for the navigator
	 *
	 * @param target: Target Position in NED-Frame
	 * @param obstacle: Obstacle Position in NED-Frame
	 */
	void DEBUG_nav_set_fake_field(NEDpoint target, NEDpoint obstacle) {

			field.targets[0] = target;
			field.obstacles[0] = obstacle;

			//printf("Target: %f/%f",field.targets[0].northx,field.targets[0].easty);
			//printf("Obstac: %f/%f",field.obstacles[0].northx,field.obstacles[0].easty);


			//Only one Target/Obstacle
			field.NumberOfTargets = 1;
			field.NumberOfObstacles = 1;
		}

#endif




