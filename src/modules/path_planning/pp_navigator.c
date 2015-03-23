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
 * Note: The winddirection is defined as where the wind is coming from! (0° = North/ 180° = South) Sensorframe is used.
 *
 *  Created on: 04.03.2015
 *      Author: Jonas Wirz <wirzjo@student.ethz.ch>
 */

/* TODO:
 * - add Potentialfield Method
 * - Winddirection => how's the definition? Wind from Nort = 0°/ Wind from South = 180° (Sensor-Frame)
 * - Check E7-Convention (North must have 9 Digits!!!!) 470000000
 * - Add logging of Pathplanning parameters
 * - Add logging of ESSC-Parameters
 *
 */


#include "pp_config.h"
#include "pp_navigator.h"
#include "pp_cost_method.h"

#if C_DEBUG == 0
#include "pp_communication_buffer.h"
#include <drivers/drv_hrt.h>
#endif

static char txt_msg[150]; ///used to send messages to QGC


/** Struct holding the main configuration variables of the navigator */
static struct {
	uint64_t period; 		//The period of calls to Pathplanning (time between two calls to nav_navigate())
	float max_headchange;	//Maximum possible change in heading of the boat [rad]
	uint8_t method; 		//Which method should be used for Pathplanning
	 	 	 	 	 	 	/* Chooses a path-planning algorithm
	 	 	 	 	 	 	 * 1 = Cost-Function-Method
	 	 	 	 	 	 	 * 2 = Potential-Field-Method */
} config = {
	.period = 1000,
	.max_headchange = 0.1745329f, //~10°/s
	.method = 1
};


/** Init the Struct containing the current state of the Navigator */
static struct nav_state_s state;


/** Init the Struct containing the Race-Field-Information */
static struct nav_field_s field;






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

	//Set the initial Values for the Configuration
	config.period = 1000000;			// = 1s
	config.max_headchange = 2.5f*0.17453292f; // = 12°/period
	config.method = 1; //As a default use the Cost-Function-Method


	//Update the state by requesting the values from the communication Buffer
	#if C_DEBUG == 0
	nav_listen2helsman();	//Check, if a maneuver is completed
	nav_heading_update();	//Check for a new Heading (alpha)
	nav_position_update();  //Check for a new Position update
	nav_wind_update();		//Check for new Wind measurements
	#endif


	//For Debug only
	#if P_DEBUG == 1
	NEDpoint target;
	target.northx = 0;
	target.easty = 300;
	NEDpoint obstacle;
	obstacle.northx = 0;
	obstacle.easty = 150;
	DEBUG_nav_set_fake_field(target,obstacle);
	#endif
}



/**
 * Calculate a new optimal reference Heading
 */
void nav_navigator(void) {

	/** DEBUG: */
	#if P_DEBUG == 1
	//printf("  Navigator called: %d\n",state.last_call);
	#endif


	/** Check if new Information is available and update the state accordingly. */
	#if C_DEBUG == 0
	nav_listen2helsman();	//Check, if a maneuver is completed
	nav_wind_update();		//Check for new Wind measurements

	#if P_DEBUG == 0
	nav_position_update();  //Check for a new Position update
	nav_heading_update();	//Check for a new Heading (alpha)
	#endif

	#endif //C_DEBUG


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
			if(config.method == 1) {
				//Use Cost-Function-Method

				state.heading_ref = cm_NewHeadingReference(&state,&field);
			}

			if(config.method == 2) {
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

				#if P_DEBUG == 1
				//Never tell the Controller to do a Maneuver in LAB-Environment
				state.maneuver = false;
				#endif


			} //if boat should do a maneuver


			/* The boat has a limited turnrate. Therefore, ensure that the pathplanning
			 * does not suggest heading-changes bigger than the maximum possible turnrate.
			 * The check is only done when the boat is not doing a maneuver. If you want to
			 * disable the turnrate, just set the value to a high value. */
			if(false && !state.maneuver) {
				//TODO: Remove false....somewhere here's an error, because the boat does Q-tacks
				//No Maneuver is necessary

				float diff = nh_heading_diff(state.heading_cur, state.heading_ref);
				if(diff > config.max_headchange) {
					//The desired change in heading is bigger than the maximum possibe heading-change

					if(state.heading_cur-diff < 0) {
						if((2*PI+(state.heading_cur-diff)) - state.heading_ref < 0.00000001f) {
							//Reference lays on the left of Current Heading => -

							state.heading_ref = fmod(state.heading_cur - config.max_headchange,2*PI);

						} else {
							//Reference lays on the right of the current Heading => +

							state.heading_ref = fmod(state.heading_cur + config.max_headchange,2*PI);
						}
					} else {
						if((state.heading_cur-diff) - state.heading_ref < 0.00000001f) {
							//Reference lays on the left of current Heading => -

							state.heading_ref = fmod(state.heading_cur - config.max_headchange,2*PI);

						} else {
							//Reference lays on the right of current Heading => +

							state.heading_ref = fmod(state.heading_cur + config.max_headchange,2*PI);

						}
					}

				}

			}


			/****COMMUNICATION
			* A new Reference Heading is generated => send this data to the Helsman (autonomous_sailing module) */
			nav_speak2helsman();

		} //if no tack or gybe is in progress

	} else {
		//We do no pathplanning in this step
		//smq_send_log_info(" ");
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

	if(cb_is_maneuver_completed()) {
		//The ongoing maneuver is completed and therefore the state can be reseted.

		state.maneuver = false;
	}
	#endif

} //end of nav_listen2helsman



/**
 * A new heading reference is available. Communicate this new information to the "autonomous_sailing module".
 * Therefore, speak to the Helsman.
 *
 */
void nav_speak2helsman() {

	/* Set the new alpha reference Value
	 * alpha = yaw-twd;
	 * alpha is either computed using the yaw-angle or the COG (Course over Ground) */
	float alpha_star = 0;
	alpha_star = fmod(state.heading_ref - state.wind_dir,2*PI); //In Compass-Frame
	alpha_star = nh_compass2dumas(alpha_star);					//Convert to Duma's convention for Autonomous Sailing Module

	#if C_DEBUG == 0
	cb_set_alpha_star(alpha_star);
	#endif


	/* Tell the Helsman to tack/gybe as soon as possible, if pathplanning wants to tack/gybe */
	if(state.maneuver) {
		//A maneuver is necessary
		#if C_DEBUG == 0
		cb_do_maneuver(alpha_star);
		#endif
	}

    #if C_DEBUG == 1
		printf("New Heading Reference: %f / ",state.heading_ref*RAD2DEG);
		printf("New Alpha Star: %f\n",alpha_star*RAD2DEG);

		//Write a file with the shared-Memory Content for Matlab
		FILE *f = fopen("sharedMemory.txt", "a");
		if (f == NULL) {
			printf("Error opening file!\n");
		}

		fprintf(f,"%f, %d, ",state.heading_ref,state.maneuver);
		fclose(f);

	#endif

    #if P_DEBUG == 1
		printf("New Heading Reference: %f\n",(double)(state.heading_ref*RAD2DEG));
	#endif

	//Report the Result of the Pathplanning to QGround Control
	sprintf(txt_msg, "New Alpha Star = %0.1f [deg], Maneuver = %d", (double)(state.heading_ref*RAD2DEG),(int)(state.maneuver));
	smq_send_log_info(txt_msg);
}



/**
 * New information about the heading is available. Therefore, the state of the navigator needs to be updated
 *
 * @param *strs_p: Pointer to the topics-struct
 *
 * Debug-State: Should be OK
 */
void nav_heading_update(void) {

	/* Get the new alpha Value
	 * alpha = yaw-twd => yaw = alpha + twd;
	 * alpha is either computed using the yaw-angle or the COG (Course over Ground) */
	float alpha = 0;
	#if C_DEBUG == 0
	alpha =  cb_get_alpha();
	#endif


	/* Alpha is given in Duma's Frame. Therefore, it needs to be converted to the
	 * Compass-Frame.  */
	alpha = nh_dumas2compass(alpha);				//Convert to Compass-Frame


	/* The Heading of the Boat is needes, which is either the COG or the yaw-angle. */
	alpha = fmod(alpha + state.wind_dir,2*PI);		//Add Wind-Direction (heading = alpha + twd)

	state.heading_cur = alpha;

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
	n_get_boat_ned(ned);
	#endif

	NEDpoint newPos;
	newPos.northx = ned[0]/10.0f;
	newPos.easty = ned[1]/10.0f;
	newPos.downz = ned[2]/10.0f; 			//Pathplanning does not need the down-value


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
void nav_set_startline(PointE7 buoy1, PointE7 buoy2) {

	//Convert from int32 to double


	field.startline[0] = nh_geo2ned(nh_e7_to_point(buoy1));
	field.startline[1] = nh_geo2ned(nh_e7_to_point(buoy2));
}



/**
 * Set a new Obstacle
 * This functions is called by QGroundControl to set a new Value
 *
 * @param ObstNumber: The position of the obstacle in the Array of all Obstacles
 * @param ObstPos: The GPS-Position of the obstacle represented as a Point
 */
void nav_set_obstacle(uint8_t ObstNumber, PointE7 ObstPos) {

	/* Convert to NED-Frame */
	field.obstacles[ObstNumber] = nh_geo2ned(nh_e7_to_point(ObstPos));

	field.NumberOfObstacles = ObstNumber;
}


/**
 * Set a new Target
 * This functions is called by QGroundControl to set a new Value
 *
 * @param TargetNumber: The position of the target in the Array of all Targets
 * @param TargetPos: The GPS-Position of the target represented as a Point
 */
void nav_set_target(uint8_t TargetNumber, PointE7 TargetPos) {

	field.targets[TargetNumber] = nh_geo2ned(nh_e7_to_point(TargetPos));

	field.NumberOfTargets = TargetNumber;
}



/**
 * Set the configuration of the Navigator by QGround Control
 * This functions is called by QGroundControl to set a new Value
 *
 * @param period: Time between two calls to pathplanning [us]
 * @param turnrate: Maximum Turnrate of the boat [°/s]
 */
void nav_set_configuration(uint64_t period, uint32_t turnrate) {

	//Store the period
	config.period = period*1000000.0f;

	//Store the maximum possible change in Heading between two consecutive
	//executions of Path planning
	config.max_headchange = turnrate * RAD2DEG * period;
}


/* FUNCTIONS FOR DEBUGGING */
#if C_DEBUG == 1 || P_DEBUG == 1

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




