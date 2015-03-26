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
 *
 * - Check ESSC-Loging....
 * - Test if Upwind Course is possible...
 * - Log Pathplanning - Constants on SD-Card
 *
 * - One solution: Copy autonomous_sailing app from marco's working version into my gid-branch (manually)
 * - Introduce flag and call the do_maneuver() by QGround Control
 * - A problem could be, that a maneuver command is sent right after startup...
 *   => a second one is sent to autonomous sailing, what makes it behave crazy, but why is then do_maneuver set to false????
 *   => evtl. comment out navigator and call do_maneuver() instead...
 *
 *
 * - Comment out the "cb_set_alpha_star(alpha_star);" in speak2helsman()
 */


#include "pp_config.h"
#include "pp_navigator.h"
#include "pp_cost_method.h"

#include "pp_communication_buffer.h"
#include <drivers/drv_hrt.h>
#include <stdbool.h>

//static char txt_msg[150]; ///used to send messages to QGC


/** Struct holding the main configuration variables of the navigator */
static struct {
	uint64_t period; 		//The period of calls to Pathplanning (time between two calls to nav_navigate())
	float max_headchange;	//Maximum possible change in heading of the boat [rad]
	uint8_t method; 		//Which method should be used for Pathplanning
	 	 	 	 	 	 	/* Chooses a path-planning algorithm
	 	 	 	 	 	 	 * 1 = Cost-Function-Method
	 	 	 	 	 	 	 * 2 = Potential-Field-Method */
	bool reset; 			//Resets all local varables to a predefined state (init-state), if true
	uint64_t maneuverduration;
} config = {
	.period = 1000000,
	.max_headchange = 0.1745329f, //~10°/s
	.method = 1,
	.reset = false,
	.maneuverduration = 10*1e6	  //A maneuver normally tooks no longer than 10s
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
	state.heading_ref = PI/2;
	state.wind_dir = 0;
	state.wind_speed = 3;
	state.maneuver = false;
	state.targetNum = 0;
	Point home;
	home.lat = HOMELAT;
	home.lon = HOMELON;
	state.position = nh_geo2ned(home);
	state.last_call = 0;
	state.maneuver_start = 0;
	state.command_maneuver = false;

	//Set the initial Values for the Configuration
	config.period = 1000000;			// = 1s
	config.max_headchange = 2.5f*0.17453292f; // = 12°/period
	config.method = 1; //As a default use the Cost-Function-Method
	config.maneuverduration = 10*1e6;


	//Update the state by requesting the values from the communication Buffer
	nav_listen2helsman();	//Check, if a maneuver is completed
	nav_heading_update();	//Check for a new Heading (alpha)
	nav_position_update();  //Check for a new Position update
	nav_wind_update();		//Check for new Wind measurements


	//For Debug only
	//Set a fake-field, as it is used in matlab for the competition-task
	#if P_DEBUG == 0
	//TODO: Set this to 1
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

	uint64_t systime = hrt_absolute_time();

	//A maneuver is in Progress => wait for the maneuver to be completed
	//TODO: remove false
	if(state.maneuver == true) {
		//DEBUG:
		smq_send_log_info("Maneuver completed... JW ");
		//state.maneuver = false; //DEBUG: delete this line in real program

		if(cb_is_maneuver_completed()) {
			//The maneuver is completed => the flag can be reset
			state.maneuver = false;
		}

		//For safety reason end the maneuver after a predefined time, if the communication buffer is not responding
		if((systime-state.maneuver_start) >= config.maneuverduration) {
			state.maneuver = false;

			smq_send_log_info("Safety Reset of the maneuver flag!");
		}
	}


	/** Pathplanning is only done with a certain frequency AND if no maneuver is under progress
	 *  Therefore, check the systemtime.
	 *  Note: When the Computer-Debug-Mode is on Pathplanning is done in every loop!*/

	if((systime-state.last_call >= config.period) && (state.maneuver == false)) {

		/** Assign the current time as the last call time */
		state.last_call = systime;


		//** Check if new information is available and change the state accordingly */
		#if P_DEBUG == 0
		//Note: This information is only available, when the boat is not in test-mode
		//nav_heading_update();   	//New Heading-Data TODO: Uncomment this
		//nav_position_update();  	//New Position-Data TODO: Uncomment this
		//nav_wind_update();		//New Wind-Data TODO: Uncomment this
		#endif


		//DEBUG: Send the current heading and Position known by the Navigator to QGroundControl
		cb_new_heading(state.heading_cur);
		cb_new_position(state.position.northx, state.position.easty);



		/** A new reference heading should only be calculated if the boat is not doing a maneuver
		 * The boat is not doing a maneuver, if the maneuver-flag is set to false! */
		//if(!state.maneuver) {

			/****FIND A NEW REFERENCE HEADING
			 * Different algorithms can be used. */
		if(config.method == 1) {
			//Use Cost-Function-Method

			state.heading_ref = cm_NewHeadingReference(&state,&field);
			//cb_new_wind(state.heading_ref); //TODO: DEBUG, set Wind as the calculated reference heading
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


		//TODO DEBUG ONLY: Use Position-Variables for the Apparent Wind-Results
		cb_new_position(NewWind,OldWind);



		/*Decide if we have to do a tack or a gybe
		 * A maneuver is necessary, iff we change the hull. A change of hull is represented as a change of sign of the
		 * apparent Wind direction.
		 */
		if(!((NewWind < 0 && OldWind < 0) || (NewWind > 0 && OldWind > 0))) {
			//A Maneuver is Necessary

			state.command_maneuver = true;

			#if P_DEBUG == 1
			//Never tell the Controller to do a Maneuver in LAB-Environment
			//state.maneuver = false;
			#endif


		} //if boat should do a maneuver


		/* The boat has a limited turnrate. Therefore, ensure that the pathplanning
		 * does not suggest heading-changes bigger than the maximum possible turnrate.
		 * The check is only done when the boat is not doing a maneuver. If you want to
		 * disable the turnrate, just set the value to a high value. */
		/*if(!state.maneuver) {
			//The boat is not doing a maneuver

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

		}*/


		/****COMMUNICATION
		* A new Reference Heading is generated => send this data to the Helsman (autonomous_sailing module) */
		nav_speak2helsman();

	} else {
		//We do NO pathplanning in this step

	} //if do pathplanning with a predefined frequency

} //end of nav_navigate



/**
 * The "autonomous_sailing module" has changed the topic "path_planning", because a maneuver was finished.
 * Therefore, the Helsman speaks to the Navigator => the Navigator has to listen.
 *
 * @param *strs_p: Pointer to the topics-struct
 */
void nav_listen2helsman(void) {

	if(cb_is_maneuver_completed()) {
		//The ongoing maneuver is completed and therefore the state can be reseted.

		state.maneuver = false;
	}

	/** A maneuver is under progress. A maneuver is normally completed after a certain time => reset the maneuver flag in this case.
	 * Note: This is very ugly, but it hopefully helps...*/
	if(state.maneuver) {
		if(hrt_absolute_time()-state.maneuver_start >= config.maneuverduration) {
			//state.maneuver = false;

			smq_send_log_info("Safety Reset of the maneuver flag!");
		}
	}

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
	//float alpha_star = 0;
	//alpha_star = fmod(state.heading_ref - state.wind_dir,2*PI); //In Compass-Frame
	//alpha_star = nh_compass2dumas(alpha_star);					//Convert to Duma's convention for Autonomous Sailing Module

	float alpha_star = nh_appWindDir(state.heading_ref, state.wind_dir);

	/* Tell the Helsman to tack/gybe as soon as possible, if pathplanning wants to tack/gybe */
	if(state.command_maneuver == true) {
		//A maneuver is necessary

		state.maneuver_start = hrt_absolute_time();	//Define the start of the maneuver
		if(cb_is_maneuver_completed()==true) {
			//Check if the previous maneuver is completed before commanding a maneuver
			cb_do_maneuver(-alpha_star);			//Tell the helsman to do a maneuver
			//cb_tack_now();
			smq_send_log_info("HELSMAN: Do maneuver! JW");
		} else {
			smq_send_log_info("HELSMAN: Finish the maneuver! JW");
		}
		state.command_maneuver = false;		//The command has been sent to the navigator => no need to tell it any more
		state.maneuver = true;				//A maneuver is in progress => wait for maneuver completed
	} else {
		//No maneuver is necessary => command the course the helsman should sail at

		if(cb_is_maneuver_completed()==true) {
			//Check if the previous maneuver is completed before commanding a maneuver

			cb_set_alpha_star(alpha_star);
			smq_send_log_info("Do normal sailing... JW");
		}
	}

	//Report the Result of the Pathplanning to QGround Control
	//sprintf(txt_msg, "New Alpha Star = %1.1f [deg], Maneuver = %d @ %d" , (double)(alpha_star*RAD2DEG),(int)(state.maneuver),(int)state.last_call);
	//sprintf(txt_msg, "New Alpha Star = %1.1f [deg], Maneuver = %d @ %d" , (double)(state.heading_ref*RAD2DEG),(int)(state.maneuver),(int)state.last_call);
	//smq_send_log_info(txt_msg);
}



/**
 * New information about the heading is available. Therefore, the state of the navigator needs to be updated
 *
 * @param *strs_p: Pointer to the topics-struct
 *
 * Debug-State: Should be OK, Tested in a separate Program
 */
void nav_heading_update(void) {

	/* Get the new alpha Value
	 * alpha = yaw-twd => yaw = alpha + twd;
	 * alpha is either computed using the yaw-angle or the COG (Course over Ground) */
	float alpha =  cb_get_alpha();


	/* Alpha is given in Duma's Frame. Therefore, it needs to be converted to the
	 * Compass-Frame. heading = alpha + twd */
	alpha = alpha + state.wind_dir;

	if(alpha < 0) {
		alpha = 2*PI + alpha;
	}

	alpha = fmod(alpha,2*PI);

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
	cb_get_tw_info(&(state.wind_dir),&(state.wind_speed));

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
	n_get_boat_ned(ned);

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
			smq_send_log_info("Final target reached!");

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

	#if P_DEBUG == 0
	//The update of the target position should only be done, if we are not debugging
	field.obstacles[ObstNumber] = nh_geo2ned(nh_e7_to_point(ObstPos));

	field.NumberOfObstacles = ObstNumber;
	#endif
}


/**
 * Set a new Target
 * This functions is called by QGroundControl to set a new Value
 *
 * @param TargetNumber: The position of the target in the Array of all Targets
 * @param TargetPos: The GPS-Position of the target represented as a Point
 */
void nav_set_target(uint8_t TargetNumber, PointE7 TargetPos) {

	#if P_DEBUG == 0
	//The update of the target position should only be done, if we are not debugging
	field.targets[TargetNumber] = nh_geo2ned(nh_e7_to_point(TargetPos));

	field.NumberOfTargets = TargetNumber;
	#endif
}



/**
 * Set the configuration of the Navigator by QGround Control
 * This functions is called by QGroundControl to set a new Value
 *
 * @param period: Time between two calls to pathplanning [s]
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

			//Only one Target/Obstacle
			field.NumberOfTargets = 1;
			field.NumberOfObstacles = 1;
		}





