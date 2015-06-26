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
 * - Winddirection => how's the definition? Wind from North = 0°/ Wind from South = 180° (Sensor-Frame)
 *
 * -
 */


#include "pp_config.h"
#include "pp_navigator.h"
#include "pp_cost_method.h"
#include "pp_potentialfield_method.h"

#include "kalman_tracker/kt_tracker.h"
#include "kalman_tracker/kt_track_list.h"

#include <stdbool.h>
#include <stdlib.h>

#if C_DEBUG == 0
#include "pp_communication_buffer.h"
#include "pp_failsafe.h"
#include <drivers/drv_hrt.h>
#endif

//static char txt_msg[150]; ///used to send messages to QGC

/***********************************************************************************/
/*****  V A R I A B L E S  *********************************************************/
/***********************************************************************************/

/** Struct holding the main configuration variables of the navigator */
static struct {
	uint64_t period; 		//The period of calls to Pathplanning (time between two calls to nav_navigate())
	float max_headchange;	//Maximum possible change in heading of the boat [rad]
	uint8_t method; 		//Which method should be used for Pathplanning
	 	 	 	 	 	 	/* Chooses a path-planning algorithm
	 	 	 	 	 	 	 * 1 = Cost-Function-Method
	 	 	 	 	 	 	 * 2 = Potential-Field-Method */
	bool reset; 			//Resets all local variables to a predefined state (init-state), if true
	bool use_yaw; 			//If set to true, the yaw from the weather station is used unfiltered for the Heading calculation
	bool nogybe; 			//If set, no maneuver is commanded on downwind courses => simply change the reference
} config = {
	.period = 1000000,
	.max_headchange = 0.1745329f, //~10°/s
	.method = 1,
	.reset = false,
	.use_yaw = false,
	.nogybe = false
};

uint16_t last_tar_num = 0; 	//Last changed Target number (from QGround Control)


/** Init the Struct containing the current state of the Navigator */
static struct nav_state_s state;


/** Init the Struct containing the Race-Field-Information */
static struct nav_field_s field;


/** Variable for enabling communication with autonomous sailing app
 * if true, the pathplanner is communicating with autonomous sailing app
 * Note: The pathplanner is running all the time in background. But as soon
 * 		 as this variable is set to true, it communicates the outputs to the
 * 		 helsman (autonomous sailing app) */
static bool enable_pathplanner = false;

/** If true, a quick_target is set and when switching to autonomous mode the next time
 * the target is set. */
static bool quick_target = false;
static NEDpoint ntarget;	//Target that should be reached when quick_target is set


#if SIMULATION_FLAG == 1
	static float last_alpha = 0;
#endif


static float dbg_alpha = 0;
static bool dbg_alpha_status = false;
static bool dbg_alpha_minus = false;

static uint8_t qground_obstnum = 0; //number, auto increasing in every step. It is used to store all obstacles in the SD-Log



/***********************************************************************************/
/*****  F U N C T I O N   P R O T O T Y P E S **************************************/
/***********************************************************************************/





/***********************************************************************************/
/*****  P U B L I C    F U N C T I O N S  ******************************************/
/***********************************************************************************/


/**
 * Init a new Navigator by creating all necessary variables.
 *
 */
void nav_init(void) {

	//Set the initial values for the Field
	field.NumberOfTargets = 1;
	field.NumberOfObstacles = 1;

	//Set the initial Target-Position
	for(uint8_t i=0; i < MAXTARGETNUMBER; i++) {
		field.targets[i].northx = 0;
		field.targets[i].easty = 0;
	}

	//Set the initial Obstacle-Position
	for(uint8_t i=0; i < MAXOBSTACLENUMBER; i++) {
		field.obstacles[i].northx = 0;
		field.obstacles[i].easty = 0;
	}

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
	state.command_maneuver = false;

	//Set the initial Values for the Configuration
	config.period = 1000000;			// = 1s
	config.max_headchange = 2.5f*0.17453292f; // = 12°/period
	config.method = 1; //As a default use the Cost-Function-Method
	config.reset = false;
	config.use_yaw = false;


	//Update the state by requesting the values from the communication Buffer
	nav_wind_update();		//Check for new Wind measurements
	nav_heading_update();	//Check for a new Heading (alpha)
	nav_position_update();  //Check for a new Position update


	//Disable the Pathplanner by default
	enable_pathplanner = false;
	quick_target = false;
	ntarget = state.position;



	//For Debug only
	//Set a fake-field, as it is used in matlab for the competition-task
	#if P_DEBUG == 1
	NEDpoint target;
	target.northx = 0;
	target.easty = 300;
	NEDpoint obstacle;
	obstacle.northx = 0;
	obstacle.easty = 150;
	DEBUG_nav_set_fake_field(target,obstacle);
	#endif


	//DEBUG ONLY
	dbg_alpha = 0;
	dbg_alpha_status = false;
}

/**
 * Reset the Pathplanner to restart the path => start again with the first target
 */
void nav_reset(void) {
	state.targetNum = 0;
}



/**
 * Calculate a new optimal reference Heading
 */
void nav_navigator(void) {

	#if C_DEBUG == 0
	uint64_t systime = hrt_absolute_time();
	#else
	uint64_t systime = 0;
	#endif



	/** MANEUVER IS IN PROGRESS
	 * Listen to Navigator if a maneuver is in progress.
	 * The Navigator needs to wait until the helsman has finished the maneuver */
	if(state.maneuver == true) {
		//A maneuver is in progress

		if(cb_is_maneuver_completed()) {
			//Wait for maneuver to be completed...

			state.maneuver = false;				//The maneuver flag can be reseted and pathplanning can be done again...
			state.command_maneuver = false;

			//smq_send_log_info("Maneuver is completed! JW");


			#if SIMULATION_FLAG == 1
			//The Maneuver is completed => we must set a new current heading, otherwise the pathplanner forces to do
			//another maneuver...and another...and another... endless-while!

			//Alpha is given in Duma's Frame. Therefore, it needs to be converted to the
			//Compass-Frame. heading = last_alpha + twd
			float alpha = last_alpha + state.wind_dir;

			if(alpha < 0) {
				alpha = 2*PI + alpha;
			}

			if(alpha > 2*PI) {
				alpha = alpha - 2*PI;
			}

			//alpha = fmod(alpha,2*PI);

			state.heading_cur = alpha; //270 * DEG2RAD; //alpha;

			#endif

		}
	}


	/** SET A QUICK TARGET
	 * A quick-target is set.  We wait for the remote control to be switched to autonomous mode and then
	 * set the target.
	 */
	if(quick_target == true) {
		//A quick-target was set.

		if(cb_is_autonomous_mode() == true) {
			//The remote control is in autonomous mode now => we can set the quick target

			//Reset the flag
			quick_target = false;

			//Set the target
			field.targets[0] = ntarget;
			state.targetNum = 0;
			field.NumberOfTargets = 1;

			//Inform QGround Control
			smq_send_log_info("Quick Target was set!");
			cb_new_target(field.targets[state.targetNum].northx, field.targets[state.targetNum].easty);

			//Enable the Pathplanner
			enable_pathplanner = true;

		}
	}


	/** ENABLE PATHPLANNER WHEN IN AUTONOMOUS MODE */
	if(cb_is_autonomous_mode() == true) {
		//The remote control is in autonomous mode now => we can enable the pathplanner

		enable_pathplanner = true;
	} else {
		//The remote control is NOT in autonomous mode => we disable the pathplanner

		enable_pathplanner = false;
	}


	/** FAILSAFE
	 * When failsafe is enabled, the next target is always forced to be the HOME position
	 */
	#if USE_FAILSAFE == 1
		if(fs_is_failsafe_active() == true) {

			//We force the next target to be the HOME position
			Point home;
			home.lat = HOMELAT;
			home.lon = HOMELON;
			home.alt = HOMEALT;

			field.targets[0] = nh_geo2ned(home);
			field.NumberOfTargets = 1;

			state.targetNum = 0;

			//Communicate Failsafe to autonomous sailing app
			//cb_set_failsafe(true); //As soon as we comment this in, the software crashes...don't know why...

			smq_send_log_info("FAILSAFE: Navigating HOME... (JW)!");
		}
	#endif






	/** MAIN PATHPLANNING
	 *  Pathplanning is only done with a certain frequency AND if no maneuver is under progress
	 *  Therefore, check the systemtime.
	 *  Note: Pathplanning is NEVER done during maneuvers */

	if((systime-state.last_call >= config.period) && (state.maneuver == false)) {

		/** Assign the current time as the last call time */
		state.last_call = systime;


		//** Check if new information is available and change the state accordingly */
		#if SIMULATION_FLAG == 0
		//Note: This information is only available, when the boat is not in test-mode
		nav_wind_update();			//New Wind-Data
		nav_heading_update();   	//New Heading-Data
		nav_position_update();  	//New Position-Data
		#endif


		//****SEND THE DATA USED FOR PATHPLANNING TO QGROUND CONTROL
		cb_new_target(field.targets[state.targetNum].northx, field.targets[state.targetNum].easty);

		/*cb_new_obstacle(field.obstacles[qground_obstnum].northx, field.obstacles[qground_obstnum].easty);
		qground_obstnum++;
		if(qground_obstnum>field.NumberOfObstacles) {
			qground_obstnum = 0;
		}*/

		//Log the Sensor-Obstacles
		cb_new_obstacle(field.sensorobstacles[qground_obstnum].northx,field.sensorobstacles[qground_obstnum].easty);
		qground_obstnum++;
		if(qground_obstnum>field.NumberOfSensorobstacles) {
			qground_obstnum = 0;
		}

		cb_new_targetnum(state.targetNum);



		//****GET THE OBSTACLES IDENTIFIED BY THE SENSOR
		//Note: This is only executed, if the Kalman Tracker is activated by QGround Control
		//get_sensor_obstacles();


		/****FIND A NEW REFERENCE HEADING
		 * Different algorithms can be used. */
		if(config.method == 1) {
			//Use Cost-Function-Method

			state.heading_ref = cm_NewHeadingReference(&state,&field);
		}

		if(config.method == 2) {
			//Use Potential-Field-Method

			state.heading_ref = pm_NewHeadingReference(&state,&field);
		}

		//Display the new reference heading in QGround Control (also Log this on SD-Card)
		cb_new_refheading(state.heading_ref);


		//****DECISION MAKING
		/* In the following section the decisions based on the optimal Heading are made. In particular
		 * the Navigator decides if the boat should tack or gybe or just track a reference heading */
		float NewWind = nh_appWindDir(state.heading_ref,state.wind_dir); 		//New Apparent Winddirection
		float OldWind = nh_appWindDir(state.heading_cur,state.wind_dir);		//Current Apparent Winddirection


		/*Decide if we have to do a tack or a gybe
		 * A maneuver is necessary, iff we change the hull. A change of hull is represented as a change of sign of the
		 * apparent Wind direction.
		 */
		if(!((NewWind < 0 && OldWind < 0) || (NewWind > 0 && OldWind > 0))) {
			//A Maneuver is Necessary

			state.command_maneuver = true;

			//**START NEW CODE
			if(fabsf(NewWind) > DOWNWIND_COURSE && config.nogybe == true) {
				//We are sailing on a downwind course => when we tell the low-level control to perform a maneuver,
				//then the boat starts behaving crazy => therefore do not tell the boat to gybe and simply change the
				//reference heading

				state.command_maneuver = false;
				state.maneuver = false;
			}
			//**END NEW CODE

		} else {
			//No Maneuver is necessary and we can therefore set all flags to zero
			state.command_maneuver = false;
			state.maneuver = false;
		}//if boat should do a maneuver


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
 * A new heading reference is available. Communicate this new information to the "autonomous_sailing module".
 * Therefore, speak to the Helsman.
 * NOTE: The helsman needs to know if he/she has to do a maneuver or just sailing at a given alpha (angle wrt. true wind) angle
 *
 */
void nav_speak2helsman() {

	//***DEBUG ONLY
	//If we entered an alpha manually, we send this alpha to the autonomous sailing app
	if(dbg_alpha_status) {
		cb_set_alpha_star(dbg_alpha);
		return;
	}


	/* Set the new alpha reference Value
	 * alpha = yaw-twd;
	 * alpha is either computed using the yaw-angle or the COG (Course over Ground) */

	float alpha_star = nh_appWindDir(state.heading_ref, state.wind_dir);
	alpha_star = -alpha_star;	//Invert alpha, because of Duma's convention

	#if SIMULATION_FLAG == 1
		//Store the current Alpha (this is needed to set a new current alpha star, after the boat has "virtually" done a maneuver
		last_alpha = alpha_star;
	#endif


	/* DEBUG:
	 * If the Debug Flag for inverted Signs for alpha is set, the sign of alpha is inverted here
	 */
	/*if(dbg_alpha_minus == true) {
		alpha_star = -alpha_star;
	}*/



	/* Tell the Helsman to tack/gybe as soon as possible, if pathplanning wants to tack/gybe */
	if(state.command_maneuver == true) {
		//A maneuver is necessary

		if(cb_is_maneuver_completed()==true) {
			//Check if the previous maneuver is completed before commanding a maneuver

			if(enable_pathplanner == true) {
				cb_do_maneuver(alpha_star);			//Tell the helsman to do a maneuver
			}

			//smq_send_log_info("HELSMAN: Do maneuver! JW");
		} else {
			//smq_send_log_info("HELSMAN: Finish the maneuver! JW");
		}
		state.command_maneuver = false;		//The command has been sent to the navigator => no need to tell it any more
		state.maneuver = true;				//A maneuver is in progress => wait for maneuver completed
	} else {
		//No maneuver is necessary => command the course the helsman should sail at

		if(cb_is_maneuver_completed()==true) {
			//Check if the previous maneuver is completed before commanding a new alpha

			if(enable_pathplanner == true) {
				cb_set_alpha_star(alpha_star);		//Send the new reference to autonomous sailing app
			}

			//smq_send_log_info("Do normal sailing... JW");

			state.command_maneuver = false;
			state.maneuver = false;
		}
	}

}



/**
 * New information about the heading is available. Therefore, the state of the navigator needs to be updated
 *
 * @param *strs_p: Pointer to the topics-struct
 *
 * Debug-State: Should be OK, Tested in a separate Program
 */
void nav_heading_update(void) {

	if(config.use_yaw == false) {
		//If we do not use the yaw from the magnetic compass, we use the alpha provided by the autonomous sailing app
		//for the heading calculation! => Note: this is standard, we use the alpha as a standard

		/* Get the new alpha Value
		 * alpha = yaw-twd => yaw = alpha + twd;
		 * alpha is either computed using the yaw-angle or the COG (Course over Ground) */
		float alpha =  cb_get_alpha();

		//Convert from Dumas to Sensor Frame
		alpha = nh_dumas2sensor(alpha); //This is nothing else than a change of Sign!


		/* Alpha is given in Duma's Frame. Therefore, it needs to be converted to the
		 * Compass-Frame. heading = alpha + twd
		 * The wind direction is in compass frame, since it gets converted when a new wind-direction is available! */
		alpha = alpha + state.wind_dir;

		//fmod() of the yaw
		//Note: fmod() from <math.h> does NOT work, since it needs positive values!
		if(alpha > 4*PI) {
			alpha = alpha - 4*PI;
		}

		if(alpha > 2*PI) {
			alpha = alpha - 2*PI;
		}

		if(alpha < 0) {
			alpha = 2*PI + alpha;
		}

		state.heading_cur = alpha;

		//Send current Heading to QGround control for debugging
		cb_new_heading(state.heading_cur);

	}

} //end of nav_heading_update



/**
 * Use only yaw for calculating the current heading of the boat
 *
 * @param Pointer to the struct of the topic
 */
void yaw_update(struct structs_topics_s *strs) {

	if(config.use_yaw == true) {
		state.heading_cur = nh_sensor2compass(strs->vehicle_attitude.yaw);

		//Send current Heading to QGround control for debugging
		//TOOD: Commented out, because is used for targetnumber
		//cb_new_heading(state.heading_cur);
	}

}



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

	//Send current Wind-Value to QGround Control for debugging
	cb_new_wind(state.wind_dir);

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
	if(enable_pathplanner == true)
		//We only check, if we have reached the target, when the pathplanner is on <=> we are in autonomous mode
		if(nh_ned_dist(newPos,field.targets[state.targetNum]) <= TARGETTOLERANCE) {
			//We are inside the tolerance => target is counted as reached

			if(state.targetNum != (field.NumberOfTargets)) {
				//This is not the last target => set new Target

				state.targetNum += 1;
				//state.targetNum = 0;
				smq_send_log_info("Waypoint reached!");

				//Send the new target to QGround Control
				cb_new_target(field.targets[state.targetNum].northx, field.targets[state.targetNum].easty);

			} else {
				//This was the last target

				//Don't know what to do here...just be happy?... maybe report to QGround Control?
				smq_send_log_info("Final target reached!");

				//We set again the first target as the next target => ensures that the boat alway has a target position to reach
				state.targetNum = 0;

			}
		}

	//Update the state to the new Position
	state.position = newPos;

	//Send new Position-Value to QGround Control for debugging
	cb_new_position(state.position.northx, state.position.easty);

} //end of nav_heading_update


/**
 * Get the current position of the boat known as by the navigator
 *
 */
NEDpoint nav_get_position(void) {
	return state.position;
}



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

	field.NumberOfObstacles = ObstNumber+1;
	#endif

	//Send new Obstacle Position to QGround Control for debugging
	cb_new_obstacle(field.obstacles[ObstNumber].northx, field.obstacles[ObstNumber].easty);
}


/**
 * Set a new Obstacle in NED-Coordinates
 * This functions is called by QGroundControl to set a new Value
 *
 * @param ObstNumber: The position of the obstacle in the Array of all Obstacles
 * @param ObstPos: The GPS-Position of the obstacle represented as a Point in NED-Coordinates
 */
void nav_set_obstacle_ned(uint8_t ObstNumber, NEDpoint ObstPos) {

	#if P_DEBUG == 0
	//The update of the target position should only be done, if we are not debugging
	field.obstacles[ObstNumber] = ObstPos;

	field.NumberOfObstacles = ObstNumber+1;
	//field.NumberOfObstacles = 1;	//TODO: This is for debug only. We assume that we have only one obstacle!
	#endif

	//Send new Obstacle Position to QGround Control for debugging
	cb_new_obstacle(field.obstacles[0].northx, field.obstacles[0].easty);
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
	field.NumberOfTargets = TargetNumber + 1;

	cb_new_target(field.targets[state.targetNum].northx, field.targets[state.targetNum].easty);

	#endif
}


/**
 * Set a new Target in NED-Coordinates
 * This functions is called by QGroundControl to set a new Value
 *
 * @param TargetNumber: The position of the target in the Array of all Targets
 * @param TargetPos: The GPS-Position of the target represented as a Point in NED-Coordinates
 */
void nav_set_target_ned(uint8_t TargetNumber, NEDpoint TargetPos) {

	#if P_DEBUG == 0
	//The update of the target position should only be done, if we are not debugging
	field.targets[TargetNumber] = TargetPos;
	field.NumberOfTargets = TargetNumber + 1;

	cb_new_target(field.targets[state.targetNum].northx, field.targets[state.targetNum].easty);

	#endif

}



/**
 * Set the configuration of the Navigator by QGround Control
 * This functions is called by QGroundControl to set a new Value
 *
 * @param period: Time between two calls to pathplanning [s]
 * @param turnrate: Maximum Turnrate of the boat [°/s]
 */
void nav_set_configuration(float period, uint32_t turnrate) {

	//Store the period
	if(period > 0) {
		//The period can only be changed if it is bigger than zero. Otherwise the navigator is called in every loop and
		//the computational cost increases rapidly, what leads to a total system failure!
		config.period = period*1000000.0f;
	} else {
		config.period = 1000000.0f;	//Set default value
	}

	//Store the maximum possible change in Heading between two consecutive
	//executions of Path planning
	config.max_headchange = turnrate * RAD2DEG * period;
}


/**
 * Set the noGybe-Variable. If set, no gybe is commanded on downwind courses => simply change the reference
 *
 * @param nogybe: 1, if no gybe should be commanded
 */
void nav_set_nogybe(uint8_t status) {

	if(status == 1) {
		config.nogybe = true;
	} else {
		config.nogybe = false;
	}

}


/**
 * Enable the use of the navigator
 * This function is called by QGroundControl to set a new Value
 *
 * @param enable: if 1, pathplanner is enabled, else, disabled
 */
void nav_enable_navigator(uint8_t enable) {
	if(enable == 1) {
		enable_pathplanner = true;
	} else {
		enable_pathplanner = false;
	}
}



/**
 * Define the method that should be used for Pathplanning
 * This function is called by QGroundControl to set a new Value
 *
 * @param method: Integer representing a method. The selectable methods
 * 				  are defined in the definition of the struct "config" at
 * 				  the top of this page.
 */
void nav_set_method(uint8_t method) {

	//Send an appropriate message to QGround Control, iff method has changed
	if(config.method != method) {
		if(method == 1) {
			smq_send_log_info("USE COST-METHOD");
		} else if(method == 2) {
			smq_send_log_info("USE POTENTIAL-METHOD");
		} else {
			smq_send_log_info("ERROR: METHOD UNKNOWN!");
		}
	}

	if(method > 0) {
		config.method = method;
	}
}

/**
 * Enable/Disable the use of the yaw only for heading measurement of the boat
 * Note: This function is called by QGroundControl
 *
 * @param status: 1 = enable, 0 = disable
 */
void nav_set_use_yaw(uint8_t status) {
	if(status == 1) {
		config.use_yaw = true;
	} else {
		config.use_yaw = false;
	}
}



/*
 * Set the current position of the boat as the next target position.
 * Then one can sail away from this position manually and then switch to
 * autonomous mode. The pathplanner should then guide the boat back to this position.
 */
void nav_set_quick_target(void) {
	ntarget = state.position;

	quick_target = true;
}


/**
 * Set the current number of target to be reached.
 */
void nav_set_targetnumber(uint8_t tar_num) {

	if(tar_num != last_tar_num) {
		//The user has changed the value of the target number
		//Only in this case, we feed a new target number to the algorithm

		state.targetNum = tar_num;

		last_tar_num = tar_num;
	}


}




/***********************************************************************************/
/*****  P R I V A T E    F U N C T I O N S  ****************************************/
/***********************************************************************************/


/**
 * Get the Obstacles from the Sensor and store them in the Race-Field-Matrix
 *
 */
bool nav_get_sensor_obstacles(void) {

	if(kt_get_state() == true) {
		//The Kalman Tracker is active => We want to include the obstacles in the race-field

		//Free the previously allocated memory, since we are going to update the number of obstacles
		free(field.sensorobstacles);

		//Allocate memory for the obstacles
		field.sensorobstacles = malloc(kt_get_nrofobstacles()*sizeof(NEDpoint));

		//Get the obstacles from the linked-list
		field.NumberOfSensorobstacles = kt_get_obstacles(field.sensorobstacles);
	} else {
		//Make sure the Array for Sensor Obstacles is empty

		NEDpoint origin;
		origin.northx = 0;
		origin.easty = 0;
		for (uint8_t ind = 0; ind < field.NumberOfSensorobstacles; ind++) {

			field.sensorobstacles[ind] = origin;
		}

		//free(field.sensorobstacles);
		field.NumberOfSensorobstacles = 0;
	}

	return true;

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
	state.wind_dir = 90*DEG2RAD;
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
			state.targetNum = 0;

			//Only one Target/Obstacle
			field.NumberOfTargets = 1;
			field.NumberOfObstacles = 1;

			cb_new_target(field.targets[state.targetNum].northx, field.targets[state.targetNum].easty);
		}


/**
 * Set alpha manually. => in order to check correct working of low-level controller
 */
void DEBUG_nav_setalpha(uint8_t status, float alpha) {

	if(status == 1) {
		dbg_alpha = DEG2RAD*alpha;
		dbg_alpha_status = true;
	} else {
		dbg_alpha_status = false;
	}

}


/**
 * Invert Sign of alpha, if QGround-Control Variable is set
 */
void DEBUG_nav_alpha_minus(uint8_t status) {
	if(status == 1) {
		dbg_alpha_minus = true;
	} else {
		dbg_alpha_minus = false;
	}
}



