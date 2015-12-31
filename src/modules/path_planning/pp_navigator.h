/*
 * pp_navigator.h
 *
 * This file contains a navigator. The navigator calculates a new heading reference and gives orders to the helsman.
 * In our case the helsman is the "autonomous_sailing module". The helsman polls the uORB topic "path_planning" for
 * changes and adjustes its control according to the orders.
 *
 *  Created on: 04.03.2015
 *      Author: Jonas Wirz <wirzjo@student.ethz.ch>
 */

#ifndef PP_NAVIGATOR_H_
#define PP_NAVIGATOR_H_

#include <stdbool.h>

#include "pp_config.h"
#include "pp_navigation_helper.h"

#if C_DEBUG == 0
#include "pp_topics_handler.h"
#endif


/**Struct containing the status of the Navigator */
struct nav_state_s {
	NEDpoint position; 				//Last known Position (x,y in NED-Frame) [m]
	float heading_cur;				//Current Heading of the boat [rad]	(Compass-Frame)
	float heading_ref;				//Heading Reference for optimal path [rad] (Compass-Frame)
	float wind_dir; 				//average direction of the wind (where the wind is coming from) [rad] (Compass-Frame)
	float wind_speed; 				//average Wind Speed [m/s]
	uint8_t targetNum; 				//Current number of target to be reached (limits number of targets to 256)
	bool maneuver;					//true, iff a maneuver is in progress
	uint64_t last_call; 			//Systemtime, when the Pathplanning was done the last time [us]
	bool command_maneuver;			//True, if the boat should do a maneuver, false else(<=> command a maneuver)
};

/**Struct containing the Race-Field-Information */
struct nav_field_s {
	NEDpoint targets[MAXTARGETNUMBER];	//Matrix holding all targets (waypoints) (in NED-Frame)
	uint8_t NumberOfTargets;		//Number of waypoints currently in the Matrix
	NEDpoint obstacles[MAXOBSTACLENUMBER];	//Matrix holding the obstacles (in NED-Frame)
	uint8_t NumberOfObstacles;		//Number of obstacles currently in the Matrix

	NEDpoint *sensorobstacles;		//Pointer to a dynamic array holding the sensor obstacles
	uint8_t NumberOfSensorobstacles; //Number of Sensor Obstacles currently in the Matrix

	NEDpoint startline[2]; 			//Matrix containing the start-line [buoy1, buoy2]
};



/** @brief Init a Navigator */
extern void nav_init(void);


/** @brief Reset the navigator to start again with the first target */
void nav_reset(void);


/** @brief Calculate a new optimal heading reference */
void nav_navigator(void);


/** @brief Speak to the helsman */
void nav_speak2helsman(void);


/** @brief New position information is available */
void nav_position_update(void);


/** @brief Return the position of the boat as known by the Navigator */
NEDpoint nav_get_position(void);


/** @brief New wind-information is available */
void nav_wind_update(void);


/** @brief New heading information is available */
void nav_heading_update(void);


/** @brief Use the Yaw for calculating the heading */
void yaw_update(struct pp_structs_topics_s *strs);

/** @brief update mission */
void mission_update(struct mission_planning_s mp);

/** @brief Set Obstacles */
extern void nav_set_obstacle(uint8_t ObstNumber, PointE7 ObstPos);


/** @brief Set Obstacles in NED-Coordinates */
extern void nav_set_obstacle_ned(uint8_t ObstNumber, NEDpoint ObstPos);


/** @brief Set Targets */
extern void nav_set_target(uint8_t TargetNumber, PointE7 TargetPos);


/** @brief Set Targets in NED-Coordinates */
extern void nav_set_target_ned(NEDpoint TargetPos);


/** @brief Set the number out of the stored targets should be reached */
void nav_set_targetnumber(uint8_t tar_num);


/** @brief Set the Start-Line */
void nav_set_startline(PointE7 buoy1, PointE7 buoy2);


/** @brief Set Configuration Parameters for the Navigator */
void nav_set_configuration(float period, uint32_t turnrate);


/** @brief Set, if a Gybe should be commanded or not (QGround Control)*/
void nav_set_nogybe(uint8_t status);


/** @brief Set the current position of the boat as the next target position*/
void nav_set_quick_target(void);


/** @brief Set the current position of the boat as the "zero"-th obstacle */
void nav_set_quick_obstacle(void);


/** @brief Enable the communication between pathplanner and autonomous sailing app */
void nav_enable_navigator(uint8_t enable);


/** @brief Set the method that is used for pathplanning */
void nav_set_method(uint8_t method);


/** @brief Use Yaw instead of alpha from autonomous sailing app for Heading calcualtion of the boat */
void nav_set_use_yaw(uint8_t state);


/* FUNCTIONS FOR DEBUGGING */

/* @brief Set a fake-state for debugging */
void DEBUG_nav_set_fake_state(NEDpoint pos, float heading);


/* @brief Set a fake-field for debugging */
void DEBUG_nav_set_fake_field(NEDpoint target, NEDpoint obstacle);

/* @brief Set the alpha-reference for Autonomous Sailing App manually */
void DEBUG_nav_setalpha(uint8_t status, float alpha);

/* @brief Invert Alpha, before it is sent to the autonomous Sailing App */
void DEBUG_nav_alpha_minus(uint8_t status);


/* @brief Get the Obstacles identified by the Kalman Tracker */
bool nav_get_sensor_obstacles(void);


/* FUNCTIONS FOR THE QUEUE */
extern void nav_queue_init(void);
int nav_queue_put_wp(NEDpoint *new);
int nav_queue_next_wp(void);
int nav_queue_read(NEDpoint *old);


#endif /* PP_NAVIGATOR_H_ */
