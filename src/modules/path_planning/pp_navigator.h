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

#if C_DEBUG != 1
	#include "pp_topics_handler.h"
#endif

/**Struct containing the status of the Navigator */
struct nav_state_s {
	Point position; 				//Last known Position (lat/lon) [rad]
	float heading_cur;				//Current Heading of the boat [rad]
	float heading_ref;				//Heading Reference for optimal path [rad]
	float wind_dir; 				//average direction of the wind (where the wind is blowing to) [rad]
	float wind_speed; 				//average Wind Speed [m/s]
	uint8_t targetNum; 				//Current number of target to be reached (limits number of targets to 256)
	bool maneuver;					//true, iff a maneuver is in progress
};

/**Struct containing the Race-Field-Information */
struct nav_field_s {
	Point targets[MAXTARGETNUMBER];	//Matrix holding all targets (waypoints) (in NED-Frame)
	uint8_t NumberOfTargets;		//Number of waypoints currently in the Matrix
	Point obstacles[MAXOBSTACLENUMBER];	//Matrix holding the obstacles (in NED-Frame)
	uint8_t NumberOfObstacles;		//Number of obstacles currently in the Matrix
};



/** @brief Init a Navigator */
void nav_init(void);


/** @brief Calculate a new optimal heading reference */
void nav_navigate(void);


/** @brief Listen to the helsman */
void nav_listen2helsman(const struct structs_topics_s *strs_p);


/** @brief Speak to the helsman */
void nav_speak2helsman(void);


/** @brief New position information is available */
void nav_position_update(const struct structs_topics_s *strs_p);


/** @brief New heading information is available */
void nav_heading_wind_update(const struct structs_topics_s *strs_p);


/** @brief Set Obstacles */
void nav_setObstacle(uint8_t ObstNumber, Point ObstPos);


/** @brief Set Targets */
void nav_setTarget(uint8_t TargetNumber, Point TargetPos);


#endif /* PP_NAVIGATOR_H_ */
