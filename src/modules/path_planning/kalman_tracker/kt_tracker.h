/*
 * kt_tracker.h
 *
 *  Created on: 22.05.2015
 *      Author: Jonas Wirz <wirzjo@student.ethz.ch>
 */

#ifndef KT_TRACKER_H_
#define KT_TRACKER_H_

#include "../pp_config.h"
#include "../pp_navigation_helper.h"

/* @brief Init the use of the Kalman Tracker */
bool tr_init(void);


/* @brief Hanlder funciton for the Kalman Tracker */
bool tr_handler(void);


/* @brief Signal that new data is present and a Kalman-Step can be performed */
bool tr_newdata(uint16_t new_dist_mat[],uint16_t heading);


/* @brief Enable the use of the Kalman Tracker by QGroundControl */
bool kt_enable(uint8_t status);


/* @brief Get the state of the Kalman Tracker (enabled/disables) */
bool kt_get_state(void);


/* @brief Get the Obstacles of the Kalman Tracker in global NED-Frame */
uint16_t kt_get_obstacles(NEDpoint *array);


/* @brief Get the number of obstacles currently tracked */
uint16_t kt_get_nrofobstacles(void);




#endif /* KT_TRACKER_H_ */
