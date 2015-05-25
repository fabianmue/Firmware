/*
 * kt_tracker.h
 *
 *  Created on: 22.05.2015
 *      Author: Jonas Wirz <wirzjo@student.ethz.ch>
 */

#ifndef KT_TRACKER_H_
#define KT_TRACKER_H_

#include "../pp_config.h"

/* @brief Init the use of the Kalman Tracker */
bool tr_init(void);


/* @brief Hanlder funciton for the Kalman Tracker */
bool tr_handler(void);


/* @brief Signal that new data is present and a Kalman-Step can be performed */
bool tr_newdata(uint16_t new_dist_mat[],uint16_t heading);


#endif /* KT_TRACKER_H_ */
