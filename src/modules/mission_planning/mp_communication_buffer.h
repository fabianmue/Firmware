/*
 * mp_communication_buffer.h
 *
 *  Created on: 04.12.2015
 *      Author: Fabian
 */

#ifndef MP_COMMUNICATION_BUFFER_H_
#define MP_COMMUNICATION_BUFFER_H_

#include "mp_topics_handler.h"

/* @brief send new mission_id */
bool mp_cb_new_mission_id(int id);

/* @brief send new target position */
bool mp_cb_new_waypoint(float wp_lat, float wp_lon);

/* @brief increase obstacle number */
bool mp_cb_new_obstacle(float ob_lat, float ob_lon, float ob_rad);

/* @brief publish mission_planning module if it has been updated */
void mp_cb_publish_if_updated(void);

/* @brief init module */
void mp_cb_init(void);

#endif /* MP_COMMUNICATION_BUFFER_H_ */
