/*
 * mp_communication_buffer.h
 *
 *  Created on: 04.12.2015
 *      Author: Fabian
 */

#ifndef MP_COMMUNICATION_BUFFER_H_
#define MP_COMMUNICATION_BUFFER_H_

#include "mp_topics_handler.h"

/* @brief send new target position to the mission_planning topic */
bool mp_cb_new_target(float lat, float lon);

/* @brief update QGC obstacle number */
bool mp_cb_new_obstacle(void);

/* @brief publish mission_planning module if it has been updated */
void mp_cb_publish_if_updated(void);

/* @brief init module */
void mp_cb_init(void);

#endif /* MP_COMMUNICATION_BUFFER_H_ */
