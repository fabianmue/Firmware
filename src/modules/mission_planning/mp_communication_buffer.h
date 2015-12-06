/*
 * mp_communication_buffer.h
 *
 *  Created on: 04.12.2015
 *      Author: Fabian
 */

#ifndef MP_COMMUNICATION_BUFFER_H_
#define MP_COMMUNICATION_BUFFER_H_

#include "mp_topics_handler.h"
#include "../path_planning/pp_send_msg_qgc.h"

/* @brief send new target position to the mission_planning topic */
bool cb_new_target(float lat, float lon, float north, float east);

/* @brief publish mission_planning module if it has been updated */
void cb_publish_mp_if_updated(void);

/* @brief init module */
void cb_init(void);

#endif /* MP_COMMUNICATION_BUFFER_H_ */
