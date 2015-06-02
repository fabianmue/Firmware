/*
 * kt_topic_handler.h
 *
 *  Created on: 27.05.2015
 *      Author: Jonas
 */

#ifndef KT_TOPIC_HANDLER_H_
#define KT_TOPIC_HANDLER_H_

#include <stdint.h>
#include <stdbool.h>

/* @brief Init the use of topics */
bool th_init(void);


/* @brief Update the Topic */
bool th_update(void);


/* @brief Set the number of current tracks */
bool th_set_nroftracks(uint16_t tracknum);


/* @brief Set the number of newly added tracks in the last step */
bool th_set_nrofnewtracks(uint16_t refoundnum);


/* @brief Set the number of refound tracks in the last step */
bool th_set_nrofrefoundtracks(uint16_t refoundnum);


/* @brief Set the position of a measured obstacle */
bool th_set_obstacleposition(float px, float py, uint16_t num);


#endif /* KT_TOPIC_HANDLER_H_ */
