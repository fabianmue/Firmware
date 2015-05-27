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


#endif /* KT_TOPIC_HANDLER_H_ */
