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

#include "pp_topics_handler.h"


/** @brief Calculate a new optimal heading reference */
void nav_navigate(void);


/** @brief Listen to the helsman */
void nav_listen2helsman(const struct structs_topics_s *strs_p);


/** @brief Speak to the helsman */
void nav_speak2helsman(void);


/** @brief New position information is available */
void nav_position_update(const struct structs_topics_s *strs_p);


/** @brief New heading information is available */
void nav_heading_update(const struct structs_topics_s *strs_p);


#endif /* PP_NAVIGATOR_H_ */
