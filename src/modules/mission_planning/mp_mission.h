/*
 * mp_mission.h
 *
 *  Created on: 25.05.2015
 *      Author: Jonas
 *
 *      09.10.2015: moved from path_planning to mission_planning app (Fabian Müller)
 */

#ifndef MP_MISSION_H_
#define MP_MISSION_H_

#include "../pp_navigation_helper.h"

/* @brief set competition race-field */
void mi_init(void);

/* @brief set the configuration variables */
void mi_set_configuration(float dist, float o1x, float o1y, float rotation);

/* @brief start new task */
bool mi_start_task(uint8_t task_type);

/* @brief stop current task */
bool mi_stop_task(void);

/* @brief check if boat is within competition race-field */
bool mi_isinside(NEDpoint boatpos);

#endif /* MP_MISSION_H_ */

/*	@brief Handler Function for the missions as called by the main-program loop
 *  bool mi_handler(void);
 *
 */
