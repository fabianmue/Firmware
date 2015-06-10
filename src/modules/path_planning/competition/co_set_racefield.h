/*
 * kt_cog_list.h
 *
 *  Created on: 25.05.2015
 *      Author: Jonas
 */

#ifndef CO_SET_RACEFIELD_H_
#define CO_SET_RACEFIELD_H_


/* @brief Set the configuration variables */
void co_set_configuration(float dist, float o1x, float o1y, float rotation);


/* @brief Set the new competition task */
bool co_set_new_task(uint8_t tasknum, uint8_t state);




#endif /* CO_SET_RACEFIELD_H_ */
