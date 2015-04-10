/*
 * sensorboard.h
 *
 *  Created on: 09.04.2015
 *      Author: Jonas
 */

#ifndef SENSORBOARD_H_
#define SENSORBOARD_H_

#include <stdbool.h>

/* Initialize the communciation with the Sensorboard */
bool sb_init(int *com_port);

/* Write a command to the COM-Port */
bool sb_write(const int *com_port, const uint8_t cmd);

/* Read from the COM-Port */
int sb_read(int *com_port);

/* Update the state */
bool sb_update_state(float heading);


#endif /* SENSORBOARD_H_ */
