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
bool sb_init(void);

/* Write a command to the COM-Port */
bool sb_write(const uint8_t cmd);

/* Read from the COM-Port */
int sb_read(void);

/* Check, if new data is available */
uint8_t sb_is_new_data(void);

/* Read the data from the buffer */
bool sb_read_data(uint8_t *buffer);

/* Regularely called handler-function */
bool sb_handler(void);


#endif /* SENSORBOARD_H_ */
