/*
 * ps_data_update.c
 *
 * Request the sensorboard for data and wait for the answer.
 * Then interprete the data and update the topic accordingly.
 *
 *  Created on: 15.04.2015
 *      Author: Jonas Wirz <wirzjo@student.ethz.ch>
 */


#include <stdbool.h>
#include <stdio.h>
#include <stdint.h>
#include "../pp_config.h"
#include "ps_data_update.h"
#include "ps_sensorboard.h"



/***********************************************************************************/
/*****  V A R I A B L E S  *********************************************************/
/***********************************************************************************/

static uint8_t global_buffer[255];		//global buffer holding the data received from the UART (actually only the pointer to the UART data buffer)

#define STEPSIZE 2 					//Stepsize of the LIDAR between two distance measurements [°]

/***********************************************************************************/
/*****  P R O T O C O L    D E F I N I T I O N  ************************************/
/***********************************************************************************/

#define CMD_OBSTACLES	0x4F	//Send the bearings and distances to every obstacle in range
								//Note: bearing (high/low byte) and then the distance is sent
#define CMD_NUMOBSTACLES 0x4E   //Number of obstacles currently in range
#define CMD_LASTDIST    0x4A    //Latest known distance from the LIDAR
#define CMD_DISTMAT1    0x4B    //Return the distance Matrix for 0-179
#define CMD_DISTMAT2    0x4C    //Return the distance Matrix for 180-355
#define CMD_RESET       0x20    //Reset the Sensor to initial conditions


/***********************************************************************************/
/*****  P U B L I C    F U N C T I O N S  ******************************************/
/***********************************************************************************/

/**
 * Check for new data and update the state accordingly
 * This function is called in every iteration of the main-loop and
 * executes repetitive tasks
 *
 */
bool du_handler(void) {


	//*** SEND A COMMAND
	sb_write(CMD_DISTMAT1);		//Send the command to the Sensorboard
	sb_read();					//Read and parse the data



	//*** READ DATA FROM THE SERIAL INTERFACE
	uint8_t cmd = sb_is_new_data();
	//printf("     -- Command: %d",cmd);

	if(cmd == 0x00) {
		//No new valid data is received => might flag unhappy

	} else {
		//New data is received => check, to which command the data belongs and act accordingly

		//Store the new data locally
		sb_read_data(global_buffer);

		switch(cmd) {
			case CMD_LASTDIST: {
				//The last known distance from the sensor was received

				//printf("     -- Buffer 0,1: %d,%d",global_buffer[0],global_buffer[1]);

				uint16_t dist = 0;
				dist = (((uint16_t)global_buffer[0])<<8) | ((uint16_t)global_buffer[1]);

				printf("     -- New Distance: %d cm\n",dist);

				break;
			}
			case CMD_DISTMAT1: {
				//Get the first half of the distance Matrix 0-179° in Steps of 2°

				/*uint16_t i;
				for(i=0; i < 180/STEPSIZE; i++) {
					uint16_t ind1 = i;
					uint16_t ind2 = i+1;
					uint16_t dist = (((uint16_t)global_buffer[ind1])<<8) | ((uint16_t)global_buffer[ind2]);

					printf("Dist %d: %d\n",i,dist);
				}*/

				break;
			}
			default: {
				//Undesirable, but nothing we can do about it

				return false;
			}
		}
	}

	//Everything is OK => return true
	return true;

}

