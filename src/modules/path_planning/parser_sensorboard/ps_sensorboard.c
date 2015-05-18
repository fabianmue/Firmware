/*
 * sensorboard.c
 *
 * Implements the use of the Sensorboard
 * Specifies the communication and the protocol
 *
 *  Created on: 09.04.2015
 *      Author: Jonas Wirz <wirzjo@student.ethz.ch>
 */

#include <stdbool.h>
#include <stdio.h>
#include "ps_sensorboard.h"
//#include "ps_data_update.h"

//for baud rate selection of the UART port:
#include <termios.h>
#include <sys/types.h>

// to open a UART port:
#include <sys/stat.h>
#include <fcntl.h>

#include <systemlib/err.h>
#include <drivers/drv_hrt.h>

#include "../pp_config.h"


/***********************************************************************************/
/*****  V A R I A B L E S  *********************************************************/
/***********************************************************************************/

/* Buffer for data from UART */
static char com_buffer[500];

static struct {			//Struct holding the state of the ps_sensorboard
	float heading;		//Heading of the boat [rad]
	uint8_t cmd; 		//Last received command
	uint8_t nrofbytes; 	//Number of data bytes that will be sent in the current message
	uint8_t dataindex; 	//Current index of the data byte to be read
	uint8_t data[255]; 	//Data bytes received with the last command
	bool newdata; 		//Flag that signals new data (true, if new data is present, false else)
	uint64_t last_call; //Time, when the Sensorboard was queried for new data the last time
} state = {
	.heading = 0,
	.cmd = 0x00,
	.nrofbytes = 0x00,
	.dataindex = 0x00,
	.newdata = false,
	.last_call = 0
};

static struct {
	uint64_t period;	//Period with which the Sensorboard is queried for new data
} config = {
	.period = 2e6 		//in us => 2s
};

typedef enum{IDLE,STARTCHAR, COMMAND, NUMOFBYTES, DATA, ENDCHAR} state_enum;
static state_enum rx_state = IDLE;  //State for the receive-finite state machine

static int comport = -1;		//COM-Port Object



/***********************************************************************************/
/*****  F U N C T I O N   P R O T O T Y P E S **************************************/
/***********************************************************************************/

/* @brief Set baudrate for communication */
bool sb_set_baudrate(int baudrate);

/* @brief Parse a received message */
bool parse_message(uint8_t data);



/***********************************************************************************/
/*****  P R O T O C O L    D E F I N I T I O N  ************************************/
/***********************************************************************************/

#define MSG_START		0x02	//Start character for a message
#define MSG_END			0x03	//End character for a message


#define CMD_OBSTACLES	0x4F	//Send the bearings and distances to every obstacle in range
								//Note: bearing (high/low byte) and then the distance is sent
#define CMD_NUMOBSTACLES 0x4E   //Number of obstacles currently in range
#define CMD_LASTDIST    0x4A    //Latest known distance from the LIDAR
#define CMD_DISTMAT1    0x4B    //Return the distance Matrix for 0-179
#define CMD_DISTMAT2    0x4C    //Return the distance Matrix for 180-355
#define CMD_RESET       0x20    //Reset the Sensor to initial conditions



/***********************************************************************************/
/*****  P A R A M E T E R S  *******************************************************/
/***********************************************************************************/

#define STEPSIZE 2 					//Stepsize of the LIDAR between two distance measurements [°]




/***********************************************************************************/
/*****  P U B L I C    F U N C T I O N S  ******************************************/
/***********************************************************************************/

/*
 * Init the Sensorboard => establish communication and open serial interface
 *
 * @param wx_port_pointer: Pointer to the COM-port handler
 */
bool sb_init(void) {

	comport = -1;

	//Open the COM-Port
	//Note: /dev/ttyS6 is the UART8 <=> SERIAL4 of the Pixhawk. Further details can be found on https://pixhawk.org/users/wiring
	printf("Try to open COM4...");
	comport = open("/dev/ttyS6", O_RDWR);	//We want to use Serial4 for the connection
	printf("...COM4 is open!\n");

	/* The port could not be opened due to some reasons. Nothing we can do about it! */
	if (comport < 0) {
		errx(1, "failed to open port: /dev/ttyS6");
	    return false;
	}

	//Wait before sending commands
	sleep(2); //wait 2s

	//Set baudrate
	sb_set_baudrate(38400);

	//Wait for stability
	sleep(2); //wait 2s

	//Initialize Receive State-Machine
	rx_state = IDLE;

	//Initialize State Variables
	state.heading = 0x00;
	state.cmd = 0x00;
	state.nrofbytes = 0x00;
	state.dataindex = 0x00;

	for (uint8_t i=0; i<255; i++){
		state.data[i] = 0;
	}
	state.newdata = false;

	//Everything is OK and we can return true
	return true;
}



/*
 * Write a command to the Sensorboard using the serial interface
 *
 * Note: The message always starts with 0x02 (MSG_START), 0x02 and ends with 0x03 (MSG_END)
 *
 * @param command as a uint8_t
 */
bool sb_write(const uint8_t cmd) {

	//Convert Heading from radian to degrees
	uint16_t head = RAD2DEG * state.heading;

	//Split up Heading into two uint8_t values
	uint8_t head0 = (uint8_t)(head>>8);
	uint8_t head1 = (uint8_t)head;

	//Create the String that has to be sent
	uint8_t msg[6] = {MSG_START, MSG_START, cmd, head0, head1, MSG_END};

	//Send the message to the Sensorboard
	write(comport, &msg, 6);

	return true;
}


/*
 * Read from the Sensorboard using the serial interface
 *
 * @param COM-Port object to be read from
 * @return number of bytes read from the buffer
 */
int sb_read(void) {

	int size = 0;

	//Read the data
	size = read(comport,com_buffer,sizeof(com_buffer));

	if(size > 0) {
		//The buffer contains some data => call the parser and try to find the message

		//printf("***Received data: %d \n",(int)com_buffer[0]);

		//Parse the buffer starting at the first element
		uint8_t i;
		for(i=0; i<size; i++) {
			parse_message(com_buffer[i]);
		}

		//Something could go wrong while receiving the data => reset the state machine
		rx_state = IDLE;

		//printf("...End of Message (%d)\n",size);
	} else {
		printf("   No data received!\n");
	}

	return size;
}


/**
 * Set the baudrate for the Serial communication
 *
 * @param baudrate: Baudrate to be used for communication
 */
bool sb_set_baudrate(int baudrate) {
	struct termios wx_port_config;
	tcgetattr(comport, &wx_port_config);
	int in_return;
	int out_return;

	in_return = cfsetispeed(&wx_port_config, baudrate);
	out_return = cfsetospeed(&wx_port_config, baudrate);

	if(in_return == -1 || out_return == -1){
		//error
		errx(1, "failed to set speed of: /dev/ttyS6");
		return false;
	}
	tcsetattr(comport, TCSANOW, &wx_port_config); // Set the new configuration

	return true;
}


/*
 * Update the internal state
 *
 * @param heading: Heading of the boat wrt. true North [rad]
 */
bool sb_update_state(float heading) {
	state.heading = heading;

	return true;
}


/*
 * Is new data available?
 *
 * @return last command numbe, 0x00 if no new data is present
 */
uint8_t sb_is_new_data(void) {

	if(state.newdata) {
		//New data is present => we return the command number

		return state.cmd;
	} else {
		//No new data is present => we return 0x00

		return 0x00;
	}
}


/*
 * Read new Data from Buffer and flag that the data is read
 *
 * @return
 */
bool sb_read_data(uint8_t *buffer) {

	//Return the data from the buffer
	if(state.newdata) {
		memcpy(buffer,state.data,state.nrofbytes);
		//uint8_t i;
		//for(i=0; i<state.nrofbytes; i++) {
		//	buffer[i] = state.data[i];
		//}
		//buffer[0] = state.data[0];
		//buffer[1] = state.data[1];
		return true;
	} else {
		return false;
	}
}


/**
 * Main handler function that is called in every loop iteration of the module
 */
bool sb_handler(void) {

	uint64_t systime = hrt_absolute_time();

	if((systime-state.last_call) >= config.period) {

		state.last_call = systime;

		//*** SEND A COMMAND HERE
		sb_write(CMD_DISTMAT1);		//Send the command to the Sensorboard for receiving the
		sb_read();					//Read and parse the data


		//*** READ DATA FROM THE SERIAL INTERFACE
		uint8_t cmd = sb_is_new_data();
		//printf("     -- Command: %d",cmd);

		if(cmd == 0x00) {
			//No new valid data is received => might flag unhappy

		} else {
			//New data is received => check, to which command the data belongs and act accordingly

			switch(cmd) {
				case CMD_LASTDIST: {
					//The last known distance from the sensor was received

					//printf("     -- Buffer 0,1: %d,%d",global_buffer[0],global_buffer[1]);

					uint16_t dist = 0;
					dist = (((uint16_t)state.data[0])<<8) | ((uint16_t)state.data[1]);

					printf("     -- New Distance: %d cm\n",dist);

					break;
				}
				case CMD_DISTMAT1: {
					//Get the first half of the distance Matrix 0-179° in Steps of 2°

					printf("     -- New Distance Matrix received!");

					uint16_t numOfBytes = 180/STEPSIZE * 2;

					uint16_t i;
					for(i=0; i < numOfBytes; i++) {
						uint16_t ind1 = 2*i;
						uint16_t ind2 = 2*i+1;
						uint16_t dist = (((uint16_t)state.data[ind1])<<8) | ((uint16_t)state.data[ind2]);

						printf("Dist %d: %d\n",i,dist);
					}

					break;
				}
				default: {
					//Undesirable, but nothing we can do about it

					return false;
				}

			} //end of switch(cmd)
		} //end of if(valid command received)
	} //end of if(time)

	//Everything seems ok and we return "true"
	return true;

}


/***********************************************************************************/
/*****  P R I V A T E    F U N C T I O N S  ****************************************/
/***********************************************************************************/

/**
 * Parse a message received from the Sensorboard
 *
 * @param byte: Byte just received
 */
bool parse_message(uint8_t data) {

		//Message starts with STX = 0x02 and ends with ETX = 0x03
		//A message from the Sensorboard must have the following form:
		// 0x02 | 0x02 | 0xXX (Command byte) | 0x03

		switch(rx_state) {
			case IDLE: {
				//The state machine is idle and waits for chars to be sent

				if(data == MSG_START) {
					//We received the first Start-Character

					rx_state = STARTCHAR;
				}

				break;
			}
			case STARTCHAR: {
				//The first Start-Character was sent and we are waiting for the second one now

				if(data == MSG_START) {
					//We received the second Start-Character

					rx_state = COMMAND;

				} else {
					//No second Start-Character was sent => return to IDLE

					rx_state = IDLE;
				}

				break;
			}
			case COMMAND: {
				//The second Start-Character was sent, now we expect to receive the Command

				if(data == MSG_START || data == MSG_END) {
					//We received again a Start or End Character or a 0 => ERROR
					//return to IDLE

					rx_state = IDLE;
				} else {
					//The char is valid => store it

					state.cmd = data;

					//printf("   Received Command: %d\n",data);

					rx_state = NUMOFBYTES;
				}

				break;
			}
			case NUMOFBYTES: {
				//The command was sent => expect to receive the number of data bytes

				state.nrofbytes = data;

				printf("   Received Num Bytes: %d\n",state.nrofbytes);

				state.dataindex = 0;
				rx_state = DATA;

				break;
			}
			case DATA: {
				//The number of Bytes to be read was received => expect to read this number of bytes

				//Read the Data byte and store it
				state.data[state.dataindex] = data;

				//printf("   Read byte number %d\n",state.dataindex);

				//Increment the index
				state.dataindex++;

				if(state.dataindex > (state.nrofbytes-1)) {
					//All data Bytes are read => expect to read the ENDCHAR

					rx_state = ENDCHAR;
				}

				break;
			}
			case ENDCHAR: {
				//All data bytes received

				if(data == MSG_END) {
					//We received the End Character => Data is valid

					//Flag that new data is available
					state.newdata = true;

					//printf("   Received ENDCHAR\n");

					rx_state = IDLE;

				} else {
					//Some error occurred => return to IDLE

					state.newdata = false;

					rx_state = IDLE;
				}

				break;
			}
			default: {
				//This code should never be reached
				//Nothing we can do about, if we reach it...just go back to IDLE

				rx_state = IDLE;

				break;
			}
		}

		//Everything is OK => return true
		return true;

}





