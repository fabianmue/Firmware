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

//for baud rate selection of the UART port:
#include <termios.h>
#include <sys/types.h>

// to open a UART port:
#include <sys/stat.h>
#include <fcntl.h>


/* Buffer for data from UART */
static char com_buffer[500];


/* @brief Set baudrate for communication */
bool sb_set_baudrate(int com_port, int baudrate);




/*
 * Init the Sensorboard => establish communication and open serial interface
 *
 * @param wx_port_pointer: Pointer to the COM-port handler
 */
bool sb_init(int *com_port) {
	*com_port = open("/dev/ttyS4", O_RDWR);	//We want to use Serial4 for the connection

	/* The port could not be opened due to some reasons */
	if (*com_port < 0) {
		errx(1, "failed to open port: /dev/ttyS4");
	    return false;
	}

	//Wait before sending commands
	sleep(5); //wait 5s

	//Set baudrate
	sb_set_baudrate(*com_port,9600);

	//Wait for stability
	sleep(2); //wait 2s

	//Everything is OK and we can return true
	return true;
}



/*
 * Write a command to the Sensorboard using the serial interface
 *
 * @param command as a uint8_t
 */
bool sb_write(int *com_port) {
	//int nr_of_bytes = write(*com_port,0x02,1);

	return true;
}


/*
 * Read from the Sensorboard using the serial interface
 *
 * @param buffer: Pointer to the buffer containing the read data
 * @return number of bytes read from the buffer
 */
int sb_read(int *com_port) {

	int size = 0;

	//Read the data
	size = read(*com_port,com_buffer,sizeof(com_buffer));

	if(size > 0) {
		printf("***Received data\n");
	} else {
		printf("   No data received!\n");
	}

	return size;
}


bool sb_set_baudrate(int com_port, int baudrate) {
	struct termios wx_port_config;
	tcgetattr(com_port, &wx_port_config);
	int in_return;
	int out_return;

	in_return = cfsetispeed(&wx_port_config, baudrate);
	out_return = cfsetospeed(&wx_port_config, baudrate);

	if(in_return == -1 || out_return == -1){
		//error
		errx(1, "failed to set speed of: /dev/ttyS4");
		return false;
	}
	tcsetattr(com_port, TCSANOW, &wx_port_config); // Set the new configuration

	return true;
}





