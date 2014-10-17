/****************************************************************************
 *
 *   Copyright (c) 2012, 2013 PX4 Development Team. All rights reserved.
 *   Author: Marco Tranzatto <marco.tranzatto@gmail.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file parser_200WX.c
 *
 * Definition of device driver (for a .h file) / Implementation of device driver (for a .c file).
 *
 * Note that the above line has to end with a dot.
 * Some more details about the driver, reference links / other C-code parts of PX4.
 *
 * @author Marco Tranzatto <marco.tranzatto@gmail.com>
 */

#include <nuttx/config.h>
#include <nuttx/sched.h>
#include <unistd.h>
#include <stdio.h>
#include <string.h>

#include "../autonomous_sailing/as_settings.h"

 // for baud rate selection of the UART port:
#include <termios.h>
#include <sys/types.h>

// for uORB topics
#include <uORB/uORB.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/vehicle_attitude.h>


// to open a UART port:
#include <sys/stat.h>
#include <fcntl.h>

#include <systemlib/systemlib.h>
#include <systemlib/err.h>


//Thread management variables
static bool thread_should_exit = false;		/**< daemon exit flag */
static bool thread_running = false;			/**< daemon status flag */
static int daemon_task;						/**< Handle of daemon task / thread */

/**
 * daemon management function for indoor usage.
 *
 */
__EXPORT int parser_200WX_main(int argc, char *argv[]);

/**
 * mainloop of daemon.
 *
 * if TYPE_OF_ENVIRONMENT (located in ../autonomous_sailing/as_settings.h) is 0, then use indoor parser
 */
int parser_200WX_indoor_daemon_thread_main(int argc, char *argv[]);

/**
* initialize weather station 200WX.
*
* disable all the default messages from the station and after that enable only the messages in which
* we are interestin in.
*
* @param 				pointer com port file descriptor
* @return 				true on success
*/
//bool weather_station_init(int *wx_port_point);


/**
* set baud rate between weather station 200WX and pixhawk.
*
* @param wx_port	name of the UART port
* @param baudrate 	baudrate of the communication
*/
//bool pixhawk_baudrate_set(int wx_port, int baudrate);

/**
* initializes all the variables used in the indoor version of the parser
*
* @param wx_port_pointer		pointer to COM file descriptor
* @param sensor_sub_fd_pointer	pointer to sensor combined topic
* @param att_pub_fd_pointer		pointer to handler returnd by orb_advertise
* @param airs_pub_fd_pointer	pointer to handler returnd by orb_advertise
* @return 						true is evrything is ok
*/
/*bool indoor_variables_init(int *wx_port_pointer,
	                       int *sensor_sub_fd_pointer,
						   int *att_pub_fd_pointer,
						   int *airs_pub_fd_pointer);*/

/**
* retrieve indoor data(by readings from UART) when pool() returns correctly.
*
* @param wx_port_pointer		pointer to COM file descriptor
* @param sensor_sub_fd_pointer	pointer to sensor combined topic
* @param att_pub_fd_pointer		pointer to handler returnd by orb_advertise
* @param airs_pub_fd_pointer	pointer to handler returnd by orb_advertise
* @return 						true is evrything is ok
*/
/*bool retrieve_indoor_data(int *wx_port_pointer,
	                      int *sensor_sub_fd_pointer,
						  int *att_pub_fd_pointer,
						  int *airs_pub_fd_pointer);*/

/**
* parse transducer data received from 200WX
*
* @param index_pointer			pointer to the current position analyzed in the buffer
* @param raw_buffer_pointer		pointer to raw data buffer read from UART previously
* @param att_pub_fd_pointer		pointer to handler returnd by orb_advertise
* @return 						true is evrything is ok
*/
//bool parse_transducer_message(int *index_pointer, char *raw_buffer_pointer, int *att_pub_fd_pointer);

/**
* finds if string is in buffer starting from index 
*
* @return 	true if string is found, false otherwise
*/
//bool find_string(int index, char *buffer, char *string);

/**
* extracts data from buffer, starting from index, until a come is found. updates index
*
* @return 	the value read and converted through atof()
*/
//double extract_until_coma(int *index_pointer, char *buffer);

/**
 * print the correct usage.
 */
static void usage(const char *reason);

static void usage(const char *reason)
{
	if (reason)
		warnx("%s\n", reason);
	errx(1, "usage: daemon {start|stop|status} [-p <additional params>]\n\n");
}

/**
 * The daemon app only briefly exists to start
 * the background job. The stack size assigned in the
 * Makefile does only apply to this management task.
 *
 * The actual stack size should be set in the call
 * to task_create().
 */
int parser_200WX_main(int argc, char *argv[])
{
	if (argc < 1)
		usage("missing command");

	if (!strcmp(argv[1], "start")) {

		if (thread_running) {
			warnx("daemon already running\n");
			/* this is not an error */
			exit(0);
		}

		thread_should_exit = false;

		if(TYPE_OF_ENVIRONMENT == 0){
			//indoor parser
			daemon_task = task_spawn_cmd("daemon",
										SCHED_DEFAULT,
										SCHED_PRIORITY_MAX,	
										4096,
										parser_200WX_indoor_daemon_thread_main,
					 	(argv) ? (const char **)&argv[2] : (const char **)NULL);

			
		}else{
			//outdoor parser
			daemon_task = task_spawn_cmd("daemon",
										SCHED_DEFAULT,
										SCHED_PRIORITY_MAX,	
										4096,
										parser_200WX_indoor_daemon_thread_main,
					 	(argv) ? (const char **)&argv[2] : (const char **)NULL);
		}

		exit(0);
	}

	if (!strcmp(argv[1], "stop")) {
		thread_should_exit = true;
		exit(0);
	}

	if (!strcmp(argv[1], "status")) {
		if (thread_running) {
			warnx("\trunning\n");
		} else {
			warnx("\tnot started\n");
		}
		exit(0);
	}

	usage("unrecognized command");
	exit(1);
}


/**
* Main thread of this indoor parser.
*
* DESCRIVERE FUNZIONAMENTO
*/
int parser_200WX_indoor_daemon_thread_main(int argc, char *argv[]) {

	warnx("[parser_200WX_indoor] starting\n");

	thread_running = true;
/*
	// Indoor variables

	// file descriptor to read and write on com port
	int wx_port;
	// handler for sensor_combined topic
	int sensor_sub_fd;
	// file descriptor for attitude topic
	int att_pub_fd;
	// file descriptor for airspeed topic
	int airs_pub_fd;
	//pool return value
	int poll_ret;

	// initialize all the indoor variables
	indoor_variables_init(&wx_port, 	&sensor_sub_fd,
						  &att_pub_fd, 	&airs_pub_fd);

	// polling management
	struct pollfd fds[] = {
			{ .fd = sensor_sub_fd,   .events = POLLIN },
	};*/

	while (!thread_should_exit) {
		/*// wait for sensor update of 1 file descriptor up to 1000 ms (1 sec = 1 Hz)
		poll_ret = poll(fds, 1, 1000);	

		// handle the poll result 
		if (poll_ret == 0) {
			// this means none of our providers is giving us data 
			warnx("[parser_200WX_indoor] Got no data within a second\n");
		}
		else{
			if (poll_ret < 0) {
				// this is seriously bad - should be an emergency
				warnx("[parser_200WX_indoor] Terrible error!\n");
			}
			else{
				// evrything is ok, at least so far (i.e. pool_ret > 0)
				if (fds[0].revents & POLLIN) {	

					// read UART and retrieve indoor data 
					bool ret_retr = retrieve_indoor_data(&wx_port, 	&sensor_sub_fd,
						  				 				 &att_pub_fd, 	&airs_pub_fd);

					if (ret_retr){

					}
				
				}
			}
		}*/
		sleep(20);
	}

	warnx("[parser_200WX_indoor] exiting.\n");

	thread_running = false;

	return 0;

}

bool find_string(int index, char *buffer, char *string){

	int str_len = strlen(string);

	for(int i=0;i<str_len;i++){
		if(((char)buffer[index + i] != string[i]))
			return false;

	}

	return true;
}
/*
double extract_until_coma(int *index_pointer, char *buffer){

	int counter = 0;
	char temp_char[10];	

	while(buffer[*index_pointer] != ','){ 
					
		temp_char[counter] = buffer[i];

		*index_pointer = (*index_pointer) + 1;
		counter++;

		// da mettere ?!
		//if(counter>11){
		//	PARSING_ERR = true;
		//	break; // safety
		//}
	}
	// make *index_pointer be the ','
	*index_pointer = (*index_pointer) + 1;

	return atof(temp_char);
}*/
/*

bool weather_station_init(int wx_port_pointer){


	*wx_port_pointer = open("/dev/ttyS5", O_RDWR); // Serial 5, read works, write works
	// This is serial port 4 according to: pixhawk.org/dev/wiring
	if (*wx_port_pointer < 0) {
	        errx(1, "failed to open port: /dev/ttyS5");
	       // goto exiting;
	    }
	warnx("[as_wx_init] UART port open.\n");

	// Set baud rate of wx_port to 4800 baud
	as_pixhawk_baudrate_set(*wx_port_pointer, 4800);
	warnx("[as_wx_init] baud rate of UART port set to 4800.\n");

	// wait 5 [seconds] for the WX to power up before sending commands (SYS 2999)
	sleep(5); 

	// start with a new line:
	uint8_t new_line[] = {'\n'};				
	write(*wx_port_pointer, new_line, sizeof(new_line));

	// stop transmitting
	uint8_t stop_tx[] = {'$', 'P', 'A', 'M', 'T', 'X', '\r', '\n'};			
	write(*wx_port_pointer, stop_tx, sizeof(stop_tx));
	write(*wx_port_pointer, stop_tx, sizeof(stop_tx));
	write(*wx_port_pointer, stop_tx, sizeof(stop_tx)); // Marine talk, send everything 3 times

	// wait for 2 seconds for stability
	sleep(2); 

	// Disable all the transmitted sentences, so that we can tell the weather station exactly what to send:
	uint8_t disable_tx[] = {'$', 'P', 'A', 'M', 'T', 'C', ',', 'E', 'N', ',', 'A', 'L', 'L', ',', '0', ',', '1', '0', '\r', '\n'}; 
	write(*wx_port_pointer, disable_tx, sizeof(disable_tx));
	write(*wx_port_pointer, disable_tx, sizeof(disable_tx));
	write(*wx_port_pointer, disable_tx, sizeof(disable_tx));

	// wait for 2 seconds for stability
	sleep(2); 

	// enable relative wind speed measurement
	uint8_t enable_wind[] = {'$', 'P', 'A', 'M', 'T', 'C', ',', 'E', 'N', ',', 'M', 'W', 'V', 'R', ',', '1', ',', '1', '\r', '\n'}; 
	write(*wx_port_pointer, enable_wind, sizeof(enable_wind));
	write(*wx_port_pointer, enable_wind, sizeof(enable_wind));
	write(*wx_port_pointer, enable_wind, sizeof(enable_wind));

	// enable vessel attitude (pitch and roll)
	uint8_t enable_attitude[] = {'$','P','A','M','T','C',',','E','N',',','X','D','R','B',',','1',',','1', '\r', '\n'};  
	write(*wx_port_pointer, enable_attitude, sizeof(enable_attitude));
	write(*wx_port_pointer, enable_attitude, sizeof(enable_attitude));
	write(*wx_port_pointer, enable_attitude, sizeof(enable_attitude));

	// enable Roll, Pitch, Yaw rate relative to the vessel frame
	uint8_t enable_RPY_rate[] = {'$','P','A','M','T','C',',','E','N',',','X','D','R','E',',','1',',','1', '\r', '\n'};  
	write(*wx_port_pointer, enable_RPY_rate, sizeof(enable_RPY_rate));
	write(*wx_port_pointer, enable_RPY_rate, sizeof(enable_RPY_rate));
	write(*wx_port_pointer, enable_RPY_rate, sizeof(enable_RPY_rate));

	// enable x, y, z accelerometer readings
	uint8_t enable_IMU[] = {'$','P','A','M','T','C',',','E','N',',','X','D','R','C',',','1',',','1', '\r', '\n'};  
	write(*wx_port_pointer, enable_IMU, sizeof(enable_IMU));
	write(*wx_port_pointer, enable_IMU, sizeof(enable_IMU));
	write(*wx_port_pointer, enable_IMU, sizeof(enable_IMU));

	// switch to 38400 baud (the highest possible baud rate):
	uint8_t high_baud[] = {'$', 'P', 'A', 'M', 'T', 'C', ',', 'B', 'A', 'U', 'D', ',', '3', '8', '4', '0', '0', '\r', '\n'}; 
	write(*wx_port_pointer, high_baud, sizeof(high_baud));
	write(*wx_port_pointer, high_baud, sizeof(high_baud));

	// wait for 2 seconds for stability
	sleep(2); 

	// switch the pixhawk baudrate to 38400
	pixhawk_baudrate_set(*wx_port_pointer, 38400); 

	// tell the weather station to start transmitting again (now at 38400 baud):
	uint8_t start_tx[] = {'$', 'P', 'A', 'M', 'T', 'X', ',', '1', '\r', '\n'}; 
	write(*wx_port_pointer, start_tx, sizeof(start_tx));
	write(*wx_port_pointer, start_tx, sizeof(start_tx));
	write(*wx_port_pointer, start_tx, sizeof(start_tx));

	// erase received but not read yet data from serial buffer 
	for (int i=0; i<4; i++)
		read(*wx_port_pointer, &raw_buffer, sizeof(raw_buffer));
	sleep(0.5);		// collect enough data for first parsing
	warnx("[as_wx_init] serial buffer cleaned.\n");

	return true;
}*/
/*
bool pixhawk_baudrate_set(int wx_port, int baudrate){		// Set the baud rate of the pixhawk to baudrate:
	struct termios wx_port_config;
	tcgetattr(wx_port, &wx_port_config);
	int in_return;
	int out_return;

	in_return = cfsetispeed(&wx_port_config, baudrate);
	out_return = cfsetospeed(&wx_port_config, baudrate);
	//printf("_in returned: %i/n", in_return);			// Debug: both return zero -> good!
	//printf("_out returned: %i/n", out_return);
	tcsetattr(wx_port, TCSANOW, &wx_port_config); // Set the new configuration

	return true;
}*/
/*
bool indoor_variables_init(int *wx_port_pointer, int *sensor_sub_fd_pointer,
						   int *att_pub_fd_pointer,
						   int *airs_pub_fd_pointer){

	// open COM port to talk with wather 200WX station
	weather_station_init(wx_port_pointer);

	// subscribe to sensor_combined topic
	*sensor_sub_fd_pointer = orb_subscribe(ORB_ID(sensor_combined));
	//orb_set_interval(*sensor_sub_fd_pointer, 120);	// set px4 sensors update every 0.12 [second] = 8.3 Hz
	orb_set_interval(*sensor_sub_fd_pointer, 110);	// set px4 sensors update every 0.11 [second] = 9.1 Hz
	//orb_set_interval(*sensor_sub_fd_pointer, 100);	// set px4 sensors update every 0.10 [second] = 10 Hz
	//orb_set_interval(*sensor_sub_fd_pointer, 80);	// set px4 sensors update every 0.08 [second] = 12 Hz

	// advertise attitude topic (ATT)
	struct vehicle_attitude_s att;
	memset(&att, 0, sizeof(att));
	*att_pub_fd_pointer = orb_advertise(ORB_ID(vehicle_attitude), &att);

	// advertise airspeed topic (AIRS) 
	struct airspeed_s raw_air_vel;
	memset(&raw_air_vel, 0, sizeof(raw_air_vel));
	*airs_pub_fd_pointer = orb_advertise(ORB_ID(airspeed), &raw_air_vel);
}*/

/*
bool retrieve_indoor_data(int *wx_port_pointer, 
	                      int *sensor_sub_fd_pointer,
						  int *att_pub_fd_pointer,
						  int *airs_pub_fd_pointer){

	struct sensor_combined_s sensor_combined_raw;
	int buffer_length;
	char raw_buffer[250];
	int index;

	// copy sensors raw data into local buffer, actually this action is need only to apply downsampling time
	orb_copy(ORB_ID(sensor_combined), sensor_sub_fd, &sensor_combined_raw);	

	// read UART when px4 sensors are updated
	buffer_length = read(*wx_port_pointer, raw_buffer, sizeof(raw_buffer));

	 if(buffer_length < 1)
	 	return false;

	// start by looking for the start of the NMEA sentence:
	for (index=0; index<buffer_length; index++) {
		// it's worthless to check if there won't be enough data anyway..	
  		if (buffer_length - i > 50) {	
			parse_attitude_data(&index, raw_buffer, att_pub_fd_pointer); 
		}
	}

}*/
/*
bool parse_transducer_message(int *index_pointer, char *raw_buffer_pointer, int *att_pub_fd_pointer){

	struct vehicle_attitude_s att;
	double temp_val;

	att.timestamp = hrt_absolute_time();

	// see if we received a YXXDR command 
	if(find_string(*index_pointer, raw_buffer_pointer, "YXXDR")){

		// index+5 is the comma ','
		// index+6 is 'A' to symbolize angular displacement
		// index+7 is the comma ','

		(*index_pointer) += 8;	// position to first digit of first value

		//extract value from buffer
		temp_val = extract_until_coma(index_pointer, raw_buffer_pointer);

		(*index_pointer) += 1;	// position *index_pointer to next position after ','

		if (raw_buffer_pointer[*index_pointer] == 'D'){	
			// then it means we are parsing either attitude or RPY rate
			(*index_pointer) += 2;//we are after the ',' that is after 'D'

			if(find_string(index_pointer, raw_buffer_pointer, "PTCH")){
				// we are parsing YXXDR command type B
				att.pitch = temp_val;

				// index+1 is the comma ','
				// index+2 is 'A' to symbolize angular displacement
				// index+3 is the comma ','

				(*index_pointer) += 4;	// position to first digit of roll value

				att.roll = extract_until_coma(index_pointer, raw_buffer_pointer);
			}
			else{
				// we are parsing YXXDR command type E
				att.rollspeed = temp_val;

				// i+3 is 'R' |
				// i+4 is 'R' |
				// i+5 is 'T' |
				// i+6 is 'R' |--> ID indicating roll rate of vessel
				// i+7 is the comma ','
				// i+8 is 'A' to symbolize angular displacement
				// i+9 is the comma ','

				i += 10;	// position to first digit of pitch rate
			}
		}
	}
}*/
