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
#include <stdlib.h>
#include <string.h>
#include <poll.h>

#include "../autonomous_sailing/as_settings.h"

 // for baud rate selection of the UART port:
#include <termios.h>
#include <sys/types.h>

// for uORB topics
#include "topics_handler.h"

//#include <uORB/uORB.h>
//#include <uORB/topics/sensor_combined.h>
//#include <uORB/topics/vehicle_attitude.h>
//#include <uORB/topics/airspeed.h>
//#include <uORB/topics/wind_sailing.h>
//#include <uORB/topics/vehicle_gps_position.h>
//#include <uORB/topics/vehicle_bodyframe_meas.h>
//#include <uORB/topics/debug_values.h>


// to open a UART port:
#include <sys/stat.h>
#include <fcntl.h>

#include <systemlib/systemlib.h>
#include <systemlib/err.h>



#define SAFETY_COUNTER_EXTRACT 15 ///if extract_until coma doesn't find a ',' for SAFETY_COUNTER_EXTRACT charachters, exit

#define MIN_BYTE_FOR_PARSING_LONG_MSG 40 ///minimum number of available byte for starting parsing a long message

#define MIN_BYTE_FOR_PARSING_MEDIUM_MSG 25 ///minimum number of available byte for starting parsing a medium message

#define MIN_BYTE_FOR_PARSING_SHORT_MSG 10 ///minimum number of available byte for starting parsing a short message

#define DAEMON_PRIORITY SCHED_PRIORITY_MAX - 10 ///daemon priority

#define deg2rad 0.017453292519f // pi / 180

#define km_h2m_s 0.277777777777778f // 1000 / 3600

//Global buffer for data from 200WX
char buffer_global[400];

//Thread management variables
static bool thread_should_exit = false;		/**< daemon exit flag */
static bool thread_running = false;			/**< daemon status flag */
static int daemon_task;						/**< Handle of daemon task / thread */


__EXPORT int parser_200WX_main(int argc, char *argv[]);

/** @brief main loop. */
int parser_200WX_daemon_thread_main(int argc, char *argv[]);

/** @brief Initialize weather station. */
bool weather_station_init(int *wx_port_point);

/** @brief Encode str in a message for 200WX.*/
void encode_msg_200WX(int *wx_port_point, const char *str);

/** @brief Send msg three times, marine talk. */
void send_three_times(const int *wx_port_pointer, const uint8_t *msg, const int length);

/** @brief Set baud rate. */
bool pixhawk_baudrate_set(int wx_port, int baudrate);

/** @brief Initialize all variables. */
bool parser_variables_init(int *wx_port_pointer,
                           struct subscribtion_fd_s  *subs_p,
                           struct published_fd_s     *pubs_p,
                           struct structs_topics_s   *strs_p);
//bool parser_variables_init(int *wx_port_pointer,
//                           int *sensor_sub_fd_pointer,
//                           int *att_pub_fd_pointer, struct vehicle_attitude_s *att_raw_pointer,
//                           int *airs_pub_fd_pointer, struct airspeed_s *air_vel_raw_pointer,
//                           int *gps_pub_fd_pointer, struct vehicle_gps_position_s  *gps_raw_pointer,
//                           int *wind_sailing_fd_pointer, struct wind_sailing_s *wind_sailing_raw_pointer,
//                           int *bodyframe_meas_fd_pointer, struct vehicle_bodyframe_meas_s *bodyframe_meas_raw_pointer);

/** @brief Extract data from stream with 200WX. */
bool retrieve_data(int *wx_port_pointer,
                   struct subscribtion_fd_s  *subs_p,
                   struct structs_topics_s   *strs_p);
//bool retrieve_data(int *wx_port_pointer,
//                  int *sensor_sub_fd_pointer,
//                  struct vehicle_attitude_s *att_raw_pointer,
//                  struct airspeed_s *air_vel_raw_pointer,
//                  struct vehicle_gps_position_s *gps_raw_pointer,
//                  struct wind_sailing_s *wind_sailing_pointer,
//                  struct vehicle_bodyframe_meas_s *bodyframe_meas_raw_pointer);

/** @brief Parser for YXXDR messages. */
void xdr_parser(const char *buffer, const int buffer_length,
                struct vehicle_attitude_s *att_raw_pointer,
                struct vehicle_bodyframe_meas_s *bodyframe_meas_raw_pointer);

/** @brief Parser for GPXXX messages. */
void gp_parser(const char *buffer, const int buffer_length, struct vehicle_gps_position_s *gps_raw_pointer);

/** @brief Convert nmea coordinates in degree. */
float nmea_ndeg2degree(float val);

/** @brief Parser for MWVR message. */
void vr_parser(const char *buffer, const int buffer_length, struct wind_sailing_s *wind_sailing_pointer);

/** @brief Parser for HCHDT message. */
void hdt_parser(const char *buffer, const int buffer_length, struct vehicle_attitude_s *att_raw_pointer);

/** @brief Parser for WIMWD message. */
void mwd_parser(const char *buffer, const int buffer_length, struct wind_sailing_s *wind_sailing_pointer);

/** @brief Find string everywhere in buffer. */
int find_string_everywhere(const int start_index, const char *buffer, const int buffer_length, const char *str);

/** @brief Check if the strin is in the buffer starting from an exact position. */
int find_string_here(const int start_index, const char *buffer, const int buffer_length, const char *str);

/** @brief Extract double from string. */
bool d_extract_until_coma(int *index_pointer, const char *buffer, const int buffer_length, double *ret_val_pointer);

/** @brief Extract float from string. */
bool f_extract_until_coma(int *index_pointer, const char *buffer, const int buffer_length, float *ret_val_pointer);

/** @brief Go to buffer until next ','. */
int jump_to_next_coma(const int start_index, const char *buffer, const int buffer_length);

/** @brief Print buffer on terminal */
void debug_print_nchar(const char *buffer, const int length, const int start, const int end);

/** @brief Print buffer on terminal */
void debug_print_until_char(const char *buffer, const int length, const int start, const char stop_char);

/** @brief Publish new data. */
void publish_new_data(struct published_fd_s     *pubs_p,
                      struct structs_topics_s   *strs_p);


static void usage(const char *reason);

/**
 * Print the correct usage.
 */
static void usage(const char *reason)
{
	if (reason)
		warnx("%s\n", reason);
    errx(1, "usage: parser_200WX {start|stop|status}\n\n");
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

        daemon_task = task_spawn_cmd("parser_200WX",
                                    SCHED_DEFAULT,
                                    DAEMON_PRIORITY,
                                    4096,
                                    parser_200WX_daemon_thread_main,
                    (argv) ? (const char **)&argv[2] : (const char **)NULL);

			


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
 * mainloop of daemon.
 *
 * if TYPE_OF_ENVIRONMENT (located in ../autonomous_sailing/as_settings.h) is 1 then activeted also outdoor messages
 */
int parser_200WX_daemon_thread_main(int argc, char *argv[]) {

    if(AS_TYPE_OF_ENVIRONMENT == 1)
        warnx(" starting Outdoor version\n");
    else
        if(AS_TYPE_OF_ENVIRONMENT == 0)
            warnx(" starting Indoor version\n");
        else{
            warnx(" ERROR, set 'AS_TYPE_OF_ENVIRONMENT' in autonomous_sailing/as_settings.h\n");
            thread_should_exit = true;
        }


	thread_running = true;

	// file descriptor to read and write on com port
	int wx_port;

    struct subscribtion_fd_s subs;
    struct published_fd_s pubs;
    struct structs_topics_s structs_topics;

    // file descriptor for sensor_combined topic
    /*int sensor_sub_fd;
    // file descriptor and struct for attitude topic
	int att_pub_fd;
    struct vehicle_attitude_s att_raw;
    // file descriptor and struct for airspeed topic
	int airs_pub_fd;
    struct airspeed_s air_vel_raw;
    // file descriptor and struct for vehicle_gps_position topic
    int vehicle_gps_fd;
    struct vehicle_gps_position_s  gps_raw;
    // file descriptor and struct for wind_sailing topic
    struct wind_sailing_s wind_sailing_raw;
    int wind_sailing_fd;
    // file descriptor and struct for vehicle_bodyframe_meas topic topic
    struct vehicle_bodyframe_meas_s bodyframe_meas_raw;
    int bodyframe_meas_fd;*/



	//pool return value
    int poll_ret;

	// initialize all the indoor variables
    parser_variables_init(&wx_port, &subs,
                          &pubs, &structs_topics);
//    parser_variables_init(&wx_port, 	&sensor_sub_fd,
//                          &att_pub_fd, 	&att_raw,
//                          &airs_pub_fd, &air_vel_raw,
//                          &vehicle_gps_fd, &gps_raw,
//                          &wind_sailing_fd, &wind_sailing_raw,
//                          &bodyframe_meas_fd, &bodyframe_meas_raw);

	// polling management
	struct pollfd fds[] = {
            { .fd = subs.sensor_sub,   .events = POLLIN }
    };


	while (!thread_should_exit) {
        // wait for sensor update of 1 file descriptor up to 1000 ms (1 sec = 1 Hz)
		poll_ret = poll(fds, 1, 1000);	

		// handle the poll result 
		if (poll_ret == 0) {
			// this means none of our providers is giving us data 
            warnx(" got no data within a second\n");
		}
		else{
			if (poll_ret < 0) {
				// this is seriously bad - should be an emergency
                warnx(" terrible error!\n");
			}
			else{
				// evrything is ok, at least so far (i.e. pool_ret > 0)
                if (fds[0].revents & POLLIN){

					// read UART and retrieve indoor data 
                    retrieve_data(&wx_port,
                                  &subs,
                                  &structs_topics);

                    publish_new_data(&pubs, &structs_topics);

				}
			}
        }
	}

    warnx(" exiting.\n");

	thread_running = false;

	return 0;

}

/**
* Find if string is in buffer starting from start_index and going til the end.
*
* @param start_index    index where start to find string
* @param buffer         buffer where search for string
* @param buffer_length  length of buffer
* @param str         string to find in buffer
* @return 	the index in the buffer where 'string' begins in the buffer, -1 if not found
*/
int find_string_everywhere(const int start_index, const char *buffer, const int buffer_length, const char *str){

    int i;
    int str_len = strlen(str);
    char temp_str[10]; /**< str cannot be greater than 9 charachter! */
    int stop_index = buffer_length - str_len +1;

    for(i = start_index; i < stop_index; i++){

        strncpy(temp_str, &buffer[i], str_len);
        //add null-characther at the end
        temp_str[str_len] = '\0';

        if(!strcmp(temp_str, str)){
            //found str in buffer, starting from i
            return i;
        }
    }

    //str not found in buffer
    return -1;
}

/**
* Find if string is in buffer starting exatcly from start_index.
*
* @param start_index    index where start to find string
* @param buffer         buffer where search for string
* @param buffer_length  length of buffer
* @param str         string to find in buffer
* @return 	the index in the buffer where 'string' begins in the buffer ret == start_index on succes, -1 if not found
*/
int find_string_here(const int start_index, const char *buffer, const int buffer_length, const char *str){

    int str_len = strlen(str);
    char temp_str[15]; /**< str cannot be greater than 14 charachter! */

    if(start_index + str_len >= buffer_length)
        return -1; //not enough characters in the buffer

    strncpy(temp_str, &buffer[start_index], str_len);
    //add null-characther at the end
    temp_str[str_len] = '\0';

    if(!strcmp(temp_str, str)){
        //found str in buffer, starting from start_index
        return start_index;
    }

    //str not found in buffer
    return -1;
}

/**
* Extract data from buffer, starting from index, until a come is found. Update index. Returns a double
*
* @param index_pointer      pointer to index to be updated at the end of the function, if no error, buffer[i] = ','
* @param buffer             buffer
* @param buffer_length      length of buffer
* @param ret_val_pointer    pointer to variable with the final result
* @return 	true if no error
*/
bool d_extract_until_coma(int *index_pointer, const char *buffer, const int buffer_length, double *ret_val_pointer){

	int counter = 0;
    char temp_char[SAFETY_COUNTER_EXTRACT];
    int i = *index_pointer;

    while(i < buffer_length && buffer[i] != ','){
					
		temp_char[counter] = buffer[i];

        i++;
		counter++;
        if(counter >= SAFETY_COUNTER_EXTRACT){
            *ret_val_pointer = 0;
            return false;
        }
	}

    //check if we exited from while loop because we have a valid data or not
    if(i == *index_pointer || i >= buffer_length){
        *ret_val_pointer = 0;
        return false; //not found valid data
    }

    //null terminate string
    temp_char[counter] = '\0';

    //update index
    *index_pointer = i;
    *ret_val_pointer = atof(temp_char);

    return true;
}

/**
* Extract data from buffer, starting from index, until a come is found. Update index. Returns a float
*
* @param index_pointer      pointer to index to be updated at the end of the function, if no error, buffer[i] = ','
* @param buffer             buffer
* @param buffer_length      length of buffer
* @param ret_val_pointer    pointer to variable with the final result
* @return 	true if no error
*/
bool f_extract_until_coma(int *index_pointer, const char *buffer, const int buffer_length, float *ret_val_pointer){

    int counter = 0;
    char temp_char[SAFETY_COUNTER_EXTRACT];
    int i = *index_pointer;

    while(i < buffer_length && buffer[i] != ','){

        temp_char[counter] = buffer[i];

        i++;
        counter++;
        if(counter >= SAFETY_COUNTER_EXTRACT){
            *ret_val_pointer = 0;
            return false;
        }
    }

    //check if we exited from while loop because we have a valid data or not
    if(i == *index_pointer || i >= buffer_length){
        *ret_val_pointer = 0;
        return false; //not found valid data
    }

    //null terminate string
    temp_char[counter] = '\0';

    //update index
    *index_pointer = i;
    *ret_val_pointer = (float)atof(temp_char);

    return true;
}


/**
 * Jump to next the ',' in buffer. -1 on error.
 *
 * If buffer[start_index] is a ',' returns start_index.
 * @return -1 if no ',' found, index of ','in the buffer on succes.
*/
int jump_to_next_coma(const int start_index, const char *buffer, const int buffer_length){

    int i = start_index;

    while(i < buffer_length && buffer[i] != ','){
        i++;
    }

    //check if we exited from while loop because we have a ',' or because we arraived at the end
    if(i >= buffer_length)
        return -1;

    return i;
}

/**
* Initialize weather station 200WX.
*
* disable all the default messages from the waether station and after that enable only the messages in which
* we are interested in.
*
* @param 				pointer com port file descriptor
* @return 				true on success
*/
bool weather_station_init(int *wx_port_pointer){

    char raw_buffer[300];
	*wx_port_pointer = open("/dev/ttyS5", O_RDWR); // Serial 5, read works, write works
	// This is serial port 4 according to: pixhawk.org/dev/wiring
	if (*wx_port_pointer < 0) {
	        errx(1, "failed to open port: /dev/ttyS5");
           return false;
	    }
    warnx(" starting initialization.\n");

	// Set baud rate of wx_port to 4800 baud
    pixhawk_baudrate_set(*wx_port_pointer, 4800);

	// wait 5 [seconds] for the WX to power up before sending commands (SYS 2999)
	sleep(5); 

	// start with a new line:
	uint8_t new_line[] = {'\n'};				
	write(*wx_port_pointer, new_line, sizeof(new_line));
    //warnx(" new line.\n");

	// stop transmitting
    encode_msg_200WX(wx_port_pointer, "PAMTX,0");
    //warnx(" stop transmitting.\n");

	// wait for 2 seconds for stability
	sleep(2); 

    // Disable all the transmitted sentences.
    encode_msg_200WX(wx_port_pointer, "PAMTC,EN,ALL,0,1");
    //warnx(" PAMTC,EN,ALL,0,1\n");

	// wait for 2 seconds for stability
	sleep(2); 

    if(AS_TYPE_OF_ENVIRONMENT == 1){//outdoor
        warnx(" enabling outdoor messages.\n");

        // enable  GPS GPGGA message, set 0.1 sec as amount of time between succesive trasmission
        encode_msg_200WX(wx_port_pointer, "PAMTC,EN,GGA,1,1");

        // enable  GPS GPGSA message, set 0.1 sec as amount of time between succesive trasmission
        encode_msg_200WX(wx_port_pointer, "PAMTC,EN,GSA,1,1");

        // enable  GPS GPVTG message, set 0.1 sec as amount of time between succesive trasmission
        encode_msg_200WX(wx_port_pointer, "PAMTC,EN,VTG,1,1");

        // enable heading w.r.t. True North, message HCHDT, set 0.1 sec as amount of time between succesive trasmission
        encode_msg_200WX(wx_port_pointer, "PAMTC,EN,HDT,1,1");

        // enable wind direction and speed w.r.t. True North, message WIMWD, set 0.1 sec as amount of time between succesive trasmission
        encode_msg_200WX(wx_port_pointer, "PAMTC,EN,MWD,1,1");
    }

    // enable relative wind  measurement, set 0.1 sec as amount of time between succesive trasmission
    encode_msg_200WX(wx_port_pointer, "PAMTC,EN,VWR,1,1");

    // enable vessel attitude (pitch and roll), set 0.1 sec as amount of time between succesive trasmission
    encode_msg_200WX(wx_port_pointer, "PAMTC,EN,XDRB,1,1");

    // enable Roll, Pitch, Yaw rate relative to the vessel frame, set 0.1 sec as amount of time between succesive trasmission
    encode_msg_200WX(wx_port_pointer, "PAMTC,EN,XDRE,1,1");

    // enable x, y, z accelerometer readings, set 0.1 sec as amount of time between succesive trasmission
    encode_msg_200WX(wx_port_pointer, "PAMTC,EN,XDRC,1,1");


	// switch to 38400 baud (the highest possible baud rate):
    encode_msg_200WX(wx_port_pointer, "PAMTC,BAUD,38400");

	// wait for 2 seconds for stability
	sleep(2); 

	// switch the pixhawk baudrate to 38400
	pixhawk_baudrate_set(*wx_port_pointer, 38400); 

	// tell the weather station to start transmitting again (now at 38400 baud):
    encode_msg_200WX(wx_port_pointer, "PAMTX,1");

	// erase received but not read yet data from serial buffer 
	for (int i=0; i<4; i++)
		read(*wx_port_pointer, &raw_buffer, sizeof(raw_buffer));
    sleep(1);		// collect enough data for first parsing

    warnx(" ending initialization.\n");

	return true;
}

/**
 * Encode str between '$' and '*', add checksum at the end and \r,\n and send it to 200WX.
 *
*/
void encode_msg_200WX(int *wx_port_point, const char *str){

    int i;
    uint8_t checksum;
    char app_buff[250];

    checksum = str[0];

    for(i = 1; i < strlen(str); i++){
        checksum = checksum ^ str[i];
    }

    //encode the message
    sprintf(app_buff, "$%s*%x\r\n", str, checksum);

    //send message
    send_three_times(wx_port_point, app_buff, 6 + strlen(str));
}

/**
 * Send three times (according to marine talk) the same data to 200WX station.
 *
*/
void send_three_times(const int *wx_port_pointer, const uint8_t *msg, const int length){

    write(*wx_port_pointer, msg, length);
    write(*wx_port_pointer, msg, length);
    write(*wx_port_pointer, msg, length);
}

/**
* Set baud rate between weather station 200WX and pixhawk. (Marine talk, send everything 3 times).
*
* @param wx_port	name of the UART port
* @param baudrate 	baudrate of the communication
*/
bool pixhawk_baudrate_set(int wx_port, int baudrate){		// Set the baud rate of the pixhawk to baudrate:
	struct termios wx_port_config;
	tcgetattr(wx_port, &wx_port_config);
	int in_return;
	int out_return;

	in_return = cfsetispeed(&wx_port_config, baudrate);
	out_return = cfsetospeed(&wx_port_config, baudrate);

    if(in_return == -1 || out_return == -1){
        //error
        errx(1, "failed to set speed of: /dev/ttyS5");
        return false;
    }
	tcsetattr(wx_port, TCSANOW, &wx_port_config); // Set the new configuration

	return true;
}

/**
* Initializes all the variables used in the indoor version of the parser.
*
* Set all fields to 0, update timestamp and advertise each topic
*
* @param wx_port_pointer		pointer to COM file descriptor
* @return 						true is evrything is ok
*/
bool parser_variables_init(int *wx_port_pointer,
                          struct subscribtion_fd_s  *subs_p,
                          struct published_fd_s     *pubs_p,
                          struct structs_topics_s   *strs_p){

    // try to open COM port to talk with wather 200WX station
    if(!weather_station_init(wx_port_pointer))
        return false; //failed to open COM port

    // subscribe to sensor_combined topic
    subs_p->sensor_sub = orb_subscribe(ORB_ID(sensor_combined));
    //orb_set_interval(*sensor_sub_fd_pointer, 120);	// set px4 sensors update every 0.12 [second] = 8.3 Hz
    orb_set_interval(subs_p->sensor_sub, 110);	// set px4 sensors update every 0.11 [second] = 9.1 Hz
    //orb_set_interval(*sensor_sub_fd_pointer, 100);	// set px4 sensors update every 0.10 [second] = 10 Hz
    //orb_set_interval(*sensor_sub_fd_pointer, 80);	// set px4 sensors update every 0.08 [second] = 12 Hz

    // advertise attitude topic (ATT)
    memset(&(strs_p->att_s), 0, sizeof(strs_p->att_s));
    strs_p->att_s.timestamp = hrt_absolute_time();
    pubs_p->att_pub = orb_advertise(ORB_ID(vehicle_attitude), &(strs_p->att_s));

    // advertise vehicle_gps_position topic
    memset(&(strs_p->gps_s), 0, sizeof(strs_p->gps_s));
    strs_p->gps_s.timestamp_time  = hrt_absolute_time();
    pubs_p->gps_pub = orb_advertise(ORB_ID(vehicle_gps_position), &(strs_p->gps_s));

    // advertise wind_sailing topic
    memset(&(strs_p->wind_sailing_s), 0, sizeof(strs_p->wind_sailing_s));
    strs_p->wind_sailing_s.timestamp = hrt_absolute_time();
    pubs_p->wind_sailing = orb_advertise(ORB_ID(wind_sailing), &(strs_p->wind_sailing_s));

    // advertise vehicle_bodyframe_meas topic
    memset(&(strs_p->bodyframe_meas_s), 0, sizeof(strs_p->bodyframe_meas_s));
    strs_p->bodyframe_meas_s.timestamp = hrt_absolute_time();
    pubs_p->bodyframe_meas = orb_advertise(ORB_ID(vehicle_bodyframe_meas), &(strs_p->bodyframe_meas_s));

    return true;
}

/**
* Retrieve indoor data(by readings from UART) when pool() returns correctly.
*
* @param wx_port_pointer		pointer to COM file descriptor
* @return 						true is evrything is ok
*/
bool retrieve_data(int *wx_port_pointer,
                   struct subscribtion_fd_s  *subs_p,
                   struct structs_topics_s   *strs_p){

	struct sensor_combined_s sensor_combined_raw;
    int buffer_length;

	// copy sensors raw data into local buffer, actually this action is need only to apply downsampling time
    orb_copy(ORB_ID(sensor_combined), subs_p->sensor_sub, &sensor_combined_raw);

	// read UART when px4 sensors are updated
    buffer_length = read(*wx_port_pointer, buffer_global, sizeof(buffer_global));

    if(buffer_length < 1)
        return false;

    // see if buffer there is one (or more) YXXDR message(s)
    xdr_parser(buffer_global, buffer_length, &(strs_p->att_s), &(strs_p->bodyframe_meas_s));

    // see if buffer there is one (or more) WIVW_ message(s)
    vr_parser(buffer_global, buffer_length, &(strs_p->wind_sailing_s));

    if(AS_TYPE_OF_ENVIRONMENT == 1){//outdoor

        //Simulazione dati GPS, COMMENTA IN UTILIZZO VERO
        /*char good_b[] = {"GPGGA,151939.20,4722.9509,N,00833.3726,E,1,4,9.3,0.0,M,49.1,M,,*,$,GPVTG,182.9,T,181.0,M,0.0,N,15.9,K,A,*,$,GPGSA,A,3,11,17,20,4,,,,,,,,,14.0,9.3,10.5,*,55555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555"};

        gp_parser(good_b, sizeof(good_b), gps_raw_pointer);

        //Simulazione dati heading
        char buf_hdt[] = {"HCHDT,025.3,T,*********************************************************"};
        hdt_parser(buf_hdt, sizeof(buf_hdt), att_raw_pointer);

        //Simulazione true wind
        char buf_mwd[] = {"$WIMWD,134.0,T,132.1,M,5.5,N,2.8,M,*,***********************************************"};
        mwd_parser(buf_mwd, sizeof(buf_mwd), wind_sailing_pointer);*/

        //Fine simalazione


        // see if buffer there is one (or more) GPXXX message(s)
        gp_parser(buffer_global, buffer_length, &(strs_p->gps_s));

        // see if buffer there is one (or more) HCHDT message(s)
        hdt_parser(buffer_global, buffer_length, &(strs_p->att_s));

        // see if buffer there is one (or more) WIMWD message(s)
        mwd_parser(buffer_global, buffer_length, &(strs_p->wind_sailing_s));
    }

    //debug
    //att_raw_pointer->yaw = buffer_length; //cancella

    //debug
    //att_raw_pointer->yaw = wind_sailing_pointer->angle_apparent; //cancella

    return true;
}

/**
* Parse transducer data received from 200WX, YXXDR message type B, C and E.
*
* @param buffer                 buffer with data
* @param buffer_length          length of buffer
* @param att_pub_fd_pointer		pointer to handler returnd by orb_advertise
*/
void xdr_parser(const char *buffer, const int buffer_length,
                struct vehicle_attitude_s *att_raw_pointer,
                struct vehicle_bodyframe_meas_s *bodyframe_meas_raw_pointer){

    int i = 0;
    float temp_val;

    float pitch;
    float rollspeed;
    float pitchspeed;
    float lat_acc;
    float lon_acc;
    float vert_acc;

    // it's worthless to check if there won't be enough data anyway..
    for(i = 0; (buffer_length - i) > MIN_BYTE_FOR_PARSING_LONG_MSG; i++){

        i = find_string_everywhere(i, buffer, buffer_length, "YXXDR");

        if(i == -1){
            return; //no YXXDR found in buffer
        }

        //Uncomment for debug
        //debug_print_until_char(buffer, buffer_length, i, '*'); //cancella

        /*found YXXDR message in buffer, starting from i
         * |Y|X|X|D|R|,|A|,|byte1 of first value|byte2 of first value| etc.
         *  ^
         *  |
         *  i   */
        i += 8;	// position to byte1 of first value

        //extract first value, on error go next iteration and see in buffer for another YXXDR string
        if(f_extract_until_coma(&i, buffer, buffer_length, &temp_val)){

            //first value extracted, everything is ok, i is the comma ','
            i++;//now i is the after the above ','
            if (buffer[i] == 'D'){
                // then it means we are parsing either XDR type B or type E;
                //i+1 is ',' ; i+2 is the first charachter to analyze
                i += 2;

                //app_i = find_string_here(i, buffer, buffer_length, "PTCH");

                if(i == find_string_here(i, buffer, buffer_length, "PTCH")){
                    //we're parsng YXXDR message type B.
                    //save pitch
                    pitch = temp_val;
                    /* |P|T|C|H|,|A|,|byte1 of second value|byte2 of second value| etc.
                     *  ^
                     *  |
                     *  i
                     * */
                    i += 7;	// position to byte1 of second value

                    //extract second value
                    if(f_extract_until_coma(&i, buffer, buffer_length, &temp_val)){
                        //second value extracted, set value in topic's structure
                        att_raw_pointer->timestamp = hrt_absolute_time();
                        att_raw_pointer->roll = temp_val * deg2rad; /// Roll in rad.
                        att_raw_pointer->pitch = pitch * deg2rad;   /// Pitch in rad.

                        //cancella
                        //warnx("Roll %2.3f \t Pitch %2.3f \n", (double)temp_val, (double)pitch);
                        //fine cancella

                    }
                }
                else if(i == find_string_here(i, buffer, buffer_length, "RRTR")){

                    //we're parsng YXXDR message type E. set roll rate
                    //save rollspeed
                    rollspeed = temp_val;

                    //cancella
                    //debug_print_until_char(buffer, buffer_length, cancella, '*');
                    //fine cancella

                    /* |R|R|T|R|,|A|,|byte1 of second value|byte2 of second value| etc.
                     *  ^
                     *  |
                     *  i
                     * */
                    i += 7;	// position to byte1 of second value
                    //extract second value
                    if(f_extract_until_coma(&i, buffer, buffer_length, &temp_val)){
                        //save pitchspeed
                        pitchspeed = temp_val;

                        i++;
                        /* |D|,|P|R|T|R|,|A|,|byte1 of third value|byte2 of third value| etc.
                         *  ^
                         *  |
                         *  i
                         * */
                        i += 9;	// position to byte1 of third value
                        if(f_extract_until_coma(&i, buffer, buffer_length, &temp_val)){
                            //set value in topic's structure
                            att_raw_pointer->timestamp = hrt_absolute_time();
                            att_raw_pointer->rollspeed = rollspeed * deg2rad;     ///< Roll speed in rad/s.
                            att_raw_pointer->pitchspeed = pitchspeed * deg2rad;   ///< Pitch speed in rad/s.
                            att_raw_pointer->yawspeed = temp_val * deg2rad;       ///< Yaw speed in rad/s.

                            //cancella
                            //warnx("RR %2.3f \t PR %2.3f \t YR %2.3f \n", (double)rollspeed, (double)pitchspeed, (double)temp_val);
                            //fine cancella
                        }
                    }
                }
            }
            else{

                //we're parsng YXXDR message type C, temp_val is Acceleration on latitudinal axis
                lat_acc = temp_val;
                //i is the ','
                /*|,|G|,|X|A|C|C|,|A|,|byte1 of acc on longitudinal axis|etc..
                 * ^
                 * |
                 * i
                */
                i += 10;
                if(f_extract_until_coma(&i, buffer, buffer_length, &lon_acc)){
                    //i is the ','
                    /*|,|G|,|Y|A|C|C|,|A|,|byte1 of acc on longitudinal axis|etc..
                     * ^
                     * |
                     * i
                    */
                    i += 10;
                    if(f_extract_until_coma(&i, buffer, buffer_length, &vert_acc)){
                        //set value in topic's structure
                        bodyframe_meas_raw_pointer->timestamp = hrt_absolute_time();
                        bodyframe_meas_raw_pointer->acc_x = lon_acc;
                        bodyframe_meas_raw_pointer->acc_y = lat_acc;
                        bodyframe_meas_raw_pointer->acc_z = vert_acc;

                        //cancella
                        //warnx("acc x: %2.3f \t acc y %2.3f \t acc z %2.3f \n",(double)lon_acc,(double)lat_acc,(double)vert_acc);
                        //fine cancella
                    }

                }

            }
        }
    }

}

/**
* Parse transducer data received from 200WX, GPGGA, GPGSA, GPVTG messages.
*
* @param buffer                 buffer with data
* @param buffer_length          length of buffer
* @param gps_raw_pointer		pointer to handler returnd by orb_advertise
*/
void gp_parser(const char *buffer, const int buffer_length, struct vehicle_gps_position_s *gps_raw_pointer){

    int i = 0;
    int app_i;
    int counter = 0;
    // UTC in form of hhmmss.00 + last slot for '\0' (null that terminates string)
    char time_char[SAFETY_COUNTER_EXTRACT];
    char hour_char[3];
    char minute_char[3];
    char second_char[6];
    float latitude;
    float longitude;
    float gps_quality;
    float satellites_used;
    float eph;
    float alt;
    int hour;
    int min;
    float sec;
    float course_over_ground;
    float speed_over_ground;

    // it's worthless to check if there won't be enough data anyway..
    for(i = 0; (buffer_length - i) > MIN_BYTE_FOR_PARSING_LONG_MSG; i++){

        i = find_string_everywhere(i, buffer, buffer_length, "GP");

        if(i == -1)
            return; //no GPXX found in buffer

        //Uncomment for debug
        //debug_print_until_char(buffer, buffer_length, i, '*');

        if(buffer[i+2] == 'G' && buffer[i+3] == 'G' && buffer[i+4] == 'A'){
            /*found GPGGA message in buffer, starting from i
             * |G|P|G|G|A|,|byte1 of UTC|byte2 of UTC| etc
             *  ^
             *  |
             *  i   */


            i += 6;	// position to byte1 of UTC

            //--- handle time ---

            while(buffer[i] != ','){
                if(counter >= SAFETY_COUNTER_EXTRACT){
                    return;// safety
                }
                time_char[counter] = buffer[i];
                i++;
                counter++;
            }

            if(counter < 8){
                //failed readind time, use 00:00:00 and try to parse GPS data
                hour =  min = sec = 0;
            }
            else{
                //convert time from string to numeric values
                hour_char[0] = time_char[0];
                hour_char[1] = time_char[1];
                hour_char[2] = '\0';

                minute_char[0] = time_char[2];
                minute_char[1] = time_char[3];
                minute_char[2] = '\0';

                for (int j=0 ; j<5 ; j++){
                    second_char[j] = time_char[4+j];
                }
                second_char[5] = '\0';

                hour = atoi(hour_char) ;
                min = atoi(minute_char);
                sec = atof(second_char);
            }

            //--- latitude ---

            // i is the comma ','
            i++;// position i to byte1 of Latitude
            if(f_extract_until_coma(&i, buffer, buffer_length, &latitude)){

                //--- longitude ---

                // i   is the comma ','
                /// i+1 is 'N' to symbolize the northern hemisphere (ok for Switzerland!!!)
                // i+2 is the comma ','
                i += 3;	// position to byte1 of longitude
                if(f_extract_until_coma(&i, buffer, buffer_length, &longitude)){

                    //--- gps quality ---

                    // i   is the comma ','
                    /// i+1 is 'E' to symbolize the East longitude (ok for Switzerland!!!)
                    // i+2 is the comma ','
                    i += 3;	// position to byte1 of GPS quality indicator
                    if(f_extract_until_coma(&i, buffer, buffer_length, &gps_quality)){

                        //--- satellites_used ---

                        // i   is the comma ','
                        i ++;	// position to byte1 of number of satellites in use
                        if(f_extract_until_coma(&i, buffer, buffer_length, &satellites_used)){

                            //--- eph ---

                            // i   is the comma ','
                            i ++;	// position to byte1 of HDOP
                            if(f_extract_until_coma(&i, buffer, buffer_length, &eph)){

                                //--- altitude ---

                                // i   is the comma ','
                                i ++;	// position to byte1 of altitude relative to MSL
                                if(f_extract_until_coma(&i, buffer, buffer_length, &alt)){

                                    //save data in the struct
                                    gps_raw_pointer->timestamp_time = hrt_absolute_time();

                                    //TODO cosa mettere qui visto che 200WX non ci da tempo totale per sdlo2 ?
                                    gps_raw_pointer->time_gps_usec = 3000000l;

                                    // time in microseconds TODO da levare
                                    gps_raw_pointer->timestamp_position = (hour * 3600 +
                                                                        min * 60 + sec) * 1000000;

                                    gps_raw_pointer->timestamp_position = hrt_absolute_time();

                                    //convert lat and long in degrees and multiple for 1E7 as required in vehicle_gps_position topic
                                    gps_raw_pointer->lat = nmea_ndeg2degree(latitude) * 1e7f; /// Valid only for North latitude (for now)
                                    gps_raw_pointer->lon = nmea_ndeg2degree(longitude) * 1e7f;/// Valid only for East longitude (for now)
                                    gps_raw_pointer->satellites_used = satellites_used;
                                    gps_raw_pointer->eph = eph;
                                    //set altitude from meters in millimeter as requested in vehicle_gps_position topic
                                    gps_raw_pointer->alt = alt * 1000; ///altitude in millimeters


                                }
                            }
                        }

                    }
                }
            }


        }
        else if(buffer[i+2] == 'G' && buffer[i+3] == 'S' && buffer[i+4] == 'A'){
            /*found GPGSA message in buffer, starting from i
             * |G|P|G|S|A|,|M or A|,|Type of fix|
             *  ^
             *  |
             *  i   */

            //cancella
            //debug_print_until_char(buffer, buffer_length, i, '*');


            i += 8;	// type of fix

            switch(buffer[i]){
            case '1':
                gps_raw_pointer->fix_type = 1;
                break;
            case '2':
                gps_raw_pointer->fix_type = 2;
                break;
            case '3':
                gps_raw_pointer->fix_type = 3;
                break;
            default:
                gps_raw_pointer->fix_type = 1; //Error, no fix valid found in the message
            }

        }
        else if(buffer[i+2] == 'V' && buffer[i+3] == 'T' && buffer[i+4] == 'G'){
            /*found GPVTG message in buffer, starting from i
             * |G|P|V|T|G|,|byte1 of Course Over Ground|
             *  ^
             *  |
             *  i   */

            i += 6;

            if(f_extract_until_coma(&i, buffer, buffer_length, &course_over_ground)){

                //i is ',', i+1 is 'T', i+2 is ',' i+3 is byte of course wrt magnetic north
                i += 3;
                //do not extract course over ground w.r.t. to magnetic north
                app_i = jump_to_next_coma(i, buffer, buffer_length);

                if(app_i != -1){
                    /*
                     * |,|M|,|byte1 of speed over ground in knots|
                     *  ^
                     *  |
                     *  i
                    */
                    app_i += 3;
                    //do not extract speed over ground in knots
                    app_i = jump_to_next_coma(app_i, buffer, buffer_length);

                    if(app_i != -1){
                        //update i
                        i = app_i;

                        /*
                         * |,|K|,|byte1 of speed over ground in m/s|
                         *  ^
                         *  |
                         *  i
                        */


                        i += 3;

                        if(f_extract_until_coma(&i, buffer, buffer_length, &speed_over_ground)){
                            //save data in struct
                            gps_raw_pointer->timestamp_velocity = hrt_absolute_time();
                            //put speed ground vel in vel_n_mes beacuse vel_m_s is not saved in the SD card by sdlog2

                            //Positive course_over_ground on the right, negative on the left
                            if(course_over_ground > 180.0f && course_over_ground <= 360.0f)
                                course_over_ground = course_over_ground - 360.0f;

                            gps_raw_pointer->vel_n_m_s = speed_over_ground * km_h2m_s; /// Speed over ground in m/s.
                            gps_raw_pointer->cog_rad = course_over_ground * deg2rad; /// Course over ground w.r.t true North in rad, positive on the right, negative on the left.

                            //cancella
                            //warnx("SOG %3.2f \t COG: %3.2f \n", (double)gps_raw_pointer->vel_n_m_s, (double)gps_raw_pointer->cog_rad);
                            //fine cancella

                            //TODO vedere se usare campo 8 per validazione dei dati
                        }
                    }
                }

            }
        }




    }
}

/**
 *  Convert NDEG (NMEA degree: [degree][min].[sec/60]) to fractional degree
*/
float nmea_ndeg2degree(float val)
{
    float deg = ((int)(val / 100));
    val = deg + (val - deg * 100) / 60.0f;
    return val;
}



/**
* Parse transducer data received from 200WX, VWR messages.
*
* @param buffer                 buffer with data
* @param buffer_length          length of buffer
* @param wind_sailing_pointer   pointer to handler returnd by orb_advertise
*/
void vr_parser(const char *buffer, const int buffer_length, struct wind_sailing_s *wind_sailing_pointer){

    int i = 0;
    int app_i;
    float temp_angle;
    float temp_speed;

    // it's worthless to check if there won't be enough data anyway..
    for(i = 0; (buffer_length - i) > MIN_BYTE_FOR_PARSING_MEDIUM_MSG; i++){
        // see if we have a relative wind information
        i = find_string_everywhere(i, buffer, buffer_length, "WIVWR");

        if(i == -1)
            return;//no message found

        //Uncomment for debug
        //debug_print_until_char(buffer, buffer_length, i, '*');

        //we have WIVWR message
        /*
         * |W|I|V|W|R|,|byte1 of first value|byte2 of first value| etc.
         *  ^
         *  |
         *  i   */
        i += 6;	// position to byte1 of first value

        //extract first value
        if(f_extract_until_coma(&i, buffer, buffer_length, &temp_angle)){
            //i is ','
            //i+1 is L or R to indicate from wich direction the wind is blowing wrt vessel heading
            i++;
            if(buffer[i] == 'L')//TODO check if ok
                temp_angle = -temp_angle; /// CHECK IF OK

            //i+1 is ','
            //i+2 is the first byte of wind speed (in knot)
            i += 2;

            //do not extract wind speed in knots
            app_i = jump_to_next_coma(i, buffer, buffer_length);

            if(app_i != -1){

                //update i
                i = app_i;
                /*
                 * |,|N|,|byte1 of wind speed in m/s|
                 *  ^
                 *  |
                 *  i   */
                i += 3;

                //extract second value
                if(f_extract_until_coma(&i, buffer, buffer_length, &temp_speed)){
                    //set value in topic's structure
                    wind_sailing_pointer->timestamp = hrt_absolute_time();
                    wind_sailing_pointer->angle_apparent = temp_angle * deg2rad; ///Apparent wind angle in rad, positive on the right, negative on the left.
                    wind_sailing_pointer->speed_apparent = temp_speed; ///Apparent wind speed in m/s.
                }

            }

        }

    }

}

/**
 * Parse HCHDT message. Heading w.r.t. True North saved as yaw angle
*/
void hdt_parser(const char *buffer, const int buffer_length, struct vehicle_attitude_s *att_raw_pointer){

    int i = 0;
    float heading;

    for(i = 0; (buffer_length - i) > MIN_BYTE_FOR_PARSING_SHORT_MSG; i++){

        i = find_string_everywhere(i, buffer, buffer_length, "HCHDT");

        if(i == -1)
            return;//no message found

        //Uncomment for debug
        //debug_print_until_char(buffer, buffer_length, i, '*');

        //we have HCHDT message
        /*
         * |H|C|H|D|T|,|byte1 of heading w.r.t. True North|
         *  ^
         *  |
         *  i   */
        i += 6;	// position to byte1

        if(f_extract_until_coma(&i, buffer, buffer_length, &heading)){

            //Positive heading on the right, negative on the left
            if(heading > 180.0f && heading <= 360.0f)
                heading = heading - 360.0f;

            att_raw_pointer->yaw = heading * deg2rad; /// Heading w.r.t. true North in rad, positive on the right.
        }

    }

}


/**
  * Parses a WIMWD message, if any in the buffer. Saves wind speed and direction w.r.t. true North.
*/
void mwd_parser(const char *buffer, const int buffer_length, struct wind_sailing_s *wind_sailing_pointer){

    int i;
    int app_i;
    float speed;
    float direction;

    for(i = 0; (buffer_length - i) > MIN_BYTE_FOR_PARSING_MEDIUM_MSG; i++){

        i = find_string_everywhere(i, buffer, buffer_length, "WIMWD");

        if(i == -1)
            return;//no message found

        //Uncomment for debug
        //debug_print_until_char(buffer, buffer_length, i, '*');

        //we have WIMWD message
        /*
         * |W|I|M|W|D|,|byte1 of wind direction w.r.t. True North|
         *  ^
         *  |
         *  i   */
        i += 6;	// position to byte1

        if(f_extract_until_coma(&i, buffer, buffer_length, &direction)){
            //i is ',' i+1 is 'T' i+2 is ',' i+3 is byte1 of direction wrt magnetic north
            i += 3;
            //do not extract direction w.r.t. to magnetic north
            app_i = jump_to_next_coma(i, buffer, buffer_length);

            if(app_i != -1){

                /*
                 * |,|M|,|byte1 of wind speed in knots|
                 *  ^
                 *  |
                 *  i   */
                app_i += 3;
                //do not extract wind speed in knots
                app_i = jump_to_next_coma(app_i, buffer, buffer_length);

                if(app_i != -1){
                    //update i
                    i = app_i;
                    /*
                     * |,|K|,|byte1 of wind speed in m/s|
                     *  ^
                     *  |
                     *  i   */
                    i += 3;
                    if(f_extract_until_coma(&i, buffer, buffer_length, &speed)){

                        //save data in struct
                        wind_sailing_pointer->timestamp = hrt_absolute_time();

                        //Positive direction on the right, negative on the left
                        if(direction > 180.0f && direction <= 360.0f)
                            direction = direction - 360.0f;

                        wind_sailing_pointer->angle_true = direction * deg2rad;/// True wind direction wrt true North in rad, positive on the right.
                        wind_sailing_pointer->speed_true = speed;/// True wind speed wrt true North in m/s

                        //cancella
                        //warnx("true speed: %3.3f", (double) speed);
                        //warnx("true direction: %3.3f\n", (double) direction);
                        //fine cancella
                    }

                }
            }
        }
    }
}

/**
 * Print data in buffer from start to end (or end of buffer). buffer[end] is not printed.
*/
void debug_print_nchar(const char *buffer, const int length, const int start, const int end){
    char str[301];
    int i;

    for(i = 0; (i + start) < length && (i + start) <= end && i < 300; i++){

        str[i] = buffer[start+i];
    }

    str[i] = '\0';

    warnx("buf_len %d; start %d end %d real_end %d \n %s \n", length, start, end, start + i-1, str);
}

/**
 * Print data in buffer from start untile stop_char is found (or end of buffer). stop_char is not printed
*/
void debug_print_until_char(const char *buffer, const int length, const int start, const char stop_char){
    char str[301];
    int i;

    for(i = 0; (i + start) < length && buffer[start+i] != stop_char && i < 300; i++){

        str[i] = buffer[start+i];
    }

    str[i] = '\0';

    warnx("buf_len %d; start %d  real_end %d \n %s \n", length, start, start + i-1, str);
}

/**
 * Publish new data.
*/
void publish_new_data(struct published_fd_s *pubs_p, struct structs_topics_s *strs_p){

    //publish attituide data
    orb_publish(ORB_ID(vehicle_attitude), pubs_p->att_pub, &(strs_p->att_s));

    //publish wind_sailing data
    orb_publish(ORB_ID(wind_sailing), pubs_p->wind_sailing, &(strs_p->wind_sailing_s));

    //publish vehicle_bodyframe_meas data
    orb_publish(ORB_ID(vehicle_bodyframe_meas), pubs_p->bodyframe_meas, &(strs_p->bodyframe_meas_s));

    if(AS_TYPE_OF_ENVIRONMENT == 1){//outdoor
        //publish gps data
        orb_publish(ORB_ID(vehicle_gps_position), pubs_p->gps_pub, &(strs_p->gps_s));
    }
}
