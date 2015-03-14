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
 * @file weather_station_utility.c
 *
 * Implementation of functions to initialize weather station 200WX.
 *
 *
 * @author Marco Tranzatto <marco.tranzatto@gmail.com>
 */

#include "weather_station_utility.h"

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


    warnx(" starting weather station initialization.\n");

    // wait 5 seconds for the WX to power up before sending commands (SYS 2999)
    sleep(5);

    // Set baud rate of wx_port to 4800 baud
    pixhawk_baudrate_set(*wx_port_pointer, 4800);

    // wait for 2 seconds for stability
    sleep(2);

    // stop transmitting
    encode_msg_200WX(wx_port_pointer, "PAMTX,0");

    // switch to 38400 baud (the highest possible baud rate):
    encode_msg_200WX(wx_port_pointer, "PAMTC,BAUD,38400");

    // wait for 2 seconds for stability
    sleep(2);

    // switch the pixhawk baudrate to 38400
    pixhawk_baudrate_set(*wx_port_pointer, 38400);

    warnx(" switch pixhawk baudrate to 38400.\n");

    // tell the weather station to start transmitting again (now at 38400 baud):
    encode_msg_200WX(wx_port_pointer, "PAMTX,1");

    // Disable all the transmitted sentences.
    encode_msg_200WX(wx_port_pointer, "PAMTC,EN,ALL,0");

    if(AS_TYPE_OF_ENVIRONMENT == 1){//outdoor
        warnx(" enabling outdoor messages.\n");

        // enable  GPS GPGGA message, set 0.1 sec as amount of time between succesive trasmission
        encode_msg_200WX(wx_port_pointer, "PAMTC,EN,GGA,1,1");

        // enable  GPS GPGSA message, set 0.1 sec as amount of time between succesive trasmission
        encode_msg_200WX(wx_port_pointer, "PAMTC,EN,GSA,1,1");

        // enable  GPS GPVTG message, set 0.1 sec as amount of time between succesive trasmission
        encode_msg_200WX(wx_port_pointer, "PAMTC,EN,VTG,1,1");

        // enable  GPS GPZDA message, set 0.1 sec as amount of time between succesive trasmission
        encode_msg_200WX(wx_port_pointer, "PAMTC,EN,ZDA,1,1");

        #if ENABLE_BOAT_WEATHER_STATION_MSGS == 1
        // enable heading w.r.t. True North, message HCHDT, set 0.1 sec as amount of time between succesive trasmission
        encode_msg_200WX(wx_port_pointer, "PAMTC,EN,HDT,1,1");
        #endif

        // enable wind direction and speed w.r.t. True North, message WIMWD, set 0.1 sec as amount of time between succesive trasmission
        encode_msg_200WX(wx_port_pointer, "PAMTC,EN,MWD,1,1");
    }

    // enable relative wind  measurement, set 0.1 sec as amount of time between succesive trasmission
    encode_msg_200WX(wx_port_pointer, "PAMTC,EN,VWR,1,1");

    #if ENABLE_BOAT_WEATHER_STATION_MSGS == 1
    // enable vessel attitude (pitch and roll), set 0.1 sec as amount of time between succesive trasmission
    encode_msg_200WX(wx_port_pointer, "PAMTC,EN,XDRB,1,1");

    // enable Roll, Pitch, Yaw rate relative to the vessel frame, set 0.1 sec as amount of time between succesive trasmission
    encode_msg_200WX(wx_port_pointer, "PAMTC,EN,XDRE,1,1");

    // enable x, y, z accelerometer readings, set 0.1 sec as amount of time between succesive trasmission
    encode_msg_200WX(wx_port_pointer, "PAMTC,EN,XDRC,1,1");
    #endif //PARSE_VEHICLE_STATUS_MSG == 1

    warnx(" clean UART buffer before start.\n");

    // erase received but not read yet data from serial buffer
    for (int i=0; i<4; i++){
        read(*wx_port_pointer, raw_buffer, sizeof(raw_buffer));
    }
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

    for(int i = 0; i < 3; i++){
        write(*wx_port_pointer, msg, length);
    }
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
