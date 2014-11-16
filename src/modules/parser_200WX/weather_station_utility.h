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
 * @file weather_station_utility.h
 *
 * Definition of functions to initialize weather station 200WX.
 *
 *
 * @author Marco Tranzatto <marco.tranzatto@gmail.com>
 */
#ifndef WEATHER_STATION_UTILITY_H_
#define WEATHER_STATION_UTILITY_H_

//for baud rate selection of the UART port:
#include <termios.h>
#include <sys/types.h>

// to open a UART port:
#include <sys/stat.h>
#include <fcntl.h>

#include <stdio.h>//bool type

//setting for indoor or outdoor
#include "settings.h"

/** @brief Initialize weather station. */
bool weather_station_init(int *wx_port_point);

/** @brief Encode str in a message for 200WX.*/
void encode_msg_200WX(int *wx_port_point, const char *str);

/** @brief Send msg three times, marine talk. */
void send_three_times(const int *wx_port_pointer, const uint8_t *msg, const int length);

/** @brief Set baud rate. */
bool pixhawk_baudrate_set(int wx_port, int baudrate);

#endif /* WEATHER_STATION_UTILITY_H_ */
