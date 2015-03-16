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
 * @file topics_handler.h
 *
 * Structures to handle topic subscribtion/publishing.
 *
 *
 * @author Marco Tranzatto <marco.tranzatto@gmail.com>
 */

#ifndef TOPICS_HANDLER_H
#define TOPICS_HANDLER_H

// for uORB topics
#include <uORB/uORB.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/wind_sailing.h>
#include <uORB/topics/vehicle_gps_position.h>
#include <uORB/topics/boat_weather_station.h>
//#include <uORB/topics/debug_values.h>

#include <stdio.h>//bool type


struct subscribtion_fd_s{
    int sensor;
};

struct published_fd_s{
    orb_advert_t gps;
    orb_advert_t wind_sailing;
    #if ENABLE_BOAT_WEATHER_STATION_MSGS == 1
    orb_advert_t boat_weather_station;
    #endif
};

struct structs_topics_s{
    struct vehicle_gps_position_s  gps;
    struct wind_sailing_s wind_sailing;
    #if ENABLE_BOAT_WEATHER_STATION_MSGS == 1
    struct boat_weather_station_s boat_weather_station;
    bool boat_weather_station_updated;
    #endif

    bool gps_updated;
    bool wind_updated;
};

#endif // TOPICS_HANDLER_H
