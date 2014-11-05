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
 * @file navigation.c
 *
 * Computes NED position from geodedical information.
 *
 * @author Marco Tranzatto <marco.tranzatto@gmail.com>
 */

#include "navigation.h"

//WGS84 data
const float flatness = 0.0033528107.0f;

const float one_minus_flatness = 1.0f - flatness;

const float squared_one_minus_flatness = one_minus_flatness * one_minus_flatness;

const float earth_radius = 6378137.0f;

const float squared_earth_radius = earth_radius * earth_radius;

const float pi = 3.141592653589793f;

//Reference variables

static float lat0 = 0; ///Latitude of origin of NED system

static float lon0 = 0; ///Longitude of origin of NED system

static float alt0 = 0; ///Altitude of origin of NED system

static float x0 = 0; ///x coordinate in ECEF of origin of NED system

static float y0 = 0; ///y coordinate in ECEF of origin of NED system

static float z0 = 0; ///z coordinate in ECEF of origin of NED system

/**
 * Set the new origin position of NED frame.
 *
 * @param _lat0_p   pointer to latitude, in degrees, value of new origin.
 * @param _lon0_p   pointer to longitude, in degrees, value of new origin.
 * @param _alt0_p   pointer to altitude, in degrees, value of new origin.
*/
void set_ref0(const float *_lat0_p, const float *_lon0_p, const float *_alt0_p){

    //set geodedical reference of NED origin
    lat0 = *_lat0_p;
    lon0 = *_lon0_p;
    alt0 = *_alt0_p;

    //set ecef reference of NED origin
    geo_to_ecef(_lat0_p, _lon0_p, _lat0_p, &x0, &y0, &z0);
}

/**
 * Convert geodedical coordinate in NED coordinate
 *
 * @param gps_p     pointer to vehicle_gps_position struct.
 * @param north_p   pointer to variable which will contain north coordinate
 * @param east_p    pointer to variable which will contain east coordinate
 * @param down_p    pointer to variable which will contain down coordinate
*/
void geo_to_ned(const struct vehicle_gps_position_s *gps_p,
                float *north_p, float *east_p, float *down_p){

    const float *lat_p;
    const float *lon_p;
    const float *alt_p;

    float x;
    float y;
    float z;

    lat_p = &(gps_p->lat);
    lon_p = &(gps_p->lon);
    alt_p = &(gps_p->alt);

    //compute ECEF coordinate of the actual gps position
    geo_to_ecef(lat_p, lon_p, alt_p, &x, &y, &z);

    //compute NED position from ECEF coordinate
    ecef_to_ned(&x, &y, &z, north_p, east_p, down_p);
}

/**
 * Convert geodedical coordinate in ECEF coordinate
 *
 * @param lat_p     pointer to latitude value.
 * @param lon_p     pointer to longitude value.
 * @param alt_p     pointer to altitude value.
 * @param x_p       pointer to variable which will contain X coordinate in ECEF.
 * @param y_p       pointer to variable which will contain Y coordinate in ECEF.
 * @param z_p       pointer to variable which will contain Z coordinate in ECEF.
*/
void geo_to_ecef(const float *lat_p, const float *lon_p, const float *alt_p,
                 float *x_p, float *y_p, float *z_p){

    float mu;
    float l;
    float h;

    float lab_s;
    float r_s;

    //convert geo data from degrees to rad
    mu = *lat_p * pi / 180.0f;
    l = *lon_p * pi / 180.0f;
    h = *alt_p;

    lab_s = atan2(squared_one_minus_flatness * tan(mu), 1.0);

    r_s = sqrt((squared_earth_radius) / (1 + ((1 / squared_one_minus_flatness) - 1) * (sin(lab_s))^2));

    *x_p = r_s * cos(lab_s) * cos(l) + h * cos(mu) * cos(l);
    *y_p = r_s * cos(lab_s) * sin(l) + h * cos(mu) * sin(l);
    *z_p = r_s * sin(lab_s) + h * sin(mu);
}

