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

static const double squared_one_minus_flatness_m = 0.99330561993959; ///(1-flatness)^2 in meters

static const double  squared_earth_radius_m = 40680631590769; ///(arth_radius)^2 in meters

static const float deg2rad = 0.017453292f; // pi / 180

static float cosPhi = 0.0f;    ///cos(lat0)

static float sinPhi = 0.0f;    ///sin(lat0)

static float cosLambda = 0.0f; ///cos(lon0)

static float sinLambda = 0.0f; ///sin(lon0)

static int32_t x0_cm = 0; ///x coordinate in ECEF of origin of NED system, in centimeters

static int32_t y0_cm = 0; ///y coordinate in ECEF of origin of NED system, in centimeters

static int32_t z0_cm = 0; ///z coordinate in ECEF of origin of NED system, in centimeters

static const float E7 = 10000000.0f;

static const float E3 = 1000.0f;

static const float E2 = 100.0f;

static float R_r_ned[2][2] = {{0.0f, 0.0f},///Rotation matrix about Down axes, trasform coordinates from NED to Race frame
                             {0.0f, 0.0f}};


/** @brief convert geodedical coordinates into NED coordinate.*/
void geo_to_ned(const struct vehicle_global_position_s *gps_p,
                int32_t *north_cm_p, int32_t *east_cm_p, int32_t *down_cm_p);

/** @brief convert geodedical coordinates into ECEF coordinate.*/
void geo_to_ecef(const int32_t  *lat_d_e7_p, const int32_t  *lon_d_e7_p, const int32_t  *alt_mm_p,
                 int32_t  *x_cm_p, int32_t  *y_cm_p, int32_t  *z_cm_p);

/** @brief convert ECEF coordinates into NED coordinate.*/
void ecef_to_ned(const int32_t *x_cm_p, const int32_t *y_cm_p, const int32_t *z_cm_p,
                 int32_t *north_cm_p, int32_t *east_cm_p, int32_t *down_cm_p);

/** @brief convert Deg*E7 in rad */
float degE7_to_rad(const int32_t  *deg_e7_p);


/**
 * Convert geodedical coordinate in race frame coordinate.
 *
 * First step: convert geodedical coordinates in NED coordinates using lat0, lon0 and alt0 set by set_ref0().
 * Second step: use the mean wind angle set by set_mean_wind_angle()
 * for the rotation matrix from NED to Race frame.
 *
 * @param vehicle_global_position_s pointer to struct with gps filtered data.
 * @param x_cm_p                    pointer to returned value with x coordinate in race frame
 * @param x_cm_p                    pointer to returned value with x coordinate in race frame
 */
void geo_to_race(const struct vehicle_global_position_s *gps_p,
                 int32_t *x_cm_p, int32_t *y_cm_p){

    int32_t north_cm;
    int32_t east_cm;
    int32_t down_cm;


    //compute boat position in NED frame w.r.t. lat0 lon0 alt0 set by set_ref0()
    geo_to_ned(gps_p, &north_cm, &east_cm, &down_cm);

    //transform [north, east] coordinate in NED fram in [x, y] coordinate in race frame
    *x_cm_p = R_r_ned[0][0] * north_cm + R_r_ned[0][1] * east_cm;
    *y_cm_p = R_r_ned[1][0] * north_cm + R_r_ned[1][1] * east_cm;

}

/**
 * Convert Deg*E7 in rad.
 *
 * @param deg_e7_p  pointer to angle expressed in deg * E7.
 * @return          angle in rad.
*/
float degE7_to_rad(const int32_t  *deg_e7_p){

    return (((float)*deg_e7_p) / E7) * deg2rad;
}

/**
 * Set the new origin position of NED frame.
 *
 * @param _lat0_d_e7_p   pointer to latitude value of new origin, in degress * E7.
 * @param _lon0_d_e7_p   pointer to longitude, in degrees, in degress * E7.
 * @param _alt0_mm_p     pointer to altitude value of new origin, in millimeters
*/
void set_ref0(const int32_t *_lat0_d_e7_p, const int32_t *_lon0_d_e7_p, const int32_t *_alt0_mm_p){

    //convert lat0 and lon0 in degrees and then in rad, after that compute sinusoidal function used later
    cosPhi = (float)cos(degE7_to_rad(_lat0_d_e7_p));
    sinPhi = (float)sin(degE7_to_rad(_lat0_d_e7_p));
    cosLambda = (float)cos(degE7_to_rad(_lon0_d_e7_p));
    sinLambda = (float)sin(degE7_to_rad(_lon0_d_e7_p));

    //set ecef reference of NED origin
    geo_to_ecef(_lat0_d_e7_p, _lon0_d_e7_p, _alt0_mm_p, &x0_cm, &y0_cm, &z0_cm);
}

/**
 * Convert geodedical coordinate in NED coordinate
 *
 * @param gps_p         pointer to vehicle_gps_position struct.
 * @param north_cm_p    pointer to variable which will contain north coordinate, in centimeters.
 * @param east_cm_p     pointer to variable which will contain east coordinate, in centimeters.
 * @param down_cm_p     pointer to variable which will contain down coordinate, in centimeters.
*/
void geo_to_ned(const struct vehicle_global_position_s *gps_p,
                int32_t *north_cm_p, int32_t *east_cm_p, int32_t *down_cm_p){

    int32_t lat_d_e7;
    int32_t lon_d_e7;
    int32_t alt_mm;

    int32_t x_cm;
    int32_t y_cm;
    int32_t z_cm;

    lat_d_e7 = (int32_t)((gps_p->lat) * (double)E7);
    lon_d_e7 = (int32_t)((gps_p->lon) * (double)E7);
    alt_mm = (int32_t)((gps_p->alt) * E3);

    //compute ECEF coordinate of the actual gps position
    geo_to_ecef(&lat_d_e7, &lon_d_e7, &alt_mm, &x_cm, &y_cm, &z_cm);

    //compute NED position from ECEF coordinate
    ecef_to_ned(&x_cm, &y_cm, &z_cm, north_cm_p, east_cm_p, down_cm_p);
}

/**
 * Convert geodedical coordinate in ECEF coordinate
 *
 * @param lat_d_e7_p    pointer to latitude value, in degress * E7.
 * @param lon_d_e7_p    pointer to longitude value, in degress * E7.
 * @param alt_mm_p      pointer to altitude value, in millimeters.
 * @param x_cm_p        pointer to variable which will contain X coordinate in ECEF, in centimeters.
 * @param y_cm_p        pointer to variable which will contain Y coordinate in ECEF, in centimeters.
 * @param z_cm_p        pointer to variable which will contain Z coordinate in ECEF, in centimeters.
*/
void geo_to_ecef(const int32_t  *lat_d_e7_p, const int32_t  *lon_d_e7_p, const int32_t  *alt_mm_p,
                 int32_t  *x_cm_p, int32_t  *y_cm_p, int32_t  *z_cm_p){

    float mu_r;
    float l_r;
    float  h_m;

    float lab_s_r;
    float r_s_m;

    //convert geo data from degrees * E7 to rad
    mu_r = degE7_to_rad(lat_d_e7_p);
    l_r = degE7_to_rad(lon_d_e7_p);
    //from millimetrs to meters
    h_m = *alt_mm_p / E3;

    lab_s_r = atan2(squared_one_minus_flatness_m * tan(mu_r), 1.0);

    r_s_m = sqrt((squared_earth_radius_m) /
               (1 + ((1 / squared_one_minus_flatness_m) - 1) * (pow(sin(lab_s_r), 2))));

    //compute x, y and z and convert from meters in millimeters
    *x_cm_p = (int32_t) ((r_s_m * (float)cos(lab_s_r) * (float)cos(l_r) +
                          h_m * (float)cos(mu_r) * (float)cos(l_r)) * E2);

    *y_cm_p = (int32_t) ((r_s_m * (float)cos(lab_s_r) * (float)sin(l_r) +
                          h_m * (float)cos(mu_r) * (float)sin(l_r)) * E2);

    *z_cm_p = (int32_t) ((r_s_m * (float)sin(lab_s_r) +
                          h_m * (float)sin(mu_r)) * E2);
}

/**
 * Convert ECEF coordinate in NED coordinate
 *
 * @param x_cm_p           pointer to x coordinate in ECEF, in centimeters.
 * @param y_cm_p           pointer to y coordinate in ECEF, in centimeters.
 * @param z_cm_p           pointer to z coordinate in ECEF, in centimeters.
 * @param north_cm_p       pointer to variable which will contain north coordinate in NED, in centimeters.
 * @param east_cm_p        pointer to variable which will contain east coordinate in NED, in centimeters.
 * @param down_cm_p        pointer to variable which will contain down coordinate in NED, in centimeters.
*/
void ecef_to_ned(const int32_t *x_cm_p, const int32_t *y_cm_p, const int32_t *z_cm_p,
                 int32_t *north_cm_p, int32_t *east_cm_p, int32_t *down_cm_p){
    int32_t u_cm;
    int32_t v_cm;
    int32_t w_cm;
    float t_cm;

    u_cm = *x_cm_p - x0_cm;
    v_cm = *y_cm_p - y0_cm;
    w_cm = *z_cm_p - z0_cm;

    t_cm     =  cosLambda * u_cm + sinLambda * v_cm;

    //convert from ecef to ned
    *north_cm_p    = -sinPhi * t_cm + cosPhi * w_cm;

    *east_cm_p     = -sinLambda * u_cm + cosLambda * v_cm;

    *down_cm_p     = -cosPhi * t_cm - sinPhi * w_cm;
}

/** Set the mean wind angle with respect to true North.
 *
 * Compute the new rotation matrix that transforms NED coordinate in Race coordinate.
 *
 * @param mean_wind mean wind direction w.r.t. true North [rad], positive N. to E., negative N. to W.
*/
void set_mean_wind_angle(float mean_wind){

    /*
     * Compute the new R_r_ned matrix, it's a rotation matrix.
     * Pay attention: mean_wind is the angle between mean wind direction(wrt true North) and north axis.
     * The matrix rotation is not the "standard" form of rotation matrix about Z axis.
     * We're interested in only the transformation of north and east coordinates.
     *
     * The top mark represents the origin of the system reference frame trough which
     * two orthogonal axes pass; the X-axis is set in the average wind direction.
     * The Y-axis is defined so that the system is positively oriented.
    */

    //TODO: add traslation terms to move origin of race frame from NED origin to top mark!!!

    //first row, transform NED coordinates in x coordinate
    R_r_ned[0][0] = (float)(-cos(mean_wind));
    R_r_ned[0][1] = (float)sin(mean_wind);

    //second row, transform NED coordinates in y coordinate
    R_r_ned[1][0] = (float)sin(mean_wind);
    R_r_ned[1][1] = (float)cos(mean_wind);

}

