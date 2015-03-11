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
 * @file pp_navigation_module.c
 *
 * Computes NED position from geodedical information.
 *
 * @author Marco Tranzatto <marco.tranzatto@gmail.com>
 */

#include "pp_navigation_module.h"

//WGS84 data

static const double squared_one_minus_flatness_m = 0.99330561993959; ///(1-flatness)^2 in meters

static const double  squared_earth_radius_m = 40680631590769; ///(arth_radius)^2 in meters

static const float deg2rad = 0.0174532925199433f; // pi / 180

static float cosPhi = 0.0f;    ///cos(lat0)

static float sinPhi = 0.0f;    ///sin(lat0)

static float cosLambda = 0.0f; ///cos(lon0)

static float sinLambda = 0.0f; ///sin(lon0)

static int32_t x0_dm = 0; ///x coordinate in ECEF of origin of NED system, in decimeters

static int32_t y0_dm = 0; ///y coordinate in ECEF of origin of NED system, in decimeters

static int32_t z0_dm = 0; ///z coordinate in ECEF of origin of NED system, in decimeters

static const float E7 = 10000000.0f;

static const float E3 = 1000.0f;

//static const float E2 = 100.0f;

static const float E1 = 10.0f;

//rotation matrix from body frame to NED frame
static float R_ned_body[3][3] = {{0.0f, 0.0f, 0.0f},
                                {0.0f, 0.0f, 0.0f},
                                {0.0f, 0.0f, 0.0f}};
//velocities in NEd frame
static float vel_ned[3] = {0.0f, 0.0f, 0.0f};

//temp struct
static struct vehicle_global_position_s vehicle_global_position;

static struct{
    float sin_mwd;  ///sin(mean wind direction)
    float cos_mwd;  ///cos(mean wind direction)
    int32_t n0_dm;  ///north distance of origin of race frame from NED origin [dm]
    int32_t e0_dm;  ///east distance of origin of race frame from NED origin [dm]
    float mean_wind_angle_r;///mean wind angle set by QGC
}ned_to_race_s;


/** @brief convert geodedical coordinates into NED coordinate.*/
void geo_to_ned(const struct vehicle_global_position_s *gps_p,
                int32_t *north_dm_p, int32_t *east_dm_p, int32_t *down_dm_p);

/** @brief convert geodedical coordinates into ECEF coordinate.*/
void geo_to_ecef(const int32_t  *lat_d_e7_p, const int32_t  *lon_d_e7_p, const int32_t  *alt_mm_p,
                 int32_t  *x_dm_p, int32_t  *y_dm_p, int32_t  *z_dm_p);

/** @brief convert ECEF coordinates into NED coordinate.*/
void ecef_to_ned(const int32_t *x_dm_p, const int32_t *y_dm_p, const int32_t *z_dm_p,
                 int32_t *north_dm_p, int32_t *east_dm_p, int32_t *down_dm_p);

/** @brief convert Deg*E7 in rad */
float degE7_to_rad(const int32_t  *deg_e7_p);

/** @brief transform geodedical coordinate in race frame coordinate*/
void geo_to_race(const struct vehicle_global_position_s *gps_p,
                 int32_t *x_dm_p, int32_t *y_dm_p);


/**
 * Convert geodedical coordinate in race frame coordinate.
 *
 * First step: convert geodedical coordinates in NED coordinates using lat0, lon0 and alt0 set by set_ref0().
 * Second step: use the mean wind angle set by set_mean_wind_angle()
 * for the rotation matrix from NED to Race frame.
 *
 * @param vehicle_global_position_s pointer to struct with gps filtered data.
 * @param x_dm_p                    pointer to returned value with x coordinate in race frame, decimeters
 * @param x_dm_p                    pointer to returned value with x coordinate in race frame, decimeters
 */
void geo_to_race(const struct vehicle_global_position_s *gps_p,
                 int32_t *x_dm_p, int32_t *y_dm_p){

    int32_t north_dm;
    int32_t east_dm;
    int32_t down_dm;


    //compute boat position in NED frame w.r.t. lat0 lon0 alt0 set by set_ref0()
    geo_to_ned(gps_p, &north_dm, &east_dm, &down_dm);

    /** Race frame has X-axis oriented as the wind direction,
     * Y-axis is defined so that the system is positively oriented.
     *
     * To transform NED coordinate in Race frame coordinate we need a rotation, a translation and
     * a "reflection".
     *
     * Here we compute the final boat coordinate in Race frame by performing:
     *
     * 1)rotation from NED frame in Race' frame (defined in set_mean_wind_angle),
     * using the rotation matrix (Matlab form)
     *
     *      R_r'_ned =  [cos(mea_wind)   sin(mean_wind);
     *                   -sin(mean_wind) cos(mean_wind)];
     *
     * then
     * 2) we can write the homogeneous transformation matrix to convert coordinate
     * from NED frame to Race' frame using p0 = [n0_dm; e0_dm]
     *
     *      T_r'_ned = [R_r'_ned    |   -R_r'_ned * p0
     *                  0           |   1               ]
     *
     * then we change the direction of Race' x-axis in order to have the new x-axis
     * oriented as the wind direction. At this poin we have the Race frame.
    */

    //transform [north, east] coordinate from NED frame in [x, y] coordinate in Race frame
    *x_dm_p = -ned_to_race_s.cos_mwd * north_dm - ned_to_race_s.sin_mwd * east_dm +
            ned_to_race_s.cos_mwd * ned_to_race_s.n0_dm + ned_to_race_s.sin_mwd * ned_to_race_s.e0_dm;

    *y_dm_p = -ned_to_race_s.sin_mwd * north_dm + ned_to_race_s.cos_mwd * east_dm +
            ned_to_race_s.sin_mwd * ned_to_race_s.n0_dm - ned_to_race_s.cos_mwd * ned_to_race_s.e0_dm;


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
void n_set_ref0(const int32_t *_lat0_d_e7_p, const int32_t *_lon0_d_e7_p, const int32_t *_alt0_mm_p){

    //convert lat0 and lon0 in degrees and then in rad, after that compute sinusoidal function used later
    cosPhi = (float)cos(degE7_to_rad(_lat0_d_e7_p));
    sinPhi = (float)sin(degE7_to_rad(_lat0_d_e7_p));
    cosLambda = (float)cos(degE7_to_rad(_lon0_d_e7_p));
    sinLambda = (float)sin(degE7_to_rad(_lon0_d_e7_p));

    //set ecef reference of NED origin
    geo_to_ecef(_lat0_d_e7_p, _lon0_d_e7_p, _alt0_mm_p, &x0_dm, &y0_dm, &z0_dm);
}

/**
 * Convert geodedical coordinate in NED coordinate
 *
 * @param gps_p         pointer to vehicle_gps_position struct.
 * @param north_dm_p    pointer to variable which will contain north coordinate, in decimeters.
 * @param east_dm_p     pointer to variable which will contain east coordinate, in decimeters.
 * @param down_dm_p     pointer to variable which will contain down coordinate, in decimeters.
*/
void geo_to_ned(const struct vehicle_global_position_s *gps_p,
                int32_t *north_dm_p, int32_t *east_dm_p, int32_t *down_dm_p){

    int32_t lat_d_e7;
    int32_t lon_d_e7;
    int32_t alt_mm;

    int32_t x_dm;
    int32_t y_dm;
    int32_t z_dm;

    lat_d_e7 = (int32_t)((gps_p->lat) * (double)E7);
    lon_d_e7 = (int32_t)((gps_p->lon) * (double)E7);
    alt_mm = (int32_t)((gps_p->alt) * E3);

    //compute ECEF coordinate of the actual gps position
    geo_to_ecef(&lat_d_e7, &lon_d_e7, &alt_mm, &x_dm, &y_dm, &z_dm);

    //compute NED position from ECEF coordinate
    ecef_to_ned(&x_dm, &y_dm, &z_dm, north_dm_p, east_dm_p, down_dm_p);

    //save NED coordinates in communication_buffer
    cb_set_ned_coordinates(*north_dm_p, *east_dm_p, *down_dm_p);
}

/**
 * Convert geodedical coordinate in ECEF coordinate
 *
 * @param lat_d_e7_p    pointer to latitude value, in degress * E7.
 * @param lon_d_e7_p    pointer to longitude value, in degress * E7.
 * @param alt_mm_p      pointer to altitude value, in millimeters.
 * @param x_dm_p        pointer to variable which will contain X coordinate in ECEF, in decimeters.
 * @param y_dm_p        pointer to variable which will contain Y coordinate in ECEF, in decimeters.
 * @param z_dm_p        pointer to variable which will contain Z coordinate in ECEF, in decimeters.
*/
void geo_to_ecef(const int32_t  *lat_d_e7_p, const int32_t  *lon_d_e7_p, const int32_t  *alt_mm_p,
                 int32_t  *x_dm_p, int32_t  *y_dm_p, int32_t  *z_dm_p){

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

    //compute x, y and z and convert them from meters to decimeters
    *x_dm_p = (int32_t) ((r_s_m * cosf(lab_s_r) * cosf(l_r) +
                          h_m * cosf(mu_r) * cosf(l_r)) * E1);

    *y_dm_p = (int32_t) ((r_s_m * cosf(lab_s_r) * sinf(l_r) +
                          h_m * cosf(mu_r) * sinf(l_r)) * E1);

    *z_dm_p = (int32_t) ((r_s_m * sinf(lab_s_r) +
                          h_m * sinf(mu_r)) * E1);
}

/**
 * Convert ECEF coordinate in NED coordinate
 *
 * @param x_dm_p           pointer to x coordinate in ECEF, in decimeters.
 * @param y_dm_p           pointer to y coordinate in ECEF, in decimeters.
 * @param z_dm_p           pointer to z coordinate in ECEF, in decimeters.
 * @param north_dm_p       pointer to variable which will contain north coordinate in NED, in decimeters.
 * @param east_dm_p        pointer to variable which will contain east coordinate in NED, in decimeters.
 * @param down_dm_p        pointer to variable which will contain down coordinate in NED, in decimeters.
*/
void ecef_to_ned(const int32_t *x_dm_p, const int32_t *y_dm_p, const int32_t *z_dm_p,
                 int32_t *north_dm_p, int32_t *east_dm_p, int32_t *down_dm_p){
    int32_t u_dm;
    int32_t v_dm;
    int32_t w_dm;
    float t_dm;

    u_dm = *x_dm_p - x0_dm;
    v_dm = *y_dm_p - y0_dm;
    w_dm = *z_dm_p - z0_dm;

    t_dm     =  cosLambda * u_dm + sinLambda * v_dm;

    //convert from ecef to ned
    *north_dm_p    = -sinPhi * t_dm + cosPhi * w_dm;

    *east_dm_p     = -sinLambda * u_dm + cosLambda * v_dm;

    *down_dm_p     = -cosPhi * t_dm - sinPhi * w_dm;
}

/** Set the mean wind angle with respect to true North.
 *
 * Compute the new values to transform NED coordinate in Race coordinate.
 *
 * @param mean_wind mean wind direction w.r.t. true North [rad], positive N. to E., negative N. to W.
*/
void n_set_mean_wind_angle(float mean_wind){

    /** Race frame has X-axis oriented as the wind direction,
     * Y-axis is defined so that the system is positively oriented.
     *
     * To transform NED coordinate in Race frame coordinate we need a rotation, a translation and
     * a "reflection".
     *
     * Here we compute values needed to perform a rotation that transform coordinate to the
     * NED frame into an middle frame Race'. This frame is rotated wrt NED frame by
     * the angle mean_wind about the down axis of the NED frame.
    */

    ned_to_race_s.cos_mwd = cosf(mean_wind);
    ned_to_race_s.sin_mwd = sinf(mean_wind);
    ned_to_race_s.mean_wind_angle_r = mean_wind;

}

/**
 * get the mean wind angle with respect to true North.
*/
float n_get_mean_wind_angle(void){
    return ned_to_race_s.mean_wind_angle_r;
}

/** Set the position of the top mark.
 *
 * Compute the new values to transform NED coordinate in Race coordinate.
 *
 * @param lat_d_e7_p    pointer to latitude value, in degress * E7.
 * @param lon_d_e7_p    pointer to longitude value, in degress * E7.
 * @param alt_mm_p      pointer to altitude value, in millimeters.
*/
void n_set_pos_top_mark(const int32_t *lat_d_e7_p, const int32_t *lon_d_e7_p, const int32_t *alt_mm_p){

    int32_t north_dm;
    int32_t east_dm;
    int32_t down_dm;

    /*set lat, lon and alt in temp_pos,
     * remember to save lon and lat in degrees! (not degrees * E7).
     * Save alt in meters.
    */
    vehicle_global_position.lat = ((double) *lat_d_e7_p) / (double)E7;
    vehicle_global_position.lon = ((double) *lon_d_e7_p) / (double)E7;
    vehicle_global_position.alt =  (*alt_mm_p) / E3;

    geo_to_ned(&vehicle_global_position, &north_dm, &east_dm, &down_dm);

    /** Race frame has X-axis oriented as the wind direction,
     * Y-axis is defined so that the system is positively oriented.
     *
     * To transform NED coordinate in Race frame coordinate we need a rotation, a translation and
     * a "reflection".
     *
     * Here we compute values needed to perform a translation that moves the origin of frame Race'
     * (defined in set_mean_wind_angle()) from the origin of NED frame to the top mark.
    */

    ned_to_race_s.n0_dm = north_dm;
    ned_to_race_s.e0_dm = east_dm;
}

/**
 * Compute from vehicle_global_position topic the boat's position in Race frame.
 * Set the new X and Y coordinates in the pp_communication_buffer module.
 *
*/
void n_navigation_module(const struct vehicle_global_position_s *vgp_p){

    int32_t x_dm;
    int32_t y_dm;

    //convert gps filtered position in race frame coordinates
    geo_to_race(vgp_p, &x_dm, &y_dm);

    //convert local position from decimeters to meters and save it in path_planning struct
    cb_set_race_coordinates((float)x_dm / E1, (float)y_dm / E1);
}

/**
 * Update the rotation matrix that converts coordinates from body frame to NED frame.
*/
void n_update_r_ned_body(float R[3][3], bool valid_matrix){
    if(valid_matrix){
        for(int8_t i = 0; i < 3; i++){
            for(int8_t j = 0; j < 3; j++){
                R_ned_body[i][j] = R[i][j];
            }
        }
    }
}

/**
 * Update velocities in the NEd frame
*/
void n_update_ned_vel(float vn, float ve, float vd){
    vel_ned[0] = vn;
    vel_ned[1] = ve;
    vel_ned[2] = vd;
}

/**
 * Get longitudinal velocity in the body frame
*/
float n_get_u_vel(void){

    /* Use R_ned_body to rotate vel_ned from NED frame to body frame.
     * Rememebr that R_ned_body conerts coordinates from body frame to NED frame.
     * So we mast transponse it, that is, using its coloumns.
    */
    float u;

    u = R_ned_body[0][0] * vel_ned[0] + R_ned_body[1][0] * vel_ned[1] + R_ned_body[2][0] * vel_ned[2];

    return u;
}

/**
 * Convert geodedical coordinates (latitude, longitude, altitude) in race frame coordinates.
 *
 * @param lat_deg   latitude in degrees
 * @param lon_deg   longitude in degrees
 * @param alt       altitude in meters
 * @param x_dm_p    *x_dm_p <-- X coordinate in decimeters
 * @param y_dm_p    *x_dm_p <-- Y coordinate in decimeters
*/
void n_geo_to_race(double lat_deg, double lon_deg, float alt_m,
                   int32_t *x_dm_p, int32_t *y_dm_p){
    //fill vehicle_global_position struct
    vehicle_global_position.lat = lat_deg;
    vehicle_global_position.lon = lon_deg;
    vehicle_global_position.alt = alt_m;
    //convert geo to race frame
    geo_to_race(&vehicle_global_position, x_dm_p, y_dm_p);
}
