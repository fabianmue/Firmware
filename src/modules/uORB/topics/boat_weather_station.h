#ifndef BOAT_WEATHER_STATION_H
#define BOAT_WEATHER_STATION_H

#include <stdint.h>
#include "../uORB.h"

/** Measurements from weather station 200WX */

struct boat_weather_station_s {

    uint64_t	timestamp;
    float acc_x_g;          ///Longitudinal acceleration, value reported in g
    float acc_y_g;          ///Latitudinal acceleration, value reported in g
    float acc_z_g;          ///Vertical acceleration, value reported in g
    float roll_r;           ///Roll in rad
    float pitch_r;          ///Pitch in rad
    float heading_tn;       ///Heading relative to the true north
    float roll_rate_r_s;    ///Roll rate in rad/s
    float pitch_rate_r_s;   ///Pitch rate in rad/s
    float yaw_rate_r_s;     ///Yaw rate in rad/s
};

ORB_DECLARE(boat_weather_station);

#endif // BOAT_WEATHER_STATION_H
