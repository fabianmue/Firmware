#ifndef TOPICS_HANDLER_H
#define TOPICS_HANDLER_H

// for uORB topics
#include <uORB/uORB.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/airspeed.h>
#include <uORB/topics/wind_sailing.h>
#include <uORB/topics/vehicle_gps_position.h>
#include <uORB/topics/vehicle_bodyframe_meas.h>
#include <uORB/topics/debug_values.h>


struct subscribtion_fd_s{
    int sensor_sub;
};

struct published_fd_s{
    int att_pub;
    int gps_pub;
    int wind_sailing;
    int bodyframe_meas;
};

struct structs_topics_s{
    struct vehicle_attitude_s att_s;
    struct vehicle_gps_position_s  gps_s;
    struct wind_sailing_s wind_sailing_s;
    struct vehicle_bodyframe_meas_s bodyframe_meas_s;
};

#endif // TOPICS_HANDLER_H
