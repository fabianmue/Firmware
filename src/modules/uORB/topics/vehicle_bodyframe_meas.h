#ifndef VEHICLE_BODYFRAME_MEAS_H
#define VEHICLE_BODYFRAME_MEAS_H

#include <stdint.h>
#include "../uORB.h"

/** Body frame measurements from onboard sensors*/

struct vehicle_bodyframe_meas_s {

    uint64_t	timestamp;
    float acc_x;
    float acc_y;
    float acc_z;
};

ORB_DECLARE(vehicle_bodyframe_meas);

#endif // VEHICLE_BODYFRAME_MEAS_H
