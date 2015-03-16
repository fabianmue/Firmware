#ifndef BOAT_LOCAL_POSITION_H
#define BOAT_LOCAL_POSITION_H

#include <stdint.h>
#include "../uORB.h"


struct boat_local_position_s {
    uint64_t timestamp;    /**< Time of the last Pathplanning Update since System Start in Microseconds */
    float    x_race_m;    /**< X coordinate in race frame, [m]*/
    float    y_race_m;    /**< Y coordinate in race frame, [m]*/
    float    dist_m;      /**< Distance from the top mark, [m] */
};


/* register this as object request broker structure */
ORB_DECLARE(boat_local_position);

#endif // BOAT_LOCAL_POSITION_H
