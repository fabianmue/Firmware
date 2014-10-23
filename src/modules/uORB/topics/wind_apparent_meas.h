#ifndef WIND_APPARENT_MEAS_H
#define WIND_APPARENT_MEAS_H

#include <stdint.h>
#include "../uORB.h"

/** apparent wind measuremetns */
struct wind_apparent_meas_s {
    uint64_t	timestamp;
    float angle_meas; ///< measured wind angle relative to the vessel, to the nearest 0.1 degree
    float speed_m_s;  ///< measured wind speed in m/s, to the nearest 0.1 m/s
};

ORB_DECLARE(wind_apparent_meas);

#endif // WIND_APPARENT_MEAS_H
