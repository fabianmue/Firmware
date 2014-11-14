#ifndef TOPICS_HANDLER_H
#define TOPICS_HANDLER_H

#include <uORB/uORB.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/actuator_armed.h>

#include <uORB/topics/wind_sailing.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/boat_weather_station.h>
#include <uORB/topics/vehicle_attitude.h>


struct subscribtion_fd_s{
    int att;
    int gps;
    int wind_sailing;
    int boat_weather_station;
};

struct published_fd_s{
        orb_advert_t actuator_pub;
};

struct structs_topics_s{
   struct actuator_controls_s actuators;
   struct vehicle_attitude_s att;
   struct vehicle_global_position_s gps_filtered;
   struct wind_sailing_s wind_sailing;
   struct boat_weather_station_s boat_weather_station;

};

struct parameters_qgc{
    float rudder_servo;
    float sail_servo;

    float p_gain;
    float i_gain;

    int32_t lat0;
    int32_t lon0;
    int32_t alt0;

    float epsilon;

    int32_t moving_window;
};

struct pointers_param_qgc{
    param_t sail_pointer;         /**< pointer to param AS_SAIL*/
    param_t rudder_pointer;       /**< pointer to param AS_RUDDER*/

    param_t p_gain_pointer;       /**< pointer to param AS_P_GAIN*/
    param_t i_gain_pointer;       /**< pointer to param AS_I_GAIN*/

    param_t lat0_pointer;         /**< pointer to param AS_LAT0*/
    param_t lon0_pointer;         /**< pointer to param AS_LON0*/
    param_t alt0_pointer;         /**< pointer to param AS_ALT0*/

    param_t epsilon_pointer;      /**< pointer to param AS_EPSI*/

    param_t moving_window_pointer;/**< pointer to param AS_WIN*/
};

struct apparent_angle_s{
    float *app_angle_p;
    int32_t window_size;
    int32_t oldest_value;
    float app_angle_mean;
};


#endif // TOPICS_HANDLER_H
