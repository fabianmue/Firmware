#ifndef TOPICS_HANDLER_H
#define TOPICS_HANDLER_H

#include <uORB/uORB.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/actuator_armed.h>

#include <uORB/topics/wind_sailing.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/boat_weather_station.h>
#include <uORB/topics/vehicle_gps_position.h>
#include <uORB/topics/boat_guidance_debug.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/debug_values.h>


//only for debug on qcg
#include <uORB/topics/airspeed.h>

struct subscribtion_fd_s{
    int gps_raw;
    int gps_filtered;
    int wind_sailing;
    int parameter_update;
    int att;
    int boat_weather_station;
};

struct published_fd_s{
    orb_advert_t actuator_pub;
    orb_advert_t boat_guidance_debug_pub;
    orb_advert_t debug_values;

    //only for debug on qGC
    orb_advert_t airspeed;
};

struct structs_topics_s{
   struct actuator_controls_s actuators;
   struct vehicle_gps_position_s gps_raw;
   struct vehicle_global_position_s gps_filtered;
   struct wind_sailing_s wind_sailing;
   struct boat_guidance_debug_s boat_guidance_debug;
   struct parameter_update_s update;
   struct vehicle_attitude_s att;
   struct boat_weather_station_s boat_weather_station;
   struct debug_values_s debug_values;

   bool debug_updated;

   //only for debug on qGC
   struct airspeed_s airspeed;
};




#endif // TOPICS_HANDLER_H
