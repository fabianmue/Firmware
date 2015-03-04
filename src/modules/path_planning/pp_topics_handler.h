/*
 * topics_handler.h
 *
 *  Created on: 04.03.2015
 *      Author: Jonas
 */

#ifndef TOPICS_HANDLER_H_
#define TOPICS_HANDLER_H_

#include <uORB/uORB.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/wind_estimate.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/boat_weather_station.h>
#include <uORB/topics/rc_channels.h>

#include <stdbool.h>
#include <systemlib/err.h>



//Structs of all topic-Subscriptions
struct subscribtion_fd_s{
    int vehicle_global_position;			//Contains the filtered GPS-Data
    int wind_estimate;						//Estimate of True Windspeed and Direction
    int parameter_update;					//Update of Parameters from QGroundControl
    int boat_weather_station;				//Provides Heading-Information wrt. true North
    int rc_channels;						//RC-Channels (probably used for detecting changes in Switch-States)
};



//Structs of interested topics
struct structs_topics_s{
	struct vehicle_global_position_s vehicle_global_position;
	struct wind_estimate_s wind_estimate;
	struct parameter_update_s parameter_update;
	struct boat_weather_station_s boat_weather_station;
	struct rc_channels_s rc_channels;
};


/* @brief Subscribe to interested Topics*/
bool pp_th_subscribe(struct subscribtion_fd_s *subs_p, struct structs_topics_s *strs_p);



#endif /* TOPICS_HANDLER_H_ */
