/*
 * topics_handler.h
 *
 *  Created on: 04.03.2015
 *      Author: Jonas
 */

#ifndef TOPICS_HANDLER_H_
#define TOPICS_HANDLER_H_

//Struct of all topic-Subscriptions
struct subscribtion_fd_s{
    int vehicle_global_position;			//Contains the filtered GPS-Data
    int wind_estimate;						//Estimate of True Windspeed and Direction
    int parameter_update;					//Update of Parameters from QGroundControl
    int boat_weather_station;				//Provides Heading-Information wrt. true North
    int rc_channels;						//RC-Channels (probably used for detecting changes in Switch-States)
};


#endif /* TOPICS_HANDLER_H_ */
