/*
 * topics_handler.c
 *
 *  Created on: 04.03.2015
 *      Author: Jonas
 */

#include "pp_topics_handler.h"


/**
 * Subscribe to all Topics in the Topic-Subscription Struct
 *
 * @param *subs_p: Pointer to a struct of all File-Descriptors
 * @param *strs_p: Pointer to a struct of all interested Topics
 * @return true, iff successfully subscribed to all topics
 */
bool pp_th_subscribe(struct subscribtion_fd_s *subs_p, struct structs_topics_s *strs_p) {

	//Subscribe to the topics
	subs_p->vehicle_global_position = orb_subscribe(ORB_ID(vehicle_global_position));
	subs_p->wind_estimate = orb_subscribe(ORB_ID(wind_estimate));
	subs_p->parameter_update = orb_subscribe(ORB_ID(parameter_update));
	subs_p->boat_weather_station = orb_subscribe(ORB_ID(boat_weather_station));
	subs_p->rc_channels = orb_subscribe(ORB_ID(rc_channels));


	//Check correct subscription
    if(subs_p->vehicle_global_position == -1){
        warnx(" error on subscribing on vehicle_global_position Topic \n");
        return false;
    }

    if(subs_p->wind_estimate == -1){
        warnx(" error on subscribing on wind_estimate Topic \n");
        return false;
    }

    if(subs_p->parameter_update == -1){
        warnx(" error on subscribing on parameter_update Topic \n");
        return false;
    }

    if(subs_p->boat_weather_station == -1){
        warnx(" error on subscribing on boat_weather_station Topic \n");
        return false;
    }

    if(subs_p->rc_channels == -1){
        warnx(" error on subscribing on rc_channels Topic \n");
        return false;
    }


    //Subscription to all topics was successful
    warnx(" subscribed to all topics \n");
    return true;
}


