/*
 * topics_handler.c
 *
 *  Created on: 04.03.2015
 *      Author: Jonas
 */

#include "pp_topics_handler.h"



/**
 * The uORB Topic "path_planning" is the interface between the modules "autonomous_sailing" and
 * "path_planning".
 * This is the declaration of the topic and registers it in uORB.
 */
ORB_DEFINE(path_planning, struct path_planning_s);


static struct published_fd_s pubs;      //File-Descriptors of published topics



/**
 * Subscribe to all Topics in the Topic-Subscription Struct
 *
 * @param *subs_p: Pointer to a struct of all File-Descriptors
 * @param *strs_p: Pointer to a struct of all interested Topics
 * @return true, iff successfully subscribed to all topics
 */
bool th_subscribe(struct subscribtion_fd_s *subs_p, struct structs_topics_s *strs_p) {

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



/**
 * Advertise Topics
 *
 * @param *subs_p: Pointer to a struct of all File-Descriptors
 * @param *strs_p: Pointer to a struct of all interested Topics
 * @return true, iff successfully subscribed to all topics
 */
bool th_advertise(struct structs_topics_s *strs_p) {

	//Advertise the Pathplanning topic
	memset(&(strs_p->path_planning), 0, sizeof(strs_p->path_planning));
	pubs.path_planning = orb_advertise(ORB_ID(path_planning), &(strs_p->path_planning));

	return true;
}


/**
 * Update Pathplanning Topic
 *
 *  @param pp_p: Pointer to a path_planning Struct
 *	@return true, if successfully updated
 */
bool th_update_pathplanning(float heading, bool tack, bool gybe) {

	//Create new Struct with data
	struct path_planning_s newPPS = {
			.heading_ref = heading,
			.tack = tack,
			.gybe = gybe
	};

	//Publish the new values
	orb_publish(ORB_ID(path_planning), pubs.path_planning, &newPPS);

	return true;
}


