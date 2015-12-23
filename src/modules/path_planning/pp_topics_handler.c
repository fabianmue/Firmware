/*
 * topics_handler.c
 *
 *  Created on: 04.03.2015
 *      Author: Jonas
 */

#include "pp_topics_handler.h"
#include "pp_send_msg_qgc.h"


//Struct of all topic-Advertisements
static struct {
    orb_advert_t path_planning;        //output of path_planning topic
    orb_advert_t boat_qgc_param2;      //QGC paramters for path_planning
    orb_advert_t boat_local_position; // local position of the boat
    orb_advert_t boat_pp_debug1;		//Data from Pathplanning (Costfunction)
    orb_advert_t mi_ack;
} pubs;

/**
 * Subscribe to all Topics in the Topic-Subscription Struct
 *
 * @param *subs_p: Pointer to a struct of all File-Descriptors
 * @param *strs_p: Pointer to a struct of all interested Topics
 * @return true, iff successfully subscribed to all topics
 */
bool pp_th_subscribe(struct pp_subscribtion_fd_s *subs_p, struct pp_structs_topics_s *strs_p) {

	//Subscribe to the topics
	subs_p->vehicle_global_position = orb_subscribe(ORB_ID(vehicle_global_position));
	subs_p->parameter_update = orb_subscribe(ORB_ID(parameter_update));
    subs_p->boat_guidance_debug = orb_subscribe(ORB_ID(boat_guidance_debug));
    subs_p->rc_channels = orb_subscribe(ORB_ID(rc_channels));
    subs_p->vehicle_attitude = orb_subscribe(ORB_ID(vehicle_attitude));
    subs_p->mission_planning = orb_subscribe(ORB_ID(mission_planning));

    //Set update intervals => helps keeping the processor-load low
    //orb_set_interval(subs_p->vehicle_global_position, 500);
    //orb_set_interval(subs_p->parameter_update, 500);
    //orb_set_interval(subs_p->boat_guidance_debug, 500);
    //orb_set_interval(subs_p->rc_channels,500);
    //orb_set_interval(subs_p->vehicle_attitude, 500);


	//Check correct subscription
    if(subs_p->vehicle_global_position == -1){
        warnx(" error on subscribing on vehicle_global_position topic \n");
        return false;
    }

    if(subs_p->parameter_update == -1){
        warnx(" error on subscribing on parameter_update topic \n");
        return false;
    }

    if(subs_p->boat_guidance_debug == -1){
        warnx(" error on subscribing on boat_guidance_debug topic \n");
        return false;
    }

    if(subs_p->rc_channels == -1){
        warnx(" error on subscribing on rc_channels topic \n");
        return false;
    }

    if(subs_p->vehicle_attitude == -1){
        warnx(" error on subscribing on vehicle_attitude topic \n");
        return false;
    }

    if(subs_p->mission_planning == -1){
        warnx(" error on subscribing on mission_planning topic \n");
        return false;
    }

    //Subscription to all topics was successful
    warnx(" subscribed to all topics \n");
    return true;
}



/**
 * Advertise Topics. Call this function only once.
 *
 * @param *subs_p: Pointer to a struct of all File-Descriptors
 * @param *strs_p: Pointer to a struct of all interested Topics
 * @return true, iff successfully subscribed to all topics
 */
bool pp_th_advertise(void) {

	//Advertise the path_planning topic
    struct path_planning_s path_planning;
    memset(&path_planning, 0, sizeof(path_planning));
    pubs.path_planning = orb_advertise(ORB_ID(path_planning), &path_planning);

	//Advertise the mi_ack topic
    struct mi_ack_s mi_ack;
    memset(&mi_ack, 0, sizeof(mi_ack));
    pubs.mi_ack = orb_advertise(ORB_ID(mi_ack), &mi_ack);

    //Advertise boat_qgc_param2 topic
    struct boat_qgc_param2_s boat_qgc_param2;
    memset(&boat_qgc_param2, 0, sizeof(boat_qgc_param2));
    pubs.boat_qgc_param2 = orb_advertise(ORB_ID(boat_qgc_param2), &boat_qgc_param2);

    //Advertise boat_loca_position topic
    struct boat_local_position_s boat_local_position;
    memset(&boat_local_position, 0, sizeof(boat_local_position));
    pubs.boat_local_position = orb_advertise(ORB_ID(boat_local_position), &boat_local_position);

    //Advertise boat_pp_debug1 topic
    struct boat_pp_debug1_s boat_pp_debug1;
    memset(&boat_pp_debug1, 0, sizeof(boat_pp_debug1));
    pubs.boat_pp_debug1 = orb_advertise(ORB_ID(boat_pp_debug1), &boat_pp_debug1);

	return true;
}


/**
 * Publish path_planning topic.
 *
 * @param path_planning_p: Pointer to a path_planning_s struct
 * @return return value from orb_publish()
 */
int pp_th_publish(const struct path_planning_s *path_planning_p) {

    return orb_publish(ORB_ID(path_planning), pubs.path_planning, path_planning_p);
}

/**
 * Publish mi_ack topic.
 *
 * @param mi_ack: Pointer to a mi_ack struct
 * @return return value from orb_publish()
 */
int pp_th_publish_mi_ack(const struct mi_ack_s *mi_ack_p) {

    return orb_publish(ORB_ID(mi_ack), pubs.mi_ack, mi_ack_p);
}

/**
 * Publish topic boat_qgc_param2.
 *
 * @param boat_qgc_param2_p: Pointer to a boat_qgc_param2_s struct
 * @return return value from orb_publish()
 */
int pp_th_publish_qgc2(const struct boat_qgc_param2_s *boat_qgc_param2_p){

    return orb_publish(ORB_ID(boat_qgc_param2), pubs.boat_qgc_param2, boat_qgc_param2_p);
}

/**
 * Publish topic boat_local_position.
 *
 * @param boat_local_position_p: Pointer to a boat_local_position_s struct
 * @return return value from orb_publish()
 */
int pp_th_publish_boat_local_position(const struct boat_local_position_s *boat_local_position_p){

    return orb_publish(ORB_ID(boat_local_position),
                       pubs.boat_local_position,
                       boat_local_position_p);
}

/**
 * Publish topic boat_pp_debug1
 *
 * @param boat_pp_debug1: Pointer to the boat_pp_debug1_s struct
 * @return return value from orb_publish()
 */
int pp_th_publish_boat_debug1(const struct boat_pp_debug1_s *boat_pp_debug1_p) {
	return orb_publish(ORB_ID(boat_pp_debug1), pubs.boat_pp_debug1, boat_pp_debug1_p);
}
