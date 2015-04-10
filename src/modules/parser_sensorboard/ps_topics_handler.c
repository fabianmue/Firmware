/*
 * topics_handler.c
 *
 *  Created on: 04.03.2015
 *      Author: Jonas
 */

#include "pp_topics_handler.h"


//Stuct of all topic-Advertisements
static struct{
    //orb_advert_t path_planning;        //output of path_planning topic
}pubs;

/**
 * Subscribe to all Topics in the Topic-Subscription Struct
 *
 * @param *subs_p: Pointer to a struct of all File-Descriptors
 * @param *strs_p: Pointer to a struct of all interested Topics
 * @return true, iff successfully subscribed to all topics
 */
bool th_subscribe(struct subscribtion_fd_s *subs_p, struct structs_topics_s *strs_p) {

	//Subscribe to the topics
	subs_p->path_planning = orb_subscribe(ORB_ID(path_planning));

	//Check correct subscription
    if(subs_p->path_planning == -1){
        warnx(" error on subscribing on path_planning Topic \n");
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
bool th_advertise(void) {

	//Advertise the Pathplanning topic
    //struct path_planning_s path_planning;
    //memset(&path_planning, 0, sizeof(path_planning));
    //pubs.path_planning = orb_advertise(ORB_ID(path_planning), &path_planning);



	return true;
}




