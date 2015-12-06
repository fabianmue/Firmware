/*
 * mp_topics_handler.c
 *
 *  Created on: 04.12.2015
 *      Author: Fabian
 */

#include "mp_topics_handler.h"
#include "../path_planning/pp_send_msg_qgc.h"

// struct of all topic-advertisements
static struct {
    orb_advert_t mission_planning;     	//output of mission_planning topic
} pubs;

/**
 * subscribe to all topics in the topic-subscription struct
 *
 * @param *subs_p: pointer to a struct of all file-descriptors
 * @param *strs_p: pointer to a struct of all interested topics
 * @return true, iff successfully subscribed to all topics
 */
bool th_subscribe(struct subscribtion_fd_s *subs_p, struct structs_topics_s *strs_p) {

	subs_p->parameter_update = orb_subscribe(ORB_ID(parameter_update));

    if(subs_p->parameter_update == -1){
        warnx(" error on subscribing on parameter_update Topic \n");
        return false;
    }

    //Subscription to all topics was successful
    warnx(" subscribed to all topics \n");
    return true;
};

/**
 * advertise topics. call this function only once.
 *
 * @param *subs_p: pointer to a struct of all file-descriptors
 * @param *strs_p: pointer to a struct of all interested topics
 * @return true, iff successfully subscribed to all topics
 */
bool th_advertise(void) {

	// advertise the mission_planning topic
    struct mission_planning_s mission_planning;
    memset(&mission_planning, 0, sizeof(mission_planning));
    pubs.mission_planning = orb_advertise(ORB_ID(mission_planning), &mission_planning);

    return true;
};

/**
 * publish mission_planning topic.
 *
 * @param mission_planning_p: pointer to a mission_planning_s struct
 * @return return value from orb_publish()
 */
int th_publish_mission_planning(const struct mission_planning_s *mission_planning_p) {

    return orb_publish(ORB_ID(mission_planning), pubs.mission_planning, mission_planning_p);
};
