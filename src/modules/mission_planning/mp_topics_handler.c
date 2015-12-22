/*
 * mp_topics_handler.c
 *
 *  Created on: 04.12.2015
 *      Author: Fabian
 */

#include "mp_topics_handler.h"
#include "mp_send_msg_qgc.h"

/***********************************************************************************/
/*****  V A R I A B L E S  *********************************************************/
/***********************************************************************************/

// struct of all topic-advertisements
static struct {
    orb_advert_t mission_planning;     	//output of mission_planning topic
} pubs;

/***********************************************************************************/
/*****  F U N C T I O N   D E F I N I T I O N S  ***********************************/
/***********************************************************************************/

/**
 * subscribe to all topics in the topic-subscription struct
 *
 * @param *subs_p: pointer to a struct of all file-descriptors
 * @param *strs_p: pointer to a struct of all interested topics
 * @return true, iff successfully subscribed to all topics
 */
bool mp_th_subscribe(struct mp_subscribtion_fd_s *subs_p, struct mp_structs_topics_s *strs_p) {

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
bool mp_th_advertise(void) {

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
int mp_th_publish(const struct mission_planning_s *mission_planning_p) {

    return orb_publish(ORB_ID(mission_planning), pubs.mission_planning, mission_planning_p);
};
