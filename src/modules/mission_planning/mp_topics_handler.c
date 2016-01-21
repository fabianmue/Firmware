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

// mission planning
struct mission_planning_s mission_planning = { 	.mi_id = 0,
												.wp_lat = 0.0f,
												.wp_lon = 0.0f,
												.wp_count = 0,
												.ob_lat = 0.0f,
												.ob_lon = 0.0f,
												.ob_rad = 0.0f,
												.ob_count = 0,
};
struct mp_published_fd_s pubs;

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
bool mp_th_subscribe(struct mp_subscribtion_fd_s *subs_p) {

	subs_p->parameter_update = orb_subscribe(ORB_ID(parameter_update));
	subs_p->mi_ack = orb_subscribe(ORB_ID(mi_ack));

    if(subs_p->parameter_update == -1){
        warnx("error on subscribing on parameter_update topic \n");
        return false;
    }

    if(subs_p->mi_ack == -1){
        warnx("error on subscribing on path_planning topic \n");
        return false;
    }

    //Subscription to all topics was successful
    warnx("subscribed to all topics \n");
    return true;
};

/**
 * advertise mission planning topic. call this function only once.
 *
 * @return true
 */
bool mp_th_advertise(void) {

	// advertise the mission_planning topic
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
