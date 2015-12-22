
/*
 * mp_topics_handler.h
 *
 *  Created on: 04.12.2015
 *      Author: Fabian
 */

#ifndef MP_TOPICS_HANDLER_H_
#define MP_TOPICS_HANDLER_H_

#include <uORB/uORB.h>
#include <uORB/topics/parameter_update.h> 	// timestamp
#include <uORB/topics/mission_planning.h> 	// current target (lat-lon and NED)

#include <stdbool.h>
#include <stdio.h>
#include <systemlib/err.h>

/***********************************************************************************/
/*****  V A R I A B L E S  *********************************************************/
/***********************************************************************************/

// struct of all topic-subscriptions
struct mp_subscribtion_fd_s {
    int parameter_update;					// update of Parameters from QGroundControl
};

// structs of interested topics
struct mp_structs_topics_s{
	struct parameter_update_s parameter_update;
};

/***********************************************************************************/
/*****  F U N C T I O N   D E C L A R A T I O N S  *********************************/
/***********************************************************************************/

/* @brief subscribe to interested topics */
bool mp_th_subscribe(struct mp_subscribtion_fd_s *subs_p, struct mp_structs_topics_s *strs_p);

/* @brief advertise topics */
bool mp_th_advertise(void);

/* @brief publish mission_planning topic */
int mp_th_publish(const struct mission_planning_s *mission_planning_p);

#endif /* MP_TOPICS_HANDLER_H_ */
