/*
 * topics_handler.h
 *
 *  Created on: 04.03.2015
 *      Author: Jonas
 */

#ifndef TOPICS_HANDLER_H_
#define TOPICS_HANDLER_H_

#include <uORB/uORB.h>
#include <uORB/topics/path_planning.h>

#include <stdbool.h>
#include <stdio.h>
#include <systemlib/err.h>



//Struct of all topic-Subscriptions
struct subscribtion_fd_s{
    int path_planning;		//Contains the current heading of the boat
};


//Structs of interested topics
struct structs_topics_s{
	struct path_planning_s path_planning;
};



/** @brief Subscribe to interested Topics*/
bool th_subscribe(struct subscribtion_fd_s *subs_p, struct structs_topics_s *strs_p);

/** @brief Advertise Topics */
bool th_advertise(void);


#endif /* TOPICS_HANDLER_H_ */
