/*
 * topics_handler.h
 *
 *  Created on: 04.03.2015
 *      Author: Jonas
 */

#ifndef TOPICS_HANDLER_H_
#define TOPICS_HANDLER_H_

#include <uORB/uORB.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/wind_sailing.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/boat_guidance_debug.h>
#include <uORB/topics/path_planning.h>

#include <stdbool.h>
#include <stdio.h>
#include <systemlib/err.h>



//Struct of all topic-Subscriptions
struct subscribtion_fd_s{
    int vehicle_global_position;			//Contains the filtered GPS-Data
    int wind_sailing;						//Estimate of True Windspeed and Direction
    int parameter_update;					//Update of Parameters from QGroundControl
    int boat_guidance_debug;                //values from guidance_module from autonomous_sailing app
};


//Structs of interested topics
struct structs_topics_s{
	struct vehicle_global_position_s vehicle_global_position;
    struct wind_sailing_s wind_sailing;
	struct parameter_update_s parameter_update;
    struct boat_guidance_debug_s boat_guidance_debug;
    //published topics
    struct path_planning_s path_planning;
};



/* @brief Subscribe to interested Topics*/
bool th_subscribe(struct subscribtion_fd_s *subs_p, struct structs_topics_s *strs_p);


/* @brief Advertise Topics */
bool th_advertise(struct structs_topics_s *strs_p);


/* @brief Publish new data*/
bool th_publish(const struct structs_topics_s *strs_p);


#endif /* TOPICS_HANDLER_H_ */
