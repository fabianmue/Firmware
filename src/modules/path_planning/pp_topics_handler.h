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
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/boat_guidance_debug.h>
#include <uORB/topics/path_planning.h>
#include <uORB/topics/boat_qgc_param.h>
#include <uORB/topics/rc_channels.h>
#include <uORB/topics/boat_local_position.h>
#include <uORB/topics/boat_pp_debug.h>
#include <uORB/topics/vehicle_attitude.h>

#include <stdbool.h>
#include <stdio.h>
#include <systemlib/err.h>



//Struct of all topic-Subscriptions
struct subscribtion_fd_s{
    int vehicle_global_position;			//Contains the filtered GPS-Data
    int parameter_update;					//Update of Parameters from QGroundControl
    int boat_guidance_debug;                //Values from guidance_module from autonomous_sailing app
    int rc_channels;                        //Remote Control commands
    int vehicle_attitude;					//Raw Yaw-Angle
};


//Structs of interested topics
struct structs_topics_s{
	struct vehicle_global_position_s vehicle_global_position;
	struct parameter_update_s parameter_update;
    struct rc_channels_s rc_channels;
    struct vehicle_attitude_s vehicle_attitude;
};



/** @brief Subscribe to interested Topics*/
bool th_subscribe(struct subscribtion_fd_s *subs_p, struct structs_topics_s *strs_p);

/** @brief Advertise Topics */
bool th_advertise(void);

/** @brief publish path_planning topic*/
int th_publish_path_planning(const struct path_planning_s *path_planning_p);

/** @brief publish boat_pp_debug1 topic */
int th_publish_boat_pp_debug1(const struct boat_pp_debug1_s *boat_pp_debug1);

/** @brief publish qgc2 topic */
int th_publish_qgc2(const struct boat_qgc_param2_s *boat_qgc_param2_p);

/** @brief publish boat_local_position topic*/
int th_publish_boat_local_position(const struct boat_local_position_s *boat_local_position_p);

#endif /* TOPICS_HANDLER_H_ */
