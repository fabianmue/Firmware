/*
 * kt_cog_list.h
 *
 *  Created on: 25.05.2015
 *      Author: Jonas
 */

#ifndef KT_COG_LIST_H_
#define KT_COG_LIST_H_

#include <stdbool.h>

typedef struct cog_obj{

	float x_cog;	//x-Position of the Segment in Sensor-Frame (NED)
	float y_cog;    //y-Position of the Segment in Sensor-Frame (NED)

	float dist; 	//Distance from the Sensor

	struct cog_obj *next;
} cog_obj;


/* @brief Init a new track-list */
bool cl_init(void);


/* @brief Add a new track-element */
bool cl_add(float x_cog, float y_cog);


/* @brief Add currently untracked COG's to the list of tracking-elements */
bool cl_add_untracked(void);


/* @brief Find the best fitting COG to a given estimate */
bool cl_find_nn(float x_pos, float y_pos, float *x_meas, float *y_meas);


/* @brief Flush the list */
bool cl_flush(void);


/* @brief Delete an object from the list */
bool cl_delete_obj(cog_obj *ptr);


/* @brief Set configuration parameters by QGround Control */
bool cl_set_configuration(float nnsf_threshold);


void cl_print_list(void);

//cog_obj *get_obj(float key);


#endif /* KT_COG_LIST_H_ */
