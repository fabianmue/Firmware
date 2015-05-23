/*
 * kt_track_list.h
 *
 *  Created on: 22.05.2015
 *      Author: Jonas Wirz <wirzjo@student.ethz.ch>
 */

#ifndef KT_COG_LIST_H_
#define KT_COG_LIST_H_

typedef struct cog_obj{

	float x_cog;
	float y_cog;

	struct cog_obj *next;
} cog_obj;


/* @brief Init a new track-list */
bool cl_init(void);


/* @brief Add a new track-element */
bool cl_add(float x_cog, float y_cog);


/* @brief Find the best fitting COG to a given estimate */
bool cl_find_nn(float x_pos, float y_pos, float *x_meas, float *y_meas);


/* @brief Flush the list */
bool cl_flush(void);



#endif /* KT_COG_LIST_H_ */
