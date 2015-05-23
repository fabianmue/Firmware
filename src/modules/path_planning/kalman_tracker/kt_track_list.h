/*
 * kt_track_list.h
 *
 *  Created on: 22.05.2015
 *      Author: Jonas Wirz <wirzjo@student.ethz.ch>
 */

#ifndef KT_TRACK_LIST_H_
#define KT_TRACK_LIST_H_

typedef struct track_obj{
	float P[16];			//state Covariance Matrix for Kalman Tracker [P11,P12,P21,P22]
	float xhat[4];		//Estimated State [x,vx,y,vy]
	float S[4];			//innovation covariance [S11,S12,S21,S22]
	uint16_t seen;		//How many times has the object been seen
	uint16_t unseen;	//How many times has the object been unseen

	uint8_t type; 		//Type of Object (0 == static, 1 == moving)

	struct track_obj *next;
} track_obj;


/* @brief Init a new track-list */
bool tl_init(void);

/* @brief Add a new track-element */
bool tl_add(float x_cog, float y_cog);

/* @brief Prediction Step of the Kalman tracker */
bool tl_kalman_predict(uint64_t dt);

/* @brief Relate the tracking objects to the measurements */
bool tl_nnsf(void);






#endif /* KT_TRACK_LIST_H_ */
