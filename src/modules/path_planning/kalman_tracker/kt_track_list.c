/*
 * kt_track_list.c
 *
 * This file implements a linked list for storing tracking objects
 *
 *  Created on: 22.05.2015
 *      Author: Jonas Wirz <wirzjo@student.ethz.ch>
 */


#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include "kt_track_list.h"
#include "kt_cog_list.h"

#include <systemlib/err.h>
#include <drivers/drv_hrt.h>

#include "../pp_config.h"
//#include "../pp_communication_buffer.h"



/***********************************************************************************/
/*****  V A R I A B L E S  *********************************************************/
/***********************************************************************************/

//#define NULL 0

//State of the Linked list
static struct {
	uint8_t size;	//Current Size of the List
	track_obj *root; //Root of the tracking object list (pointer to the root)
	track_obj *conductor; //Pointer to track_obj when traversing the list
} state = {
	.size =0,
	.root = NULL,
	.conductor = NULL
};

static struct {
	uint8_t unseen_threshold; //Number of times the object must be unseen, before it gets deleted
} config = {
	.unseen_threshold = 2
};


static float Q[4];
static float sigma = 2.0f;	//variance for Kalman R





/***********************************************************************************/
/*****  F U N C T I O N   P R O T O T Y P E S **************************************/
/***********************************************************************************/

/* @brief Flush the list */
bool tl_flush(void);

/* @brief Delete a tracked object from the list */
bool tl_delete_obj(track_obj *ptr);

/* @brief Update the State by using Kalman-Update */
bool tl_kalman_update(track_obj *ptr, float x_meas, float y_meas);




/***********************************************************************************/
/*****  P U B L I C    F U N C T I O N S  ******************************************/
/***********************************************************************************/

/*
 * Init the Kalman Tracker
 *
 * @param wx_port_pointer: Pointer to the COM-port handler
 */
bool tl_init(void) {

	//The list is empty => size = 0
	state.size = 0;
	state.root = NULL;
	state.conductor = NULL;

	//Everything is OK and we can return true
	return true;
}


/*
 * Add a tracking object to the List
 *
 * @param x_cog, y_cog: Measured Position of the potential obstacle
 */
bool tl_add(float x_cog, float y_cog) {

	//We create the Object
	track_obj temp;

	//P is equal to the identity
	temp.P[0] = 1; //P11
	temp.P[1] = 0; //state.conductor->P
	temp.P[2] = 0; //P21
	temp.P[3] = 1; //P22

	//xhat reflects the first measurement
	temp.xhat[0] = x_cog; //x
	temp.xhat[1] = 1;     //vx
	temp.xhat[2] = y_cog; //y
	temp.xhat[3] = 1;     //vy

	temp.seen = 1;	//The potential obstacle has been seen once
	temp.unseen = 0; //The potential obstacle has never been unseen

	temp.type = 0; 	//We assume that the potential obstacle is static => type = 0

	temp.timestamp = hrt_absolute_time();

	//Set the next object in the list
	temp.next = NULL;



	//Append the Object to the list
	if(state.root == NULL) {
		//The list is empty => we must set the root

		state.root = &temp;

		state.size = 1;

	} else {
		//The list contains at least one object => we append the object at the end

		//Set the conductor to the root-element
		state.conductor = state.root;

		//Go to the end of the list
		while(state.conductor->next != NULL) {
			state.conductor = state.conductor->next;
		}

		state.conductor->next = &temp;

		state.size += 1; //An object was added => increase the size of the list
	}

	return true;
}




/**
 * Update the list by making use of the Kalman predict step
 *
 * @param
 */
bool tl_kalman_predict(void) {

	//Iterate over the whole list
	state.conductor = state.root;

	while(state.conductor->next != NULL) {

		//Time since the object was last tracked [s]
		uint64_t dt = (hrt_absolute_time() - state.conductor->timestamp)/1e6;


		//Calculate the Q-Matrix for Kalman
		Q[0] = dt*dt*dt/2;
		Q[1] = dt*dt/2;
		Q[2] = dt*dt/2;
		Q[3] = dt;


		//Update the estimate of the current object  (x_hat(t+1) = F*x_hat(t))
		state.conductor->xhat[0] = state.conductor->xhat[0]+dt*state.conductor->xhat[1];
		//state.conductor->xhat[1] = state.conductor->xhat[1];
		state.conductor->xhat[2] = state.conductor->xhat[2]+dt*state.conductor->xhat[3];
		//state.conductor->xhat[3] = state.conductor->xhat[3];

		//Update the estimated measurement of the object
		//z_hat[0] = state.conductor->xhat[0];	//NOTE: We do not need to store this!
		//z_hat[1] = state.conductor->xhat[2];

		//Update the P-Matrix

		//TODO: Check this P41 is missing!!!
		state.conductor->P[0] = state.conductor->P[0] + state.conductor->P[4]*dt + dt*(state.conductor->P[1] + state.conductor->P[5]*dt) + Q[0]; //state.conductor->P11
		state.conductor->P[1] = state.conductor->P[1] + state.conductor->P[5]*dt + Q[1]; //state.conductor->P12
		state.conductor->P[2] = state.conductor->P[2] + state.conductor->P[6]*dt + dt*(state.conductor->P[3] + state.conductor->P[4]*dt); //state.conductor->P13
		state.conductor->P[3] = state.conductor->P[3] + state.conductor->P[4]*dt; //state.conductor->P14
		state.conductor->P[4] = state.conductor->P[4] + state.conductor->P[5]*dt + Q[2]; //state.conductor->P24
		state.conductor->P[5] = state.conductor->P[5] + Q[3]; //state.conductor->P22
		state.conductor->P[6] = state.conductor->P[6] + state.conductor->P[7]*dt; //state.conductor->P23
		//state.conductor->P[7] = state.conductor->P[7]; //state.conductor->P24
		state.conductor->P[8] = state.conductor->P[8] + state.conductor->P[8]*dt + dt*(state.conductor->P[9] + state.conductor->P[13]*dt); //state.conductor->P31
		state.conductor->P[9] = state.conductor->P[9] + state.conductor->P[13]*dt; //state.conductor->P32
		state.conductor->P[10] = state.conductor->P[10] + state.conductor->P[14]*dt + dt*(state.conductor->P[11] + state.conductor->P[15]*dt) + Q[0]; //state.conductor->P33
		state.conductor->P[11] = state.conductor->P[11] + state.conductor->P[15]*dt + Q[1]; //state.conductor->P34
		state.conductor->P[12] = state.conductor->P[12] + state.conductor->P[13]*dt; //state.conductor->P41
		//state.conductor->P[13] = state.conductor->P[13]; //state.conductor->P42
		state.conductor->P[14] = state.conductor->P[14] + state.conductor->P[15]*dt + Q[2]; //state.conductor->P43
		state.conductor->P[15] = state.conductor->P[15] + Q[3]; //state.conductor->P44


		//Update the S-Matrix
		state.conductor->S[0] = state.conductor->P[0] + sigma;
		state.conductor->S[1] = state.conductor->P[2];
		state.conductor->S[2] = state.conductor->P[8];
		state.conductor->S[3] = state.conductor->P[10] + sigma;


		//Set the Conductor to the next element in the list
		state.conductor = state.conductor->next;
	}

	return true;
}


/**
 * Try to find matchings between the predicted object positions and the newly identified COGs
 * This is done using a NNSF (nearest neighbour standard filter)
 *
 */
bool tl_nnsf(void) {

	//Iterate over the whole list of tracking objects
	state.conductor = state.root;

	while(state.conductor->next != NULL) {

		float x_meas = 0;
		float y_meas = 0;

		//Find the best matching COG
		bool result = cl_find_nn(state.conductor->xhat[0],state.conductor->xhat[2], &x_meas, &y_meas);

		if(result == true){
			//A COG that fits the estimate was found => we have a new measurement and can do the Kalman Update-State

			tl_kalman_update(state.conductor, x_meas, y_meas);

		} else {
			//No COG matched the estimate => the object is possibly hidden by another object

			//Increase the unseen-flag
			state.conductor->unseen += 1;

			if(state.conductor->unseen > config.unseen_threshold) {
				//The object was unseen several times => we expect it to be not present => delete it

				tl_delete_obj(state.conductor);
			}
		}


		//Set the next conductor
		state.conductor = state.conductor->next;
	}

	return true;
}



/**
 * Kalman Update Step
 *
 * @param ptr: Pointer to the Object that needs to be updated
 * @param x_meas/y_meas: Measured x and y-Poistion of the object
 */
bool tl_kalman_update(track_obj *ptr, float x_meas, float y_meas) {

	//x (the new measured state-difference)
    float v0, v1;
    v0 = x_meas - ptr->xhat[0];
    v1 = y_meas - ptr->xhat[3];


    //Calcualte the W-Matrix
    float P11 = ptr->P[0];
    float P12 = ptr->P[1];
    float P13 = ptr->P[2];
    float P14 = ptr->P[3];
    float P21 = ptr->P[4];
    float P22 = ptr->P[5];
    float P23 = ptr->P[6];
    float P24 = ptr->P[7];
    float P31 = ptr->P[8];
    float P32 = ptr->P[9];
    float P33 = ptr->P[10];
    float P34 = ptr->P[11];
    float P41 = ptr->P[12];
    float P42 = ptr->P[13];
    float P43 = ptr->P[14];
    float P44 = ptr->P[15];


    float W0 = (P11*(P33 + sigma))/(P11*sigma + P33*sigma + sigma*sigma + P11*P33 - P13*P31) - (P13*P31)/(P11*sigma + P33*sigma + sigma*sigma + P11*P33 - P13*P31);
	float W1 = (P13*(P11 + sigma))/(P11*sigma + P33*sigma + sigma*sigma + P11*P33 - P13*P31) - (P11*P13)/(P11*sigma + P33*sigma + sigma*sigma + P11*P33 - P13*P31);
	float W2 = (P21*(P33 + sigma))/(P11*sigma + P33*sigma + sigma*sigma + P11*P33 - P13*P31) - (P23*P31)/(P11*sigma + P33*sigma + sigma*sigma + P11*P33 - P13*P31);
	float W3 = (P23*(P11 + sigma))/(P11*sigma + P33*sigma + sigma*sigma + P11*P33 - P13*P31) - (P13*P21)/(P11*sigma + P33*sigma + sigma*sigma + P11*P33 - P13*P31);
	float W4 = (P31*(P33 + sigma))/(P11*sigma + P33*sigma + sigma*sigma + P11*P33 - P13*P31) - (P31*P33)/(P11*sigma + P33*sigma + sigma*sigma + P11*P33 - P13*P31);
	float W5 = (P33*(P11 + sigma))/(P11*sigma + P33*sigma + sigma*sigma + P11*P33 - P13*P31) - (P13*P31)/(P11*sigma + P33*sigma + sigma*sigma + P11*P33 - P13*P31);
	float W6 = (P41*(P33 + sigma))/(P11*sigma + P33*sigma + sigma*sigma + P11*P33 - P13*P31) - (P31*P43)/(P11*sigma + P33*sigma + sigma*sigma + P11*P33 - P13*P31);
	float W7 = (P43*(P11 + sigma))/(P11*sigma + P33*sigma + sigma*sigma + P11*P33 - P13*P31) - (P13*P41)/(P11*sigma + P33*sigma + sigma*sigma + P11*P33 - P13*P31);


    //Update the state xhat
    ptr->xhat[0] = ptr->xhat[0] + W0*v0 + W1*v1;
    ptr->xhat[1] = ptr->xhat[1] + W2*v0 + W3*v1;
    ptr->xhat[2] = ptr->xhat[2] + W4*v0 + W5*v1;
    ptr->xhat[3] = ptr->xhat[3] + W6*v0 + W7*v1;


    //Update P
    float S0 = ptr->S[0];
    float S1 = ptr->S[1];
    float S2 = ptr->S[2];
    float S3 = ptr->S[3];

    ptr->P[0] = P11 -  W0*(S0*W0 + S2*W1) -  W1*(S1*W0 + S3*W1);
    ptr->P[1] = P12 -  W2*(S0*W0 + S2*W1) -  W3*(S1*W0 + S3*W1);
    ptr->P[2] = P13 -  W4*(S0*W0 + S2*W1) -  W5*(S1*W0 + S3*W1);
    ptr->P[3] = P14 -  W6*(S0*W0 + S2*W1) -  W7*(S1*W0 + S3*W1);
    ptr->P[4] = P21 -  W0*(S0*W2 + S2*W3) -  W1*(S1*W2 + S3*W3);
    ptr->P[5] = P22 -  W2*(S0*W2 + S2*W3) -  W3*(S1*W2 + S3*W3);
    ptr->P[6] = P23 -  W4*(S0*W2 + S2*W3) -  W5*(S1*W2 + S3*W3);
    ptr->P[7] = P24 -  W6*(S0*W2 + S2*W3) -  W7*(S1*W2 + S3*W3);
    ptr->P[8] = P31 -  W0*(S0*W4 + S2*W5) -  W1*(S1*W4 + S3*W5);
    ptr->P[9] = P32 -  W2*(S0*W4 + S2*W5) -  W3*(S1*W4 + S3*W5);
    ptr->P[10] = P33 -  W4*(S0*W4 + S2*W5) -  W5*(S1*W4 + S3*W5);
    ptr->P[11] = P34 -  W6*(S0*W4 + S2*W5) -  W7*(S1*W4 + S3*W5);
    ptr->P[12] = P41 -  W0*(S0*W6 + S2*W7) -  W1*(S1*W6 + S3*W7);
    ptr->P[13] = P42 -  W2*(S0*W6 + S2*W7) -  W3*(S1*W6 + S3*W7);
    ptr->P[14] = P43 -  W4*(S0*W6 + S2*W7) -  W5*(S1*W6 + S3*W7);
    ptr->P[15] = P44 -  W6*(S0*W6 + S2*W7) -  W7*(S1*W6 + S3*W7);

	return true;
}


/**
 * Add untracked, new objects to the list of tracked objects
 */
bool tl_add_untracked(void) {
	return cl_add_untracked();
}








/***********************************************************************************/
/*****  P R I V A T E    F U N C T I O N S  ****************************************/
/***********************************************************************************/

/**
 * Flush the list
 * Delete the list from memory.
 *
 */
bool tl_flush(void) {

 	return true;
}


/**
 * Delete a COG-Object from the List
 *
 * @param ptr: Pointer to the cog_obj that should be deleted
 */
bool tl_delete_obj(track_obj *ptr) {

	track_obj *next_ptr;

	//Start at the root and then search for the object before the one we want to delete
	next_ptr = state.root;

	while(next_ptr != ptr && next_ptr != NULL) {
		//TODO: check, if this is corret and the next_ptr points after the while to the object before the one we wnat to delet
		next_ptr = next_ptr->next;
	}

	//We reached the object that points to the one we want to delete
	next_ptr->next = ptr->next; //Set the pointer of the object before the one we want to delete to the object after the one we want to delete

	//Delete the Object
	free(ptr);

	return true;
}





