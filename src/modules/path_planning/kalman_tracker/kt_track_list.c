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
	float sigma; 			  //Variance for the Kalman Matrix R
} config = {
	.unseen_threshold = 2,
	.sigma = 2.0f
};






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

	#if LDEBUG_KALMANTRACKER_CMS == 1
	//printf("Added tracking object! (%f/%f)\n",(double)x_cog,(double)y_cog);
	#endif

	//We create the Object
	track_obj *temp;
	temp = malloc(sizeof(track_obj));

	//P is equal to the identity
	temp->P[0] = 1; //P11
	temp->P[1] = 0; //P12
	temp->P[2] = 0; //P13
	temp->P[3] = 0; //P14
	temp->P[4] = 0; //P21
	temp->P[5] = 1; //P22
	temp->P[6] = 0; //P23
	temp->P[7] = 0; //P24
	temp->P[8] = 0; //P31
	temp->P[9] = 0; //P32
	temp->P[10] = 1; //P33
	temp->P[11] = 0; //P34
	temp->P[12] = 0; //P41
	temp->P[13] = 0; //P42
	temp->P[14] = 0; //P43
	temp->P[15] = 1; //P44


	//xhat reflects the first measurement
	temp->xhat[0] = x_cog; //x
	temp->xhat[1] = 1;     //vx
	temp->xhat[2] = y_cog; //y
	temp->xhat[3] = 1;     //vy

	temp->seen = 1;	//The potential obstacle has been seen once
	temp->unseen = 0; //The potential obstacle has never been unseen

	temp->type = 0; 	//We assume that the potential obstacle is static => type = 0

	temp->timestamp = hrt_absolute_time();

	//Set the next object in the list
	temp->next = NULL;



	//Set the next object in the list
	temp->next = state.root;

	state.root = temp;

	state.size++;

	return true;
}




/**
 * Update the list by making use of the Kalman predict step
 *
 */
bool tl_kalman_predict(void) {

	//Iterate over the whole list
	state.conductor = state.root;

	while(state.conductor != NULL) {

		//Time since the object was last tracked [s]
		uint64_t dt = (hrt_absolute_time() - state.conductor->timestamp)/1e6;


		//Calculate the Q-Matrix for Kalman
		float Q11 = dt*dt*dt/2;
		float Q12 = dt*dt/2;
		float Q21 = dt*dt/2;
		float Q22 = dt;


		//Update the estimate of the current object  (x_hat(t+1) = F*x_hat(t))
		state.conductor->xhat[0] = state.conductor->xhat[0]+dt*state.conductor->xhat[1];
		//state.conductor->xhat[1] = state.conductor->xhat[1];
		state.conductor->xhat[2] = state.conductor->xhat[2]+dt*state.conductor->xhat[3];
		//state.conductor->xhat[3] = state.conductor->xhat[3];

		//Update the estimated measurement of the object
		//z_hat[0] = state.conductor->xhat[0];	//NOTE: We do not need to store this!
		//z_hat[1] = state.conductor->xhat[2];

		//Update the P-Matrix
		float P11 = state.conductor->P[0];
		float P12 = state.conductor->P[1];
		float P13 = state.conductor->P[2];
		float P14 = state.conductor->P[3];
		float P21 = state.conductor->P[4];
		float P22 = state.conductor->P[5];
		float P23 = state.conductor->P[6];
		float P24 = state.conductor->P[7];
		float P31 = state.conductor->P[8];
		float P32 = state.conductor->P[9];
		float P33 = state.conductor->P[10];
		float P34 = state.conductor->P[11];
		float P41 = state.conductor->P[12];
		float P42 = state.conductor->P[13];
		float P43 = state.conductor->P[14];
		float P44 = state.conductor->P[15];

		state.conductor->P[0] = P11 + Q11 + P21*dt + dt*(P12 + P22*dt);
		state.conductor->P[1] = P12 + Q12 + P22*dt;
		state.conductor->P[2] = P13 + P23*dt + dt*(P14 + P24*dt);
		state.conductor->P[3] = P14 + P24*dt;
		state.conductor->P[4] = P21 + Q21 + P22*dt;
		state.conductor->P[5] = P22 + Q22;
		state.conductor->P[6] = P23 + P24*dt;
		state.conductor->P[7] = P24;
		state.conductor->P[8] = P31 + P41*dt + dt*(P32 + P42*dt);
		state.conductor->P[9] = P32 + P42*dt;
		state.conductor->P[10] = P33 + Q11 + P43*dt + dt*(P34 + P44*dt);
		state.conductor->P[11] = P34 + Q12 + P44*dt;
		state.conductor->P[12] = P41 + P42*dt;
		state.conductor->P[13] = P42;
		state.conductor->P[14] = P43 + Q21 + P44*dt;
		state.conductor->P[15] = P44 + Q22;


		//Update the S-Matrix
		state.conductor->S[0] = state.conductor->P[0] + config.sigma;
		state.conductor->S[1] = state.conductor->P[2];
		state.conductor->S[2] = state.conductor->P[8];
		state.conductor->S[3] = state.conductor->P[10] + config.sigma;


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

	while(state.conductor != NULL) {

		//printf("Track Obj in List: %f/%f\n",(double)(state.conductor->xhat[0]),(double)(state.conductor->xhat[2]));

		float x_meas = 0;
		float y_meas = 0;

		//Find the best matching COG
		bool result = cl_find_nn(state.conductor->xhat[0],state.conductor->xhat[2], &x_meas, &y_meas);

		//printf("Predicted COG: %f/%f\n",(double)(state.conductor->xhat[0]),(double)(state.conductor->xhat[2]));

		track_obj *nextptr = NULL;

		if(result == true){
			//A COG that fits the estimate was found => we have a new measurement and can do the Kalman Update-State

			#if LDEBUG_KALMANTRACKER_CMS == 1
			//printf("Found nearest COG!\n");
			#endif

			tl_kalman_update(state.conductor, x_meas, y_meas);

			//printf("Predicted COG: %f/%f\n",(double)(state.conductor->xhat[0]),(double)(state.conductor->xhat[2]));

			nextptr = state.conductor->next;

		} else {
			//No COG matched the estimate => the object is possibly hidden by another object

			//Increase the unseen-flag
			state.conductor->unseen += 1;

			//printf("Did not find nearest COG: %d\n",state.conductor->unseen);

			if(state.conductor->unseen > config.unseen_threshold) {
				//The object was unseen several times => we expect it to be not present => delete it

				#if LDEBUG_KALMANTRACKER_CMS == 1
				//printf("Deleted Object!");
				#endif

				//nextptr = state.conductor->next;
				tl_delete_obj(state.conductor);
				nextptr = state.conductor->next;
			}
		}


		//Set the next conductor
		state.conductor = nextptr;
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
    v1 = y_meas - ptr->xhat[2];


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


    float W0 = (P11*(P33 + config.sigma))/(P11*config.sigma + P33*config.sigma + config.sigma*config.sigma + P11*P33 - P13*P31) - (P13*P31)/(P11*config.sigma + P33*config.sigma + config.sigma*config.sigma + P11*P33 - P13*P31);
	float W1 = (P13*(P11 + config.sigma))/(P11*config.sigma + P33*config.sigma + config.sigma*config.sigma + P11*P33 - P13*P31) - (P11*P13)/(P11*config.sigma + P33*config.sigma + config.sigma*config.sigma + P11*P33 - P13*P31);
	float W2 = (P21*(P33 + config.sigma))/(P11*config.sigma + P33*config.sigma + config.sigma*config.sigma + P11*P33 - P13*P31) - (P23*P31)/(P11*config.sigma + P33*config.sigma + config.sigma*config.sigma + P11*P33 - P13*P31);
	float W3 = (P23*(P11 + config.sigma))/(P11*config.sigma + P33*config.sigma + config.sigma*config.sigma + P11*P33 - P13*P31) - (P13*P21)/(P11*config.sigma + P33*config.sigma + config.sigma*config.sigma + P11*P33 - P13*P31);
	float W4 = (P31*(P33 + config.sigma))/(P11*config.sigma + P33*config.sigma + config.sigma*config.sigma + P11*P33 - P13*P31) - (P31*P33)/(P11*config.sigma + P33*config.sigma + config.sigma*config.sigma + P11*P33 - P13*P31);
	float W5 = (P33*(P11 + config.sigma))/(P11*config.sigma + P33*config.sigma + config.sigma*config.sigma + P11*P33 - P13*P31) - (P13*P31)/(P11*config.sigma + P33*config.sigma + config.sigma*config.sigma + P11*P33 - P13*P31);
	float W6 = (P41*(P33 + config.sigma))/(P11*config.sigma + P33*config.sigma + config.sigma*config.sigma + P11*P33 - P13*P31) - (P31*P43)/(P11*config.sigma + P33*config.sigma + config.sigma*config.sigma + P11*P33 - P13*P31);
	float W7 = (P43*(P11 + config.sigma))/(P11*config.sigma + P33*config.sigma + config.sigma*config.sigma + P11*P33 - P13*P31) - (P13*P41)/(P11*config.sigma + P33*config.sigma + config.sigma*config.sigma + P11*P33 - P13*P31);


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


    //Set the timestamp
    ptr->timestamp = hrt_absolute_time();

	return true;
}


/**
 * Add untracked, new objects to the list of tracked objects
 */
bool tl_add_untracked(void) {
	return cl_add_untracked();
}


/**
 * Get the number of currently tracked objects <=> number of elements in the linked list
 *
 */
uint16_t tl_get_size(void) {
	return state.size;
}


/**
 * Get the obstacles currently tracked as an array of NED-points
 * Note: This function was debugged and should be OK!
 *
 * @param *array: pointer to an array of NED-Points
 * @param curpos: Current Position of the SENSOR in global NED-Frame
 *
 * @return Size of the Array <=> number of tracked objects
 */
uint16_t tl_get_obstacles(NEDpoint *array, NEDpoint curpos) {

	//Set the conductor as the root => start at the head of the list
	state.conductor = state.root;

	uint16_t index = 0; //Index in the Array

	while(state.conductor != NULL) {
		//Iterate over the whole List

		//Store the Obstacle Position estimates in the Matrix
		array[index].northx = curpos.northx + (state.conductor->xhat[0])/100.0f;
		array[index].easty = curpos.easty + (state.conductor->xhat[2])/100.0f;

		index++;
		state.conductor = state.conductor->next;
	}

	return tl_get_size();
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

	//Set the conductor as the root
	state.conductor = state.root;

	if(state.conductor == NULL) {
		//List is empty => nothing to flush
		return true;
	}

	while(state.conductor != NULL) {

		tl_delete_obj(state.conductor);

		state.conductor = state.conductor->next;

	}

	//The list is now flushed => we have to reinit it
	tl_init();

 	return true;
}


/**
 * Delete a Track-Object from the List
 *
 * @param ptr: Pointer to the track_obj that should be deleted
 */
bool tl_delete_obj(track_obj *ptr) {

	if(ptr == state.root) {
		//We want to delete the root

		printf("Track Obj: Delete Root\n");

		track_obj *temp = ptr;

		state.root = ptr->next;
		//ptr = state.root;

		state.size--;

		free(temp);
		return true;
	}

	track_obj *conductor;
	track_obj *previous;	//Object before the one we want to delete

	//Start at the root and then search for the object before the one we want to delete
	conductor = state.root;

	int counter;
	counter = 0;
	while(conductor != ptr){// && conductor != NULL) {
		counter ++;
		previous = conductor;
		conductor = conductor->next;
	}
	//The conductor points now to the element we want to delete

	track_obj *temp = conductor;
	previous->next = conductor->next;
	free(temp);
	state.size--;

	//ptr->next = conductor->next;

	return true;
}


void tl_print_list(void) {

	state.conductor = state.root;

	printf("Track Obj-List (%d)",tl_get_size());

	while(state.conductor != NULL) {
		//Iterate through the list

		printf("%f/%f, ",(double)(state.conductor->xhat[0]),(double)(state.conductor->xhat[2]));

		state.conductor = state.conductor->next;
	}

	printf("\n");

}





