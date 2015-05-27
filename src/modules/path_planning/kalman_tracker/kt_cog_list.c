/*
 * kt_cog_list.c
 *
 * This file implements a linked list for storing the newly detected cog's
 *
 *  Created on: 22.05.2015
 *      Author: Jonas Wirz <wirzjo@student.ethz.ch>
 */


#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "kt_cog_list.h"
#include "kt_track_list.h"

//#include <systemlib/err.h>
//#include <drivers/drv_hrt.h>

#include "../pp_config.h"
//#include "../pp_communication_buffer.h"



/***********************************************************************************/
/*****  V A R I A B L E S  *********************************************************/
/***********************************************************************************/

//#define NULL 0

//State of the Linked list
static struct {
	uint8_t size;	//Current Size of the List
	cog_obj *root; //Root of the tracking object list (pointer to the root)
	cog_obj *conductor; //Pointer to track_obj when traversing the list
} state = {
	.size =0,
	.root = NULL,
	.conductor = NULL
};

static struct {
	float nnsf_threshold; //Threshold distance for NNSF [cm]
} config = {
	.nnsf_threshold = 100
};





/***********************************************************************************/
/*****  F U N C T I O N   P R O T O T Y P E S **************************************/
/***********************************************************************************/





/***********************************************************************************/
/*****  P U B L I C    F U N C T I O N S  ******************************************/
/***********************************************************************************/

/*
 * Init the Kalman Tracker
 *
 * @param wx_port_pointer: Pointer to the COM-port handler
 */
bool cl_init(void) {

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
bool cl_add(float x_cog, float y_cog) {

	//We create the Object
	cog_obj *temp = NULL;
	temp = malloc(sizeof(struct cog_obj));

	if(temp == NULL) {
		printf("Could not allocate memory!");
		return false;
	}


	//Store the data
	temp->x_cog = x_cog;
	temp->y_cog = y_cog;

	//Set the next object in the list
	temp->next = state.root;

	state.root = temp;

	state.size++;


	return true;
}



/**
 * Find the nearest neighbour to a given position
 *
 * @param x_pos/y_pos: Position of the estimate
 * @param y_meas/y_meas: Measured Position (nearest neighbour COG)
 *
 * @return true, if a nearest object closer than a threshold was found
 */
bool cl_find_nn(float x_pos, float y_pos, float *x_meas, float *y_meas) {

	//Minimum Distance between the position and the COG
	float min_dist = 1000000.0f;
	cog_obj *nearest_cog = NULL;	//Pointer to the nearest COG

	//Iterate over the whole list and try to find the nearest neighbour
	state.conductor = state.root;

	while(state.conductor != NULL) {

		//Calculate the distance
		float dx = x_pos-state.conductor->x_cog;
		float dy = y_pos-state.conductor->y_cog;
		float dist = sqrtf(dx*dx + dy*dy);

		if(dist < min_dist && dist < config.nnsf_threshold) {
			//We found a new nearest COG => store it as the nearest

			min_dist = dist;
			nearest_cog = state.conductor;
		}

		//Set the conductor to the next element
		state.conductor = state.conductor->next;
	}

	if(nearest_cog != NULL) {
		//Return the values of the nearest COG
		*x_meas = nearest_cog->x_cog;
		*y_meas = nearest_cog->y_cog;

		//Delete the COG-Value from the List
		cl_delete_obj(nearest_cog);

		return true;
	} else {
		//No nearest COG was found => return false

		return false;
	}
}


/**
 * Add the untracked cog objects as new tracks to the list
 */
bool cl_add_untracked(void) {

	//Set the conductor to the root
	state.conductor = state.root;

	while(state.conductor != NULL) {

		//Add the untracked element to the list of tracked objects
		tl_add(state.conductor->x_cog, state.conductor->y_cog);

		//Delete the object that was just added for tracking
		cl_delete_obj(state.conductor);

		//Let the conductor point to the next object
		state.conductor = state.conductor->next;
	}

	return true;

}





/***********************************************************************************/
/*****  P R I V A T E    F U N C T I O N S  ****************************************/
/***********************************************************************************/

/**
 * Flush the list
 * Delete the list from memory.
 *
 */
bool cl_flush(void) {

	//Set the conductor as the root
	state.conductor = state.root;

	if(state.conductor == NULL) {
		//List is empty => nothing to flush
		return true;
	}

	while(state.conductor != NULL) {

		//cog_obj *temp = state.root;
		//state.root = state.root->next;
		//free(temp);
		//state.size--;

		cl_delete_obj(state.conductor);

		state.conductor = state.conductor->next;

	}

	//The list is now flushed => we have to reinit it
	cl_init();

	return true;
}


/**
 * Delete a COG-Object from the List
 *
 * @param ptr: Pointer to the cog_obj that should be deleted
 */
bool cl_delete_obj(cog_obj *ptr) {

	if(ptr == state.root) {
		//We want to delete the root

		state.root = ptr->next;

		free(ptr);
		return true;
	}

	cog_obj *conductor;
	cog_obj *previous;	//Object before the one we want to delete

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

	cog_obj *temp = conductor;
	previous->next = conductor->next;
	free(temp);
	state.size--;

	return true;
}


/*cog_obj *get_obj(float key) {

	state.conductor = state.root;

	while(state.conductor != NULL) {

		if(state.conductor->x_cog == key) {
			return state.conductor;
		} else {
			state.conductor = state.conductor->next;
		}
	}

	return NULL;
}*/



bool print_list(void) {
	state.conductor = state.root;

	while(state.conductor != NULL) {

		printf("Elem: %f,\n",(double)(state.conductor->x_cog));

		state.conductor = state.conductor->next;
	}

	printf("\n");

	return true;
}





