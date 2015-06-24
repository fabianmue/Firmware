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
#include "kt_topic_handler.h"

//#include <systemlib/err.h>
//#include <drivers/drv_hrt.h>

#include "../pp_config.h"
//#include "../pp_communication_buffer.h"



/***********************************************************************************/
/*****  V A R I A B L E S  *********************************************************/
/***********************************************************************************/

#define MAXSIZE 8 //maximum number of tracks allowed in the memory

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

	/*if(state.size > MAXSIZE) {
		//Make sure, we produce no memory-overflow

		return false;
	}*/


	//We create the Object
	cog_obj *temp = NULL;
	temp = malloc(sizeof(struct cog_obj));

	if(temp == NULL) {
		smq_send_log_info("Out of Memory COG");

		#if LDEBUG_KALMANTRACKER_CMS == 1
		printf("Could not allocate memory for COG!\n");
		#endif
		return false;
	}


	//Store the data
	temp->x_cog = x_cog;
	temp->y_cog = y_cog;

	temp->dist = sqrtf(x_cog*x_cog + y_cog*y_cog);


	//Insert sorted (Insert the new element such that the list remains sorted with respect to the distance
	cog_obj *current;
	//Special case for the head end
	if (state.root == NULL || (state.root->dist >= temp->dist)){
		temp->next = state.root;
	    state.root = temp;
	} else {
	    //Locate the node before the point of insertion
	    current = state.root;
	    while (current->next!=NULL && current->next->dist < temp->dist) {
	    	current = current->next;
	    }
	    temp->next = current->next;
	    current->next = temp;
	}

	state.size++;

	//When the maximum size of the List is exceeded, we delete the element with the biggest distance
	if(state.size > MAXSIZE) {
		//We exceed the maximum size of the linked list => we delete the last element

		//printf("Exceed size\n");

		state.conductor = state.root;
		cog_obj *previous = state.conductor;
		while(state.conductor->next != NULL) {
			previous = state.conductor;
			state.conductor = state.conductor->next;
		}

		//We are at the end of the list
		//printf("End of list: %f\n",previous->x_cog);
		previous->next = NULL;
		free(state.conductor);

		state.size--;

	}



	/*//Set the next object in the list
	temp->next = state.root;

	state.root = temp;

	state.size++;*/


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

		//printf("COG in find_nn: %f/%f\n ",(double)(state.conductor->x_cog),(double)(state.conductor->y_cog));

		//Calculate the distance
		float dx = x_pos-state.conductor->x_cog;
		float dy = y_pos-state.conductor->y_cog;
		float dist = sqrtf(dx*dx + dy*dy);

		//printf("Distance: %f5.2 (of %f5.2)\n",(double)dist,(double)config.nnsf_threshold);

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
		cl_delete_obj(nearest_cog); //TODO uncomment this

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

	//Count the number of newly added objects
	uint16_t newadded;

	//Set the conductor to the root
	state.conductor = state.root;

	while(state.conductor != NULL) {

		//Add the untracked element to the list of tracked objects
		tl_add(state.conductor->x_cog, state.conductor->y_cog);

		//Delete the object that was just added for tracking
		cl_delete_obj(state.conductor);

		//Let the conductor point to the next object
		state.conductor = state.conductor->next;

		//Count the number of newly added objects
		newadded++;
	}

	//Publish the number of newly added objects for QGround Control
	th_set_nrofnewtracks(newadded);

	return true;

}


/**
 * Set configuration parameters by QGround Control
 *
 * @param nnsf_threshold: Threshold for the nnsf (nearest neighbour standard filter) [cm]
 */
bool cl_set_configuration(float nnsf_threshold) {

	config.nnsf_threshold = nnsf_threshold;

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

		cog_obj *temp = ptr;

		state.root = ptr->next;
		//ptr = state.root;

		state.size--;

		free(temp);
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

	//ptr->next = previous->next;

	return true;
}



void cl_print_list(void) {

	state.conductor = state.root;

	printf("COG-List (%d)",state.size);

	while(state.conductor != NULL) {
		//Iterate through the list

		printf("%f/%f, ",(double)(state.conductor->x_cog),(double)(state.conductor->y_cog));

		state.conductor = state.conductor->next;
	}


	printf("\n");

}





