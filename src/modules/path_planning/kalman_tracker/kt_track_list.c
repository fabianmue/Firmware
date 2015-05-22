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
#include "kt_track_list.h"

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





/***********************************************************************************/
/*****  F U N C T I O N   P R O T O T Y P E S **************************************/
/***********************************************************************************/

/* @brief Flush the list */
bool tl_flush(void);




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
	temp.P[1] = 0; //P12
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





