/*
 * kt_tracker.c
 *
 * This file implements a Kalman Tracker that segments obstacles from the Sensor and tracks their position
 *
 *  Created on: 22.05.2015
 *      Author: Jonas Wirz <wirzjo@student.ethz.ch>
 */


#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "kt_tracker.h"

#include <systemlib/err.h>
#include <drivers/drv_hrt.h>

#include "kt_cog_list.h"
#include "kt_track_list.h"
#include "../pp_config.h"
#include "../pp_communication_buffer.h"
#include "../pp_navigation_helper.h"




/***********************************************************************************/
/*****  V A R I A B L E S  *********************************************************/
/***********************************************************************************/

static uint16_t dist_mat[2*SENSOR_RANGE/SENSOR_STEPSIZE];	//Matrix holding the distances measured for the given angles
static uint16_t dist_heading = 0; 	//Heading for which the dist_mat is valid (is transferred with the distance matrix) [°] (compass frame)
static uint16_t seg_mat[2*SENSOR_RANGE/SENSOR_STEPSIZE];   //Matrix holding the segment numbers


//#define C1 //sqrtf(2.0f*(1.0f-cosf(DEG2RAD*SENSOR_STEPSIZE)))


static struct {
	float c0;	//Configuration Parameter 0 in Calculation of Dlim
	float c1;   //Configuration Parameter 1 in Calculation of Dlim
} config = {
	.c0 = 10,
	.c1 = 0.034904812874445f //sqrtf(2.0f*(1.0f-cosf(DEG2RAD*SENSOR_STEPSIZE)))
};

static struct {
	uint16_t segnum;	//Number of the Segment (increased by one every time, when a new segment is detected
	bool newdata; 		//Flag that signals that new data is present
} state = {
	.segnum = 0,
	.newdata = false
};




/***********************************************************************************/
/*****  F U N C T I O N   P R O T O T Y P E S **************************************/
/***********************************************************************************/

/* @brief Segment the distance Matrix into Obstacles */
bool segment(void);

/* @brief Calcualte the COG of each identified Segment */
bool segment_COG(void);



/***********************************************************************************/
/*****  P U B L I C    F U N C T I O N S  ******************************************/
/***********************************************************************************/

/**
 * Init the Kalman Tracker
 *
 */
bool tr_init(void) {

	//Create a linked list with the tracking-objects
	tl_init();

	//Create a linked list with the newly found COGs
	cl_init();

	//Init Variables
	state.newdata = false;
	memset(&dist_mat,0,sizeof(dist_mat));
	memset(&seg_mat,0,sizeof(seg_mat));

	//Everything is OK and we can return true
	return true;
}


/**
 * Handler for the Kalman Tracking
 *
 */
bool tr_handler(void) {

	if(state.newdata == true) {
		//New measurement Data is present => we can do a Kalman Update

		printf("Tracker Handler called!\n");

		//Segment the distance matrix into segments with similar properties
		segment();

		//Find the COG for each segment
		segment_COG();

		//Update the linked list with the predicted Kalman states
		//tl_kalman_predict();

		//NNSF (Try to relate already tracked objects with the newly detected COGs)
		//Note: This step includes the Kalman-Update too
		//tl_nnsf();

		//Add the newly detected Tracking Objects to the list
		//tl_add_untracked();

		//Reset the new data flag
		state.newdata = false;
	}

	return true;

}


/**
 * Signal the module that new data is present such that a further Kalman-Step can be performed
 *
 * @param: new_dist_mat: Matrix containing the measured distances
 * @param: heading: Heading for which the measured distances are valid
 */
bool tr_newdata(uint16_t new_dist_mat[],uint16_t heading) {

	//printf("Tracker: Got new data!\n");

	for(uint8_t i=0; i<2*SENSOR_RANGE/SENSOR_STEPSIZE; i++) {
		//printf("%d/%d, ",i,new_dist_mat[i]);
		dist_mat[i] = new_dist_mat[i];
	}

	printf("Tracker: Got new data and stored them locally!\n");

	state.newdata = true;
	dist_heading = heading;

	return true;
}




/***********************************************************************************/
/*****  P R I V A T E    F U N C T I O N S  ****************************************/
/***********************************************************************************/


/**
 * Segment the Distance Matrix into Obstacles by calculating the COG of each detected Segment
 *
 * Note: This function is validated!
 */
bool segment(void) {

	state.segnum = 0;
	bool newnum = false;

	for (uint16_t ind = 0; ind < 2*SENSOR_RANGE/SENSOR_STEPSIZE-1; ind++) {
		//Loop over the whole Distance Matrix and try to identify segments

		//printf("%d,",ind);

        uint16_t r1 = dist_mat[ind];
        uint16_t r2 = dist_mat[ind+1];


        //Calculate Dlim
        float dlim = config.c0;
        if(r1<r2) {
        	dlim += config.c1 * r1;
        } else {
        	dlim += config.c1 * r2;
        }


        //Calculate D(r1,r2)
        float d = sqrtf(r1*r1 + r2*r2 - 2*r1*r2*cosf(DEG2RAD*SENSOR_STEPSIZE));


        //Decide if the two measurements belong to the same object
        if(d > dlim) {
        	//The points do NOT belong to the same object

            if(newnum == true) {
            	state.segnum = state.segnum + 1;
                newnum = false;
            }

        } else {
        	//The points belong to the same object

            seg_mat[ind] = state.segnum;
            seg_mat[ind+1] = state.segnum;

            newnum = true;
        }
	}

	printf("Segment: Found %d segments!\n",state.segnum+1);

	return true;
}



/**
 * Calculate the COG of each identified Segment
 */
bool segment_COG(void) {

	//Delete the old COGs if there are any
	cl_flush();

	//Iterate over the distance Matrix and calculate the COG's
	uint16_t seg = 0; //Segment Number
	float x_sum = 0;
	float y_sum = 0;
	uint16_t seg_length = 0; //Length of the Segment

	for (uint16_t ind = 0; ind < 2*SENSOR_RANGE/SENSOR_STEPSIZE-1; ind++) {

		float ang_deg = ind*SENSOR_STEPSIZE-SENSOR_RANGE+dist_heading;
		float angle = nh_mod(ang_deg*DEG2RAD); //Angle of the current measurement [rad]

		if(seg_mat[ind] == seg) {
			//We still calculate the COG of one segment

			//Increase the sum for the mean
			x_sum += cosf(angle)*dist_mat[ind];
			y_sum += sinf(angle)*dist_mat[ind];

			//Increase the number of measurement points by one
			seg_length++;


		} else {
			//A new segment is detected => store the COG in the matrix and restart the mean calculation

			//Store the COG of the Segment in the Matrix
			cl_add(x_sum/seg_length,y_sum/seg_length);

			printf("COG: %f/%f\n",(double)x_sum/seg_length,(double)y_sum/seg_length);

			//Take the new segment number as the reference
			seg = seg_mat[ind];

			//Reinit the calculation of the sum
			x_sum = cosf(angle)*dist_mat[ind];
			y_sum = sinf(angle)*dist_mat[ind];
			seg_length = 1;
		}

	}

	return true;
}














