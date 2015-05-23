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
#include "kt_tracker.h"

#include <systemlib/err.h>
#include <drivers/drv_hrt.h>

#include "kt_track_list.h"
#include "kt_cog_list.h"
#include "../pp_config.h"
#include "../pp_communication_buffer.h"



/***********************************************************************************/
/*****  V A R I A B L E S  *********************************************************/
/***********************************************************************************/

uint16_t dist_mat[2*SENSOR_RANGE/SENSOR_STEPSIZE];	//Matrix holding the distances measured for the given angles
uint16_t dist_heading; 	//Heading for which the dist_mat is valid (is transferred with the distance matrix)
uint16_t seg_mat[2*SENSOR_RANGE/SENSOR_STEPSIZE];   //Matrix holding the segment numbers


static struct {
	float c0;	//Configuration Parameter 0 in Calculation of Dlim
	float c1;   //Configuration Parameter 1 in Calculation of Dlim
} config = {
	.c0 = 10,
	.c1 = sqrtf(2.0f*(1.0f-cosf(DEG2RAD*SENSOR_STEPSIZE)))
};

static struct {
	uint16_t segnum;	//Number of the Segment (increased by one every time, when a new segment is detected
} state = {
	.segnum = 0
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

	//Everything is OK and we can return true
	return true;
}


/**
 * Handler for the Kalman Tracking
 *
 */
bool tr_handler(void) {

	//Calcualte the time difference since the last measurement was done [s]
	float dt = 1; //TODO

	//Segment the distance matrix into segments with similar properties
	segment();

	//Find the COG for each segment
	segment_COG();

	//Update the linked list with the predicted Kalman states
	tl_kalman_predict(dt);

	//NNSF (Try to relate already tracked objects with the newly detected COGs)


	return true;
}





/***********************************************************************************/
/*****  P R I V A T E    F U N C T I O N S  ****************************************/
/***********************************************************************************/


/**
 * Segment the Distance Matrix into Obstacles by calculating the COG of each detected Segment
 *
 */
bool segment(void) {

	state.segnum = 0;
	bool newnum = false;

	for (uint16_t ind = 0; ind < 2*SENSOR_RANGE/SENSOR_STEPSIZE-1; ind++) {
		//Loop over the whole Distance Matrix and try to identify segments

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

            newnum = 1;
        }
	}

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

		float angle = 1; //Angle of the current measurement [rad] TODO

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














