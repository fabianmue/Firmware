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
#include "../pp_config.h"
#include "../pp_communication_buffer.h"



/***********************************************************************************/
/*****  V A R I A B L E S  *********************************************************/
/***********************************************************************************/

uint16_t dist_mat[2*SENSOR_RANGE/SENSOR_STEPSIZE];	//Matrix holding the distances measured for the given angles
uint16_t seg_mat[2*SENSOR_RANGE/SENSOR_STEPSIZE];   //Matrix holding the segment numbers

//Definition of a COG (Center of Gravity)
typedef struct cog{
	float x;
	float y;
} cog;

cog *cog_mat;	//Matrix holding the found cog's of the segments


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

/*
 * Init the Kalman Tracker
 *
 * @param wx_port_pointer: Pointer to the COM-port handler
 */
bool tr_init(void) {

	//Create a linked list with the tracking-objects
	tl_init();

	//Everything is OK and we can return true
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

	//Delete the old COG's
	free(cog_mat);

	//Allocate Memory for the new COG's
	cog_mat = malloc(sizeof(cog) * (state.segnum+1));


	//Iterate over the distance Matrix and calculate the COG's
	for (uint16_t ind = 0; ind < 2*SENSOR_RANGE/SENSOR_STEPSIZE-1; ind++) {

		if()

	}



	return true;
}











