/*
 * extremum_sailcontrol.c
 *
 *  Extremum Seeking Sail Control (ESSC) is an algorithm to find the optimal sail angle for maximizing the speed of the boat.
 *  It is based on the approach presented in the following paper: "Online Speed Optimization for Sailing Yachts Using Extremum
 *  Seeking, L.Xiao, J.Alves, N.Cruz, J. Joeffroy, IEEE 2012".
 *
 *  Created on: 26.02.2015
 *      Author: Jonas Wirz (wirzjo@student.ethz.ch)
 */


#include "extremum_sailcontrol.h"


/** @brief Calculate the Signum of Speed/Sailcontrol (Signum according to the Paper) */
int sign(float value);


/** Set default Values for the Configuration Parameters */
static struct ESSC_Config Config = {
		.k = 2.0f,
		.frequency = 1.0f,
		.buffersize = 8};

/** Set initial Values for the State Values */
static struct ESSC_State State = {
		.ActBuffersize = 0,
		.meanspeed = 0,
		.ActDs = 0,
		.lastCall = 0
};




/**********************************************************************************************/
/****************************  PUBLIC FUNCTIONS  **********************************************/
/**********************************************************************************************/

/**
 * When a new speed value is available from the GPS, this function is called. The speed-value is
 * pushed into a buffer of variable length and the mean over the last speed-values is calculated.
 *
 * @param	strs_p: pointer to the struct for GPS Data
*/
void ESSC_SpeedUpdate(const struct structs_topics_s *strs_p) {

	State.meanspeed = 0.0f;

} //End of ESSC_SpeedUpdate



/**
 * Return a control signal for the sail actuator.
 *
 * @return	Signal for the sail actuator
*/
float ESSC_SailControlValue() {

	return 0.0f;

} //End of ESSC_SailControlValue



/**
 * Get the Configuration Parameters from QGroundControl and assign them to the struct ESSC_Config
 *
 * @param k:Stepsize
*/
void ESSC_SetQGroundValues(float k, int buffersize, float frequency) {

	//Assign the Stepsize (make sure the stepsize is bigger than zero)
	if(k > 0) {
		Config.k = k;
	} else {
		k = 2.0f;
	}

	//Assign the Frequency
	Config.frequency = frequency;

	//Assign the Size of the Buffer (must be bigger than two)
	if(buffersize >= 2) {
		Config.buffersize = buffersize;
	} else {
		Config.buffersize = 2;
	}

}







/**********************************************************************************************/
/****************************  PRIVATE FUNCTIONS  *********************************************/
/**********************************************************************************************/

/**
 * Return the signum of a value.
 * Note: The signum is defined in a special way according to the paper.
 *
 * @param 	value: The value for the signum calculation.
 * @return	int; 1, if value > 0; -1, else
*/
int sign(float value) {

	if(value>0) {
		return 1;
	} else {
		return -1;
	}

} //End of sign





