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

/** @brief Build Mean of Speed */
float meanSpeed(void);

/** @brief Add a value to the Buffer */
bool buffer_add(float value);

/** @brief Delete the oldest value from the Buffer */
bool buffer_deleteOldest(void);

/** @brief Flush the buffer */
bool buffer_flush(void);

/** @brief Get the value at a given buffer position */
float buffer_getValue(unsigned int pos);


/** Set default Values for the Configuration Parameters */
static struct ESSC_Config Config = {
		.k = 2.0f,
		.frequency = 1.0f,
		.windowSize = 8
};

/** Set initial Values for the State Values */
static struct ESSC_State State = {
		.buffer = {{0},0,0,BUFFERSIZE,0},
		.meanSpeed = 0,
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




	State.meanSpeed = 0.0f;

} //End of ESSC_SpeedUpdate



/**
 * Return a control signal for the sail actuator.
 *
 * @return	Signal for the sail actuator
*/
float ESSC_SailControlValue() {

	/* TODO: Add a time-measurement here, such that the sail-value is only updated with the specified frequency */


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
	if(frequency > 0) {
		Config.frequency = frequency;
	} else {
		Config.frequency = 1.0f;
	}

	//Assign the Size of the Buffer (must be bigger than two)
	if(buffersize >= 2) {
		Config.windowSize = buffersize;
	} else {
		Config.windowSize = 2;
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



/**
 * Return the mean speed of all values currently stored in the buffer
 *
 * @return	the mean speed of all values stored in the buffer
*/
float meanSpeed(void) {

	/* Get the current number of elements currently available for Speed-Calculation. When starting the system or
	 * flushing the buffer it can happen that the buffer does not contain the full number of elements. Therefore,
	 * take the minimum of the current buffersize and the windowsize.
	 */
	unsigned int minSize = Config.windowSize;
	if(Config.windowSize > State.buffer.buffersize) {
		minSize = State.buffer.buffersize;
	}


	//Calculate the mean
	float sum = 0;
	for(int i = 0; i < minSize; i++) {
		sum += buffer_getValue(i);
	}

	return sum/minSize;

} //End of meanSpeed



/**
 * Add a value to the RingBuffer
 *
 * @param   value: Value to be added to the buffer
 * @return	true, if the value was successfully added
*/
bool buffer_add(float value) {
	unsigned int next = (unsigned int)(State.buffer.head + 1) % State.buffer.maxBuffersize;

	if (next != State.buffer.tail) {
		//The buffer is not full => add value to the buffer

		State.buffer.bufferData[State.buffer.head] = value;
		State.buffer.head = next;
		State.buffer.buffersize = State.buffer.buffersize + 1;
	} else {
		//The buffer is full => delete oldest element and add the new value
		buffer_deleteOldest();
		buffer_add(value);
	}

	return true;

} //End of buffer_add



/**
 * Delete the oldest value from the RingBuffer
 *
 * @return true, if the value was successfully deleted
 */
bool buffer_deleteOldest(void) {

	if(State.buffer.head == State.buffer.tail) {
		//The buffer is empty
		return false;
	} else {
		//There is at least one element in the buffer
		State.buffer.tail = (unsigned int) (State.buffer.tail + 1) % State.buffer.maxBuffersize;
		State.buffer.buffersize = State.buffer.buffersize - 1;
		return true;
	}

} //End of buffer_deleteOldest



/**
 * Get the value at a certain position in the buffer
 *
 * @param  pos: Position in the buffer
 * @return the value at the given position in the buffer
 */
float buffer_getValue(unsigned int pos) {

	unsigned int index = (unsigned int)(State.buffer.tail + pos) % State.buffer.maxBuffersize;

	return State.buffer.bufferData[index];

} //End of buffer_getValue



/**
 * Flush the buffer
 *
 * @return true, if the buffer was successfully flushed
 */
bool buffer_flush(void) {

	State.buffer.head = State.buffer.tail;
	State.buffer.buffersize = 0;

	return true;

} //End of buffer_getValue



