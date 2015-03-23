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


#include <stdio.h>
#include <uORB/uORB.h>
#include "uORB/topics/boat_qgc_param.h"


#include "extremum_sailcontrol.h"


/** Struct for a Circluar Buffer */
typedef struct {
	float *bufferData_p;			//Pointer to an Array containing the Buffer-Data
	uint8_t head;					//Position of the head of the buffer (index in the array)
	uint8_t tail;              		//Position of the tail of the buffer (index in the array)
	uint8_t maxBuffersize;     		//Maximum possible Buffersize
	uint8_t buffersize;        		//Current size of the buffer
} CircularBuffer;

/** Pointer for logging on SD-Card */
struct published_fd_s *local_pubs; //Pointer to the published-Struct


/** @brief Calculate the Signum of Speed/Sailcontrol (Signum according to the Paper) */
int sign(float value);

/** @brief Build Mean of Speed */
float mean_speed(void);

/** @brief Init a circular Buffer of a given Size*/
CircularBuffer buffer_init(uint8_t buffersize);

/** @brief Update the Size of an existing Circular Buffer */
bool buffer_updateSize(CircularBuffer *buffer, uint8_t buffersize);

/** @brief Add a value to the Buffer */
bool buffer_add(CircularBuffer *buffer, float value);

/** @brief Delete the oldest value from the Buffer */
bool buffer_delete_oldest(CircularBuffer *buffer);

/** @brief Get the value at a given buffer position */
float buffer_get_value(CircularBuffer *buffer, uint8_t pos);

/** @brief Convert the opening Angle for the sail in degrees to a PWM signal */
float deg2pwm(float degSail);

/** @brief Convert the opening Angle for the sail as a PWM signal to degrees */
float pwm2deg(float pwmSail);

/** @brief Log usefull data for postprocessing on SD-Card */
void essc_log_data(void);



/** Struct holding the state of the Controller */
static struct {
	CircularBuffer buffer;          //Buffer containing a limited number of Speed values used to build a mean (window-averaging)
	float meanSpeeds[2];			//Mean of the speedbuffer at times t-1 and t <=> meanSpeeds = [t-1,t] (in [m/s])
	float ds[3];					//Last three Sail Control-Values a times t,t-1,t-2 <=> ds = [t-2,t-1,t] (in [°])
	float ActDs;					//Current Sail Control-Value (as a PWM-Value)
	uint64_t lastCall; 		    	//Timestamp of the last Functioncall (in [us])
} State = {
		.buffer = {0},				//Init the Buffer with all zeros
		.meanSpeeds = {0, 0},			//Init the Array with all zeros
		.ds = {SAIL_INIT,SAIL_INIT,SAIL_INIT},	//Init the Array with all zeros
		.ActDs = 0,
		.lastCall = 0
};


/** Struct holding the Configuration Parameters for the Controller */
static struct {
	float k;						//Stepsize in Degrees
	unsigned int windowSize;		//Windowsize for building the mean over the speeds (window-averaging)
	float period	;				//Timeinterval between two changes in the sail control value [us]
} Config = {
		.k = 2.0f,					//Init the values of the struct with the default values
		.period = 1000000.0f,		//Start with 1s Period-Time
		.windowSize = 8
};






/**********************************************************************************************/
/****************************  PUBLIC FUNCTIONS  **********************************************/
/**********************************************************************************************/

/**
 * Initialize the use of the Extremum Seeking Sailcontrol (ESSC)
 *
 *
 */
void essc_init(struct published_fd_s *pubs) {

	/* Create the Speed-Buffer */
	State.buffer = buffer_init(Config.windowSize);

	/* Store the pointer to the publish-topics-struct */
	local_pubs = pubs;

}



/**
 * When a new speed value is available from the GPS, this function is called. The speed-value is
 * pushed into a buffer of variable length and the mean over the last speed-values is calculated.
 *
 * @param	strs_p: pointer to the struct for GPS Data
*/
void essc_speed_update(const struct structs_topics_s *strs_p) {

	/* Get the boatspeed from the Kalman-Filtered values and calculate the forward
	 * u-Velocity
	 */
    float NorthVel = strs_p->vehicle_global_position.vel_n;
    float EastVel = strs_p->vehicle_global_position.vel_e;

    float uVel = sqrtf(NorthVel * NorthVel + EastVel * EastVel);

    buffer_add(&(State.buffer), uVel);

} //End of ESSC_SpeedUpdate



/**
 * Return a control signal for the sail actuator. This function is called by the guidance_module in irregular
 * timeintervals (due to scheduling).
 *
 * @return	Signal for the sail actuator
*/
float essc_sail_control_value() {

	/* The Sail Control should not change the value of the sail every time it is called.
	 * Therefore, the system-time is called to adjust the sail only in intervals specified by the
	 * QGroundControl Variable ESSC_frequency.
	 *
	 * Note: The following statement limits the frequency to a value of 2.3e-7 Hz
	 */
	uint64_t ActTime = hrt_absolute_time();

	if(ActTime - State.lastCall >= Config.period) {
		/* A new Sail-Control-Value has to be calculated in this step */

		//Store the current time as the last time a new Sailvalue was calcualted
		State.lastCall = ActTime;

		//Calculate change in Speed (forward Speed)
		float du = State.meanSpeeds[1] - State.meanSpeeds[0];

		//Calculate change in Sailangle (State.ds = [t-2,t-1,t])
		float ds = State.ds[1] - State.ds[0];

		//Actual Control Law <=> Calculate new Sail-Control-Value
		float newDs = State.ds[2] + Config.k * sign(ds) * sign(du);

		//Saturate the Sail-Command
        if(newDs > SAIL_OPEN_DEG) {
        	newDs = SAIL_OPEN_DEG;
        }

        if(newDs < SAIL_CLOSED_DEG) {
        	newDs = SAIL_CLOSED_DEG;
        }

        //Update the Sailcontrol-History (not nice, but should be fine here...)
        State.ds[0] = State.ds[1];
        State.ds[1] = State.ds[2];
        State.ds[2] = newDs;

        //Update the meanSpeeds-History (not nice, but should be fine here...)
    	State.meanSpeeds[0] = State.meanSpeeds[1];
    	State.meanSpeeds[1] = mean_speed();

        //Set the new Control-Value
        State.ActDs = deg2pwm(newDs);
	}

	return State.ActDs;

} //End of ESSC_SailControlValue



/**
 * Get the Configuration Parameters from QGroundControl and assign them to the struct ESSC_Config
 *
 * @param k:Stepsize in Degrees [°]
 * @param windowSize: Size of the window for Speed-Averaging
 * @param period: Time between two changes of the sailcontrol value [s]
*/
void essc_set_qground_values(float k, int windowSize, float period) {


	//Assign the Stepsize (make sure the stepsize is bigger than zero, else set a default value)
	if(k > 0) {
		Config.k = k;
	} else {
		//Set the default Value
		k = 2.0f;
	}

	//Assign the Period
	if(period > 0) {
		Config.period = period*1000000.0f;
	} else {
		//Set the default Value
		Config.period = 1000000.0f;
	}

	//Assign the Size of the Window for the speed Averaging (must be bigger than two, otherwise
	//a default value is set)
	if(windowSize >= 2) {
		Config.windowSize = windowSize;
		buffer_updateSize(&(State.buffer), windowSize);
	} else {
		//Set the default Value
		Config.windowSize = 8;
	}


	//The parameters are updated => add these parameters to the SD-Log
	essc_log_data();

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
float mean_speed(void) {

	//Calculate the mean
	float sum = 0;
	uint8_t i;
	for(i = 0; i < State.buffer.buffersize; i++) {
		sum += buffer_get_value(&(State.buffer),i);
	}

	return sum/State.buffer.buffersize;

} //End of meanSpeed



/**
 * Convert a sailangle in degrees to a PWM signal for the actuator.
 *
 * @param   degSail: Sailangle in Degrees
 * @return  sailangle as a PWM signal
 */
float deg2pwm(float degSail) {
	return (SAIL_OPEN_PWM-SAIL_CLOSED_PWM) / (SAIL_CLOSED_DEG-SAIL_OPEN_DEG) * (SAIL_OPEN_DEG-degSail);
} //End of deg2pwm



/**
 * Convert a sailangle as a PWM signal to degrees.
 *
 * @param   pwmSail: Sailangle as a PWM Signal
 * @return  sailangle in degrees
 */
float pwm2deg(float pwmSail) {
	return (SAIL_CLOSED_DEG-SAIL_OPEN_DEG) / (SAIL_OPEN_PWM-SAIL_CLOSED_PWM) * (SAIL_OPEN_PWM-pwmSail);
} //End of pwm2deg







/**************************************************************/
/**** DYNAMIC CIRCULAR BUFFER  ********************************/
/**************************************************************/

/**
 * Init a new Buffer of a given Size
 *
 * @return a circular Buffer
 */
CircularBuffer buffer_init(uint8_t buffersize) {

	CircularBuffer ret; 	//Create new Circular Buffer

	buffer_updateSize(&ret, buffersize);		//Set the correct length of the buffer

	return ret;
}



/**
 * Update the Size of a given Buffer
 *
 * @return true, if the size of the buffer was successfully changed
 */
bool buffer_updateSize(CircularBuffer *buffer, uint8_t buffersize) {

	//if(buffer == NULL) {
	//  //For some reason the buffer does not exist => create it.
	//	CircularBuffer newBuffer = buffer_init(Config.windowSize);
	//	buffer = &newBuffer;
	//}

	if(buffersize != buffer->maxBuffersize) {
		//The Size of the buffer has changed => initialize the new buffer

		//Delete the old buffer
		free(buffer->bufferData_p);

		//Allocate memory for the new buffer
		buffer->bufferData_p = malloc(sizeof(float) * buffersize);	//Allocate memory for the new buffer

		//Fill the new buffer with zeros
		//memset(buffer->bufferData_p, 0, buffer->buffersize);
		for(uint8_t i = 0; i < buffersize; i++) {
		        buffer->bufferData_p[i] = 0;
		}

		//Set the new maximum Buffersize
		buffer->maxBuffersize = buffersize + 1;
		buffer->buffersize = 0;
		buffer->head = 0;
		buffer->tail = 0;

		return true;
	}

	return false;
}


/**
 * Add a new Value to the buffer
 *
 * @param *buffer: Pointer to a Circluar Buffer
 * @param value: Value to be added to the buffer
 * @return true, if the value is added successfully
 */
bool buffer_add(CircularBuffer *buffer, float value) {

	uint8_t next = (unsigned int)(buffer->head + 1) % buffer->maxBuffersize;

	if (next != buffer->tail) {
		//The buffer is not full => add value to the buffer

		buffer->bufferData_p[buffer->head] = value;
		buffer->head = next;
		buffer->buffersize += 1;
	} else {
		//The buffer is full => delete oldest element and add the new value
		buffer_delete_oldest(buffer);
		buffer_add(buffer,value);
	}

	return true;
}



/**
 * Delete the oldest value from the RingBuffer
 *
 * @param *buffer: A pointer to a circular Buffer
 * @return true, if the value was successfully deleted
 */
bool buffer_delete_oldest(CircularBuffer *buffer) {

	if(buffer->head == buffer->tail) {
		//The buffer is empty
		return false;
	} else {
		//There is at least one element in the buffer
		buffer->tail = (uint8_t) (buffer->tail + 1) % buffer->maxBuffersize;
		buffer->buffersize -= 1;

		return true;
	}

} //End of buffer_deleteOldest



/**
 * Get the value at a certain position in the buffer
 *
 * Note: This function does not check if the element exists! It always returns a value,
 * even it lays outside of the buffer (due to the circularity).
 *
 * @param  *buffer: Pointer to a Circluar Buffer
 * @param  pos: Position in the buffer
 * @return the value at the given position in the buffer
 */
float buffer_get_value(CircularBuffer *buffer, uint8_t pos) {

	uint8_t ind = (uint8_t)(buffer->tail + pos) % buffer->maxBuffersize;

	return buffer->bufferData_p[ind];

} //End of buffer_getValue



/**
* Log some useful data for postprocessing
*/
void essc_log_data(void) {
	struct boat_qgc_param4_s temp_log;
	temp_log.timestamp = hrt_absolute_time();
	temp_log.k = Config.k;
	temp_log.windowsize = Config.windowSize;
	temp_log.period = Config.period;
	orb_publish(ORB_ID(boat_qgc_param4), local_pubs->boat_qgc_param4, &temp_log);
}









