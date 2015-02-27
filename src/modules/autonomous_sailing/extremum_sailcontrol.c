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
float mean_speed(void);

/** @brief Add a value to the Buffer */
bool buffer_add(float value);

/** @brief Delete the oldest value from the Buffer */
bool buffer_delete_oldest(void);

/** @brief Flush the buffer */
bool buffer_flush(void);

/** @brief Get the value at a given buffer position */
float buffer_get_value(uint8_t pos);

/** @brief Convert the opening Angle for the sail in degrees to a PWM signal */
float deg2pwm(float degSail);

/** @brief Convert the opening Angle for the sail as a PWM signal to degrees */
float pwm2deg(float pwmSail);


/** Struct for a Circluar Buffer */
typedef struct {
	float bufferData [BUFFERSIZE];	//Array containing the Buffer-Data
	uint8_t head;					//Position of the head of the buffer (index in the array)
	uint8_t tail;              		//Position of the tail of the buffer (index in the array)
	uint8_t maxBuffersize;     		//Maximum possible Buffersize
	uint8_t buffersize;        		//Current size of the buffer
} CircularBuffer;


/** Struct holding the state of the Controller */
struct essc_state{
	CircularBuffer buffer;          //Buffer containing a limited number of Speed values used to build a mean (window-averaging)
	float meanSpeeds[2];			//Mean of the speedbuffer at times t-1 and t <=> meanSpeeds = [t-1,t] (in [m/s])
	float ds[3];					//Last three Sail Control-Values a times t,t-1,t-2 <=> ds = [t-2,t-1,t] (in [°])
	float ActDs;					//Current Sail Control-Value (as a PWM-Value)
	uint32_t lastCall; 		    	//Timestamp of the last Functioncall (in [s])
};


/** Struct holding the Configuration Parameters for the Controller */
struct essc_config{
	float k;						//Stepsize in Degrees
	unsigned int windowSize;		//Windowsize for building the mean over the speeds (window-averaging)
	float frequency;				//Frequency for Changes in the sail control value.
									//Note: In QGroundControl this value is defined as a frequency, but it is internally
									//      changed to a Time-Period => 1/frequency (for computational reasons).
};


/** Set default Values for the Configuration Parameters */
static struct essc_config Config = {
		.k = 2.0f,
		.frequency = 1.0f,
		.windowSize = 8
};


/** Set initial Values for the State Values */
static struct essc_state State = {
		.buffer = {{0},0,0,BUFFERSIZE,0},
		.meanSpeeds = {0},
		.ds = {0},
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
void essc_speed_update(const struct structs_topics_s *strs_p) {

	/* Get the boatspeed from the Kalman-Filtered values and calculate the forward
	 * u-Velocity
	 */
    float NorthVel = strs_p->gps_filtered.vel_n;
    float EastVel = strs_p->gps_filtered.vel_e;

    float uVel = sqrt(NorthVel * NorthVel + EastVel * EastVel);


    //Add the new Velocity to the buffer
    buffer_add(uVel);

    //Update the meanSpeeds-Matrix
	State.meanSpeeds[0] = State.meanSpeeds[1];
	State.meanSpeeds[1] = mean_speed();

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
	uint32_t ActTime = hrt_absolute_time();

	if(ActTime - State.lastCall >= Config.frequency*1000000) {
		/* A new Sail-Control-Value has to be calculated in this step */

		//Store the current time as the last time a new Sailvalue was calcualted
		State.lastCall = ActTime;

		//Calculate change in Speed (forward Speed)
		float du = State.meanSpeeds[1] - State.meanSpeeds[0];

		//Calcualte change in Sailangle (State.ds = [t-2,t-1,t])
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

        //Update the Sailcontrol-History
        State.ds[0] = State.ds[1];
        State.ds[1] = State.ds[2];
        State.ds[2] = newDs;

        //Set the new Control-Value
        State.ActDs = deg2pwm(newDs);
		return State.ActDs;

	} else {
		/* No new Sail-Control is to be set in this Step.
		 * Return the last calculated Sail-Control-Value as the new value
		 */

		return State.ActDs;

	}

} //End of ESSC_SailControlValue



/**
 * Get the Configuration Parameters from QGroundControl and assign them to the struct ESSC_Config
 *
 * @param k:Stepsize in Degrees [°]
 * @param windowSize: Size of the window for Speed-Averaging
 * @param frequency: Frequency for Sailadjustments [Hz]
*/
void essc_set_qground_values(float k, int windowSize, float frequency) {

	//Assign the Stepsize (make sure the stepsize is bigger than zero, else set a default value)
	if(k > 0) {
		Config.k = k;
	} else {
		k = 2.0f;
	}

	//Assign the Frequency
	//Note: frequency is a quantity in Hz. Internally the period is used. Therefore, the
	//      value is converted to the corresponding period and stored. If the frequency is
	//      smaller than zero a default value is set.
	if(frequency > 0) {
		Config.frequency = 1/frequency;
	} else {
		Config.frequency = 1/1.0f;
	}

	//Assign the Size of the Window for the speed Averaging (must be bigger than two, otherwise
	//a default value is set)
	if(windowSize >= 2) {
		Config.windowSize = windowSize;
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
float mean_speed(void) {

	/* Get the current number of elements currently available for Speed-Calculation. When starting the system or
	 * flushing the buffer it can happen that the buffer does not contain the full number of elements. Therefore,
	 * take the minimum of the current buffersize and the windowsize.
	 */
	uint8_t minSize = Config.windowSize;
	if(Config.windowSize > State.buffer.buffersize) {
		minSize = State.buffer.buffersize;
	}


	//Calculate the mean
	float sum = 0;
	for(uint8_t i = 0; i < minSize; i++) {
		sum += buffer_get_value(i);
	}

	return sum/minSize;

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



/**
 * Add a value to the RingBuffer
 *
 * @param   value: Value to be added to the buffer
 * @return	true, if the value was successfully added
*/
bool buffer_add(float value) {
	uint8_t next = (unsigned int)(State.buffer.head + 1) % State.buffer.maxBuffersize;

	if (next != State.buffer.tail) {
		//The buffer is not full => add value to the buffer

		State.buffer.bufferData[State.buffer.head] = value;
		State.buffer.head = next;
		State.buffer.buffersize = State.buffer.buffersize + 1;
	} else {
		//The buffer is full => delete oldest element and add the new value
		buffer_delete_oldest();
		buffer_add(value);
	}

	return true;

} //End of buffer_add



/**
 * Delete the oldest value from the RingBuffer
 *
 * @return true, if the value was successfully deleted
 */
bool buffer_delete_oldest(void) {

	if(State.buffer.head == State.buffer.tail) {
		//The buffer is empty
		return false;
	} else {
		//There is at least one element in the buffer
		State.buffer.tail = (uint8_t) (State.buffer.tail + 1) % State.buffer.maxBuffersize;
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
float buffer_get_value(uint8_t pos) {

	uint8_t index = (uint8_t)(State.buffer.tail + pos) % State.buffer.maxBuffersize;

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



