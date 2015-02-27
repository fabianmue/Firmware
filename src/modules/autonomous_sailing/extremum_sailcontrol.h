/*
 * extremum_sailcontrol.h
 *
 *  Created on: 26.02.2015
 *      Author: Jonas Wirz (wirzjo@student.ethz.ch)
 */

#ifndef EXTREMUM_SAILCONTROL_H_
#define EXTREMUM_SAILCONTROL_H_

//Include Topics (for Variables)
#include "topics_handler.h"

//Include the math library
#include <math.h>

#define BUFFERSIZE 20			   	//Maximum size of the Buffer for the Speeds

#define SAIL_CLOSED_PWM 0.56f  		//PWM signal for a fully closed sail
#define SAIL_OPEN_PWM   0.00f  		//PWM signal for a fully open sail
#define SAIL_CLOSED_DEG 0           //Sailangle when the sail is fully closed
#define SAIL_OPEN_DEG   80          //Sailangle when the sail is fully open


/** @brief Handle a new Speed-Value*/
void ESSC_SpeedUpdate(const struct structs_topics_s *strs_p);


/** @brief Return a Control-Value for the Sail Actuator*/
float ESSC_SailControlValue(void);


/** @brief Set Configuration Parameters from QGround Control */
void ESSC_SetQGroundValues(float k, int buffersize, float frequency);


/** Struct for a Circluar Buffer */
typedef struct {
	float bufferData [BUFFERSIZE];	//Array containing the Buffer-Data
	uint8_t head;					//Position of the head of the buffer
	uint8_t tail;              		//Position of the tail of the buffer
	uint8_t maxBuffersize;     		//Maximum possible Buffersize
	uint8_t buffersize;        		//Current size of the buffer
} CircularBuffer;


/** Struct holding the state of the Controller */
struct ESSC_State{
	CircularBuffer buffer;          //Buffer containing a limited number of Speed values used to build a mean (window-averaging)
	float meanSpeeds[2];			//Mean of the speedbuffer at times t-1 and t <=> meanSpeeds = [t-1,t]
	float ds[3];					//Last three Sail Control-Values a times t,t-1,t-2 <=> ds = [t-2,t-1,t]
	float ActDs;					//Current Sail Control-Value
	uint16_t lastCall; 		    	//Timestamp of the last Functioncall
};


/** Struct holding the Configuration Parameters for the Controller */
struct ESSC_Config{
	float k;						//Stepsize in Degrees
									//Note: In QGroundControl this value is defined as an angle in degrees. Internally
									//      the angle is transformed into a number representing a PWM-Signal.
	unsigned int windowSize;		//Buffersize for building the mean over the speeds
	float frequency;				//Frequency for Changes in the sail control value.
									//Note: In QGroundControl this value is defined as a frequency, but it is internally
									//      changed to a Time-Period => 1/frequency (for computational reasons).
};




#endif /* EXTREMUM_SAILCONTROL_H_ */
