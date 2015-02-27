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

//Include the use of parameters over Telemtry
//#include "parameters.h"

#define BUFFERSIZE 20			   //Maximum size of the Buffer for the Speeds


/** @brief Handle a new Speed-Value*/
void ESSC_SpeedUpdate(const struct structs_topics_s *strs_p);


/** @brief Return a Control-Value for the Sail Actuator*/
float ESSC_SailControlValue(void);


/** @brief Set Configuration Parameters from QGround Control */
void ESSC_SetQGroundValues(float k, int buffersize, float frequency);


/** Struct for a Circluar Buffer */
typedef struct {
	float bufferData [BUFFERSIZE];	//Array containing the Buffer-Data
	unsigned int head;				//Position of the head of the buffer
	unsigned int tail;              //Position of the tail of the buffer
	unsigned int maxBuffersize;     //Maximum possible Buffersize
	unsigned int buffersize;        //Current size of the buffer
} CircularBuffer;


/** Struct holding the state of the Controller */
struct ESSC_State{
	CircularBuffer buffer;          //Buffer containing a limited number of Speed values used to build a mean
	float meanSpeed;				//Mean of the speeds in the buffer (updated as soon as the buffer is changed)
	float ds[3];					//Last three Sail Control-Values
	float ActDs;					//Current Sail Control-Value
	float lastCall; 				//Timestamp of the last Functioncall
};


/** Struct holding the Configuration Parameters for the Controller */
struct ESSC_Config{
	float k;						//Stepsize in Degrees
	int windowSize;					//Buffersize for building the mean over the speeds
	float frequency;				//Frequency for Changes in the sail control value
};




#endif /* EXTREMUM_SAILCONTROL_H_ */
