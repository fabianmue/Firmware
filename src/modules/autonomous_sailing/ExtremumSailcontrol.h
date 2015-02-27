/*
 * ExtremumSailcontrol.h
 *
 *  Created on: 26.02.2015
 *      Author: Jonas Wirz (wirzjo@student.ethz.ch)
 */

#ifndef EXTREMUMSAILCONTROL_H_
#define EXTREMUMSAILCONTROL_H_

//Include Topics (for Variables)
#include "topics_handler.h"


/** @brief Handle a new Speed-Value*/
void ESSC_SpeedUpdate(const struct structs_topics_s *strs_p);


/** @brief Return a Control-Value for the Sail Actuator*/
float ESSC_SailControlValue();


/** @brief Array holding the last few Speed-Values */
float[BUFFERSIZE] speeds;


#endif /* EXTREMUMSAILCONTROL_H_ */
