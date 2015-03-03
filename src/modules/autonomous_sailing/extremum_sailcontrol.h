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

#define BUFFERSIZE 8			   	//Maximum size of the Buffer for the Speeds

#define SAIL_CLOSED_PWM 0.56f  		//PWM signal for a fully closed sail
#define SAIL_OPEN_PWM   0.00f  		//PWM signal for a fully open sail
#define SAIL_CLOSED_DEG 0           //Sailangle when the sail is fully closed
#define SAIL_OPEN_DEG   80          //Sailangle when the sail is fully open


/** @brief Handle a new Speed-Value*/
void essc_speed_update(const struct structs_topics_s *strs_p);


/** @brief Return a Control-Value for the Sail Actuator*/
float essc_sail_control_value(void);


/** @brief Set Configuration Parameters from QGround Control */
void essc_set_qground_values(float k, int buffersize, float frequency);



#endif /* EXTREMUM_SAILCONTROL_H_ */
