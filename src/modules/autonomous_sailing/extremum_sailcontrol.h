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

//Include c-Libraries
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <drivers/drv_hrt.h>
#include <unistd.h>


#define BUFFERSIZE 8			   	//Maximum size of the Buffer for the Speeds

#define SAIL_CLOSED_PWM 0.56f  		//PWM signal for a fully closed sail
#define SAIL_OPEN_PWM   0.00f  		//PWM signal for a fully open sail
#define SAIL_CLOSED_DEG 0           //Sailangle when the sail is fully closed
#define SAIL_OPEN_DEG   80          //Sailangle when the sail is fully open

#define SAIL_INIT (SAIL_OPEN_DEG-SAIL_CLOSED_DEG)/2 //Initial Value for the Sail


/** @brief Initialize the use of Extremum Seeking Sailcontrol (ESSC) */
void essc_init(struct published_fd_s *pubs);


/** @brief Handle a new Speed-Value*/
void essc_speed_update(const struct structs_topics_s *strs_p);


/** @brief Return a Control-Value for the Sail Actuator*/
float essc_sail_control_value(float ds_pwm);


/** @brief Set Configuration Parameters from QGround Control */
void essc_set_qground_values(float k, int buffersize, float frequency);



#endif /* EXTREMUM_SAILCONTROL_H_ */
