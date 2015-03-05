/*
 * pathplanning_costfunction.h
 *
 * This file contains all functions/variables for calculating an optimal path to a given target position.
 *
 *  Created on: 02.03.2015
 *      Author: Jonas
 */

#ifndef PATHPLANNING_COSTFUNCTION_H_
#define PATHPLANNING_COSTFUNCTION_H_


#include <math.h>
#include <stdint.h>

#include "pp_topics_handler.h"




/* @brief Set the configuration Parameters from QGround Control */
void ppc_set_configuration(float Gw, float Go, float Gm, float Gs, float Gt, float GLee, float ObstSafetyRadius, float ObstHorizon);



/* @brief Calculate the optimal heading and give corresponding command to helsman*/
void cm_NewHeadingReference(void);





#endif /* PATHPLANNING_COSTFUNCTION_H_ */
