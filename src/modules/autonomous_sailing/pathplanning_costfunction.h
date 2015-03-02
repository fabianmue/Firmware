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


/* @brief Set a new target point to be reached from QGround Control */
void ppc_set_target(float lat, float lon);


/* @brief Set the position of the obstacle */
void ppc_set_obstacle(float lat, float lon);


/* @brief Set the configuration Parameters from QGround Control */
void ppc_set_configuration(float Gw, float Go, float Gm, float Gs, float Gt, float GLee, float ObstSafetyRadius, float ObstHorizon, float HeadResolution, float HeadRange);


/* @brief Update the Winddirection from the Weatherstation */
void ppc_update_WSAI(const struct structs_topics_s *strs_p);


/* @brief Update the Vehicles Position based on GPS Data */
void ppc_update_GPOS(const struct structs_topics_s *strs_p);


/* @brief Update the Vehicles Heading */
void ppc_update_HEADING(const struct structs_topics_s *strs_p);





#endif /* PATHPLANNING_COSTFUNCTION_H_ */
