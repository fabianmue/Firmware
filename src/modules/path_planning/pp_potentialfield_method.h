/*
 * pp_potentialfield_method.h
 *
 *  Created on: 27.03.2015
 *      Author: Jonas
 */

#ifndef PP_POTENTIALFIELD_METHOD_H_
#define PP_POTENTIALFIELD_METHOD_H_

#include "pp_navigator.h"

/* @brief Calculate the optimal heading and give corresponding command to helsman */
float pm_NewHeadingReference(struct nav_state_s *state, struct nav_field_s *field);


/* @brief Set weighting factors and Configuration from QGround Control */
void pm_set_configuration(float Gt, float Go, float Gm, float Gw, float SearchDist);


#endif /* PP_POTENTIALFIELD_METHOD_H_ */
