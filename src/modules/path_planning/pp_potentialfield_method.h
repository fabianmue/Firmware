/*
 * pp_potentialfield_method.h
 *
 *  Created on: 27.03.2015
 *      Author: Jonas
 */

#ifndef PP_POTENTIALFIELD_METHOD_H_
#define PP_POTENTIALFIELD_METHOD_H_

/* @brief Calculate the optimal heading and give corresponding command to helsman*/
float pm_NewHeadingReference(struct nav_state_s *state, struct nav_field_s *field);


#endif /* PP_POTENTIALFIELD_METHOD_H_ */
