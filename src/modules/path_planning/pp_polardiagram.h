/*
 * pp_polardiagram_lookup.h
 *
 * This file contains the Polardiagram of the boat stored as a lookup table for different mean windspeeds.
 *
 *
 *
 *  Created on: 06.03.2015
 *      Author: Jonas Wirz <wirzjo@student.ethz.ch>
 */

#ifndef PP_POLARDIAGRAM_LOOKUP_H_
#define PP_POLARDIAGRAM_LOOKUP_H_


#define INTERVAL 0.0873f 	//Interval between two values in lookup table (5°) [rad]
#define STARTANG 0.3491f	//First angle that can be found in the lookup table (20°) [rad]



/** @brief Get the expected boatspeed at a given point of operation */
float pol_polardiagram(float wind_dir, float wind_speed);





#endif /* PP_POLARDIAGRAM_LOOKUP_H_ */
