/*
 * pp_parameters.h
 *
 * Handle changes in QGroundControl Parameters
 *
 *  Created on: 04.03.2015
 *      Author: Jonas Wirz <wirzjo@student.ethz.ch>
 */

#ifndef PP_PARAMETERS_H_
#define PP_PARAMETERS_H_

#include <systemlib/param/param.h>



/** @brief Initialize parameters*/
void p_param_init(void);

/** @brief Check if one or more parameters have been updated and perform appropriate actions*/
void p_param_update(void);




#endif /* PP_PARAMETERS_H_ */
