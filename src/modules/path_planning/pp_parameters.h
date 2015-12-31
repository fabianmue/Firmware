/*
 * pp_parameters.h
 *
 * Handle changes in QGroundControl Parameters
 *
 *  Created on: 04.03.2015
 *      Author: Jonas Wirz <wirzjo@student.ethz.ch>
 *      Author: Marco Tranzatto <marco.tranzatto@gmail.com>
 */

#ifndef PP_PARAMETERS_H_
#define PP_PARAMETERS_H_

#include <systemlib/param/param.h>

#include "pp_topics_handler.h"
#include "pp_config.h"
#include "pp_navigation_module.h"
#include "pp_communication_buffer.h"
#include "pp_gridlines_handler.h"

/** @brief Initialize parameters*/
void pp_param_QGC_init(void);

/** @brief Check if one or more parameters have been updated and perform appropriate actions*/
void pp_param_QGC_get(bool update_path_param);




#endif /* PP_PARAMETERS_H_ */
