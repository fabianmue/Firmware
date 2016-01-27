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

/* @brief initialize QGC parameters */
void pp_param_QGC_init(void);

/* @brief get new paramater values from QGC */
void pp_param_QGC_get(bool update_path_param);

/* @brief set new mission paramater values in QGC */
void pp_param_QGC_set_mi(int id);

/* @brief get new target paramater values in QGC */
void pp_param_QGC_set_wp(float lat, float lon);

/* @brief get new obstacle paramater values in QGC */
void pp_param_QGC_set_ob(float lat, float lon, float rad);

/* @brief get new mi_ack paramater in QGC */
void pp_param_QGC_set_mi_ack(int wp_ack, int ob_ack);

#endif /* PP_PARAMETERS_H_ */
