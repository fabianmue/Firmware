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

#include "pp_topics_handler.h"
#include "pp_config.h"
#include "pp_navigation_module.h"

/** @brief Initialize parameters*/
void param_init(struct parameters_qgc *params_p,
                struct structs_topics_s *strs_p,
                const struct published_fd_s *pubs_p);

/** @brief Check if one or more parameters have been updated and perform appropriate actions*/
void param_update(struct parameters_qgc *params_p,
                  struct structs_topics_s *strs_p,
                  bool update_path_param,
                  const struct published_fd_s *pubs_p);




#endif /* PP_PARAMETERS_H_ */
