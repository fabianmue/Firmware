/****************************************************************************
 *
 *   Copyright (c) 2012, 2013 PX4 Development Team. All rights reserved.
 *   Author: Marco Tranzatto <marco.tranzatto@gmail.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/
/**
 * @file parameter.h
 *
 * Functions to handle parameters from QGrooundControl
 *
 * @author Marco Tranzatto <marco.tranzatto@gmail.com>
 */

#ifndef PARAMETERS_H
#define PARAMETERS_H


#include <systemlib/param/param.h>
#include <stdio.h>//bool type

#include "controller_data.h"

#include "navigation.h"

#include "topics_handler.h"

#include "path_planning.h"

#include "guidance_module.h"

#include "extremum_sailcontrol.h" //(JW)

//settings
#include "settings.h"

#if PRINT_DEBUG_STR == 1
#include "send_msg_qgc.h"
#endif

//struct for local copy of parameter from QGroundControl
struct parameters_qgc{
    float sail_servo;

    #ifdef SIMULATION_FLAG

    float cog_sim;
    float twd_sim;
    float yaw_sim;
    float deva1;

    #endif
};


/** @brief Initialize parameters*/
void p_param_init(struct parameters_qgc *params_p,
                struct structs_topics_s *strs_p,
                const struct published_fd_s *pubs_p);

/** @brief Check if one or more parameters have been updated and perform appropriate actions*/
void p_param_update(struct parameters_qgc *params_p,
                  struct structs_topics_s *strs_p,
                  bool update_path_param,
                  const struct published_fd_s *pubs_p);


#endif // PARAMETERS_H
