/*
 * pp_failsafe.h
 *
 *  Created on: 30.03.2015
 *      Author: Jonas
 */

#ifndef PP_FAILSAFE_H_
#define PP_FAILSAFE_H_


#include "pp_topics_handler.h"


/* @brief Init the use of the failsafe */
void fs_init(void);


/* @brief Check, if the failsafe is active */
bool fs_is_failsafe_active(void);


/* @brief Check, if an RC-Signal is present */
void fs_check_rc_signal(struct pp_structs_topics_s *strs_pp);


/* @brief Main state-machine that is executed in the main-while-loop */
void fs_state_machine(void);


#endif /* PP_FAILSAFE_H_ */
