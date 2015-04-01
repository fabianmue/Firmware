/*
 * pp_failsafe.c
 *
 * This file contains functions that are used for failsafe.
 * A home-position is defined. As soon as the RC-Signal is lost, the boat
 * returns fully autonomously to this home-position (by Avoiding Obstacles and minimizing time to target :) )
 *
 *  Created on: 30.03.2015
 *      Author: Jonas Wirz <wirzjo@student.ethz.ch>
 */


#include <stdbool.h>
#include <drivers/drv_hrt.h>

#include "pp_failsafe.h"
#include "pp_send_msg_qgc.h"


#define SAFETYTIME_US  10e6 	//Time for which the RC-Signal must be lost before the failsafe is switched to ACTIVE [us]



typedef enum {INACTIVE, ACTIVE, SAFETYTIME} STATE_enum;
static STATE_enum SM_State = INACTIVE;

static uint64_t on_time = 0;	//Timestamp, when the RC-Signal was lost the first time

static bool lost_signal = false;		//true, iff the signal is lost, false else



/**
 * Init the failsafe-mode
 *
 */
void fs_init(void) {
	SM_State = INACTIVE; 		//By default the failsafe is off (inactive)
	on_time = 0;
	lost_signal = false; 		//By default we assume that a signal is present
}



/**
 * Is failsafe mode active?
 *
 * @return true, if the rc_signal is lost and the failsafe mode is active
 */
bool fs_is_failsafe_active(void) {
	if(SM_State == ACTIVE) {
		return true;
	} else {
		return false;
	}
}



/**
 * Get the state of the RC-Signal (Signal-Lost-Detection)
 *
 * This function sets the state of the failsafe mode
 */
void fs_check_rc_signal(struct structs_topics_s *strs_p) {

	if(strs_p->rc_channels.signal_lost == true) {
		//We lost the signal => update internal signal state
		lost_signal = true;

	} else {
		//We have a valid RC-Signal => update interal signal state
		lost_signal = false;
	}

}



/**
 * Run the state-machine
 *
 * Note: This function should be executed in every main-while-loop cycle
 */
void fs_state_machine(void) {
	//**** INACTIVE - state
	if(SM_State == INACTIVE) {
		if(lost_signal == true) {
			//The signal is lost => we switch to SAFETYTIME - state
			SM_State = SAFETYTIME;
			on_time = hrt_absolute_time();

			//Report to QGround-Control
			smq_send_log_info("RC-Signal lost => entering FAILSAFE soon! JW");
		}
	}


	//**** SAFETYTIME - state
	if(SM_State == SAFETYTIME) {
		if(lost_signal == false) {
			//we regained the signal => we can switch to INACTIVE - state
			SM_State = INACTIVE;
			return;	//Do not loose anymore time and return to INACTIVE
		}

		if((hrt_absolute_time() - on_time) >= SAFETYTIME_US) {
			//the signal is lost longer than the safetytime => we can switch to ACTIVE - state
			SM_State = ACTIVE;
		} else {
			SM_State = SAFETYTIME;
		}
	}


	//**** ACTIVE - state
	if(SM_State == ACTIVE) {

		if(lost_signal == false) {
			//We regained the signal => we can switch to INACTIVE - state
			SM_State = INACTIVE;
			return; //Do not loose anymore time and return to INACTIVE
		}

		if(lost_signal == true) {
			//We still have no signal => we stay in ACTIVE - state
			SM_State = ACTIVE;

			//Note: It could be that we reach the target (home-position), but we do not care,
			//		since even if we reach the target we want to stay there until we have a
			//      valid RC-Signal. => the only way of getting back into the INACTIVE - state
			//      is by regaining the RC-Signal.
		}

	}
}




