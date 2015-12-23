/*
 * mp_mission.h
 *
 *  Created on: 25.05.2015
 *      Author: Jonas
 *
 *      09.10.2015: moved from path_planning to mission_planning app (Fabian Müller)
 */

#ifndef MP_MISSION_H_
#define MP_MISSION_H_

#include <stdint.h>
#include <stdbool.h>

#include "mp_params.h"
#include "mp_topics_handler.h"

/***********************************************************************************/
/*****  V A R I A B L E S  *********************************************************/
/***********************************************************************************/

#define MAX_NUM_MI 100
#define MAX_NUM_FR 10

/***********************************************************************************/
/*****  F U N C T I O N   D E C L A R A T I O N S  *********************************/
/***********************************************************************************/

void mp_mi_handler(int id);

void mp_tf_mi_init(void);

void mp_tf_mi_data(struct mp_structs_topics_s *strs);

int mp_add_mi_to_queue(mission *mi);

int mp_add_fr_to_queue(frame *fr);

#endif /* MP_MISSION_H_ */
