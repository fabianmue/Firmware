/*
 * mi_ack.h
 *
 *  Created on: 23.12.2015
 *      Author: Fabian
 */

#ifndef MI_ACK_H_
#define MI_ACK_H_

#include <stdint.h>
#include "../uORB.h"

struct mi_ack_s {

	uint64_t	timestamp;

	uint32_t	tar_ack;
	uint32_t	obs_ack;
};

/* register this as object request broker structure */
ORB_DECLARE(mi_ack);

#endif /* MI_ACK_H_ */
