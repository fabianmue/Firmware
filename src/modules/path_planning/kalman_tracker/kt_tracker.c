/*
 * kt_tracker.c
 *
 * This file implements a Kalman Tracker that segments obstacles from the Sensor and tracks their position
 *
 *  Created on: 22.05.2015
 *      Author: Jonas Wirz <wirzjo@student.ethz.ch>
 */


#include <stdbool.h>
#include <stdio.h>
#include "kt_tracker.h"

#include <systemlib/err.h>
#include <drivers/drv_hrt.h>

#include "../pp_config.h"
#include "../pp_communication_buffer.h"



/***********************************************************************************/
/*****  V A R I A B L E S  *********************************************************/
/***********************************************************************************/





/***********************************************************************************/
/*****  F U N C T I O N   P R O T O T Y P E S **************************************/
/***********************************************************************************/





/***********************************************************************************/
/*****  P U B L I C    F U N C T I O N S  ******************************************/
/***********************************************************************************/

/*
 * Init the Kalman Tracker
 *
 * @param wx_port_pointer: Pointer to the COM-port handler
 */
bool tr_init(void) {



	//Everything is OK and we can return true
	return true;
}





/***********************************************************************************/
/*****  P R I V A T E    F U N C T I O N S  ****************************************/
/***********************************************************************************/












