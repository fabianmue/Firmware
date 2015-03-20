/*
 * pp_polardiagram.c
 *
 * Return the expected Speed of the boat at a given Operating point.
 *
 *  Created on: 06.03.2015
 *      Author: Jonas Wirz <wirzjo@student.ethz.ch>
 *
 *  DEBUG-INFO: This function has been proved to be correct by testing it
 *              in a seperate C-Console-Program.
 */


#include "pp_polardiagram.h"

#include <stdint.h>
#include <stdio.h>
#include <math.h>




/********************************************************************/
/************************* LOOKUP TABLES ****************************/
/********************************************************************/
/*
 * The lookup table for the Polardiagram stores a value for every 5°.
 * For storage size reasons each value is multiplied by 170. Since the
 * theoretical hullspeed of the boat is limited to 1.25m/s a uint8_t
 * variable can be used to store one value.
 *
 * Further, the speed of the boat can be neglected, if the angle wrt.
 * wind is below 20°. Therefore, the lookup table covers a range of
 * [20°...180°]
 */

//Lookup Table for 3.0m/s mean Windspeed
static const uint8_t polar_3_0[33] =
{ 58,  67,  81,  94, 102, 111, 122, 123,
 126, 133, 138,	143, 144, 146, 150,	149,
 147, 145, 144,	142, 136, 128, 128,	121,
 113, 108, 108,	105, 100,  93,	88,	 82,
  77};





/********************************************************************/
/************************* FUNCTIONS  *******************************/
/********************************************************************/


/**
 * Return the speed of the boat at a given relative Wind direction for
 * a given mean Windspeed.
 *
 * @param wind_dir: realtive Winddirection wrt. the boat's middel axis [rad]
 * @param wind_speed: mean Windspeed [m/s]
 * @return expected Boatspeed [m/s] (-1 is returned in case of an error)
 */
float pol_polardiagram(float wind_dir, float wind_speed) {

	/* The sign of the Wind Direction is not important, since
	 * the polardiagram is assumed to be symmetric. */
	wind_dir = fabsf(wind_dir);

	/* Get Index that belongs to the given angle */
	if(wind_dir < STARTANG) {
		/* The Lookup Table only contains values for the range of
		 * [20°...180°]. Therefore an expected speed of zero can be
		 * returned for angles outside of this region */

		return 0;

	} else {
		/* The Lookup Table contains a value for the current angle */

		//Get the index of the angle in the Lookup table
		uint8_t ind = 0;

		wind_dir = roundf(wind_dir / INTERVAL)*INTERVAL;
		ind = (uint8_t)((wind_dir-STARTANG)/INTERVAL);

		//Check if the requested Index exists
		if(ind > 32) {
			return -1;
		}


		/* Decide which Lookup Table should be used */
		if(wind_speed > 0) {
			//The mean Speed is 3m/s

			return ((float)polar_3_0[ind]/170);
		} else {
			//printf("WSpeed: %f\n",wind_speed);
			return -1;
		}
	}
} //end of pol_polardiagram()










