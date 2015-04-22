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

#include "pp_config.h"




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
/*20°  25°  30°  35°  40°  45°  50°  55° */
{  0,   0, 106, 116, 126, 132, 140, 145, //was 76,91,106,116,126,132,140,145
/*60°  65°  70°  75°  80°  85°  90°  95° */
 150, 156, 161,	163, 166, 167, 169,	166,
/*100 105° 110° 115° 120° 125° 130° 135° */
 164, 162, 160,	153, 145, 141, 137,	129,
/*140 145° 150° 155° 160° 165° 170° 175° 180° */
 122, 121, 119,	112, 105,  99,	93,	 90,  87};

//ORIGINALLY USED!
//static const uint8_t polar_3_0[33] =
/*20°  25°  30°  35°  45°  50°  55°  60° */
//{  0,   0,   0,  94, 102, 111, 122, 123,
/*65°  70°  75°  80°  85°  90°  95° 100° */
// 126, 133, 138,	143, 144, 146, 150,	149,
/*105 110° 115° 120° 125° 130° 135° 140° */
// 147, 145, 144,	142, 136, 128, 128,	121,
/*145 150° 155° 160° 165° 170° 175° 180° */
// 113, 108, 108,	105, 100,  93,	88,	 82,
//  77};





/********************************************************************/
/************************* FUNCTIONS  *******************************/
/********************************************************************/


/**
 * Return the speed of the boat at a given relative Wind direction for
 * a given mean Windspeed.
 *
 * @param alpha: realtive Winddirection wrt. the boat's middel axis (=> alpha angle) [rad]
 * @param wind_speed: mean Windspeed [m/s]
 * @return expected Boatspeed [m/s] (-1 is returned in case of an error)
 */
float pol_polardiagram(float alpha, float wind_speed) {

	/* The sign of the Wind Direction is not important, since
	 * the polardiagram is assumed to be symmetric. */
	//alpha = fabsf(alpha);
	if(alpha<0) {
		alpha = (-1.0f)*alpha;
	}


	//DEBUG: Use a simplified version of the Polardiagram. It only specifies the No-Go-Zone and does not
	//       assume a speed that depends on the heading with respect to the wind.
#if LDEBUG_POLARDIAGRAM == 1
	if(alpha < 0.6981f) {
		return 0;
	} else {
		return 1;
	}
#endif




	/* Get Index that belongs to the given angle */
	if(alpha < STARTANG) {
		/* The Lookup Table only contains values for the range of
		 * [20°...180°]. Therefore an expected speed of zero can be
		 * returned for angles outside of this region */

		return 0;

	} else {
		/* The Lookup Table contains a value for the current angle */

		//Get the index of the angle in the Lookup table
		uint8_t ind = 0;

		//TODO: Check this function! => here must be the error!!!
		//printf("Alpha real: %f, ",alpha*RAD2DEG);
		alpha = roundf(alpha / INTERVAL)*INTERVAL;
		ind = (uint8_t)((alpha-STARTANG)/INTERVAL);
		//printf("Alpha round: %f, Index: %d\n",alpha*RAD2DEG,ind);

		//Check if the requested Index exists
		if(ind > 32) {
			return -1;
		}


		/* Decide which Lookup Table should be used */
		if(wind_speed > 0) {
			//The mean Speed is 3m/s

			return (((float)polar_3_0[ind])/160.0f);
		} else {
			//printf("WSpeed: %f\n",wind_speed);
			return -1;
		}
	}
} //end of pol_polardiagram()










