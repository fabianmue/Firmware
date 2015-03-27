/**
 * @file pp_potentialfield_method.c
 *
 * Implementation of the potential field method
 *
 * @author Jonas Wirz <wirzjo@student.ethz.ch>
 */

#include "pp_config.h"
#include "pp_potentialfield_method.h"


/**
 * Main Potentialfield Pathplanning Function
 * This Function returns the optimal heading according to the potentialfield method and the
 * according parameters.
 *
 * @param Pointer to the State-Struct
 * @param Pointer to the Field-Struct
 */
float pm_NewHeadingReference(struct nav_state_s *state, struct nav_field_s *field) {

	/*Iterate over the possible "probe" headings
	 * Note: In order to reduce computational costs only a range specified by the variables
	 * HEADRANGE and HEADRESOLUTION is used as possible headings. */
	float seg_start = (state->heading_cur-HEADRANGE);
	float seg_end = (state->heading_cur+HEADRANGE);

	uint8_t probeNum = 2*HEADRANGE/HEADRESOLUTION+1;	//Number of Probe-Headings
	float costMat[(int)(probeNum)];
	float headMat[(int)(probeNum)];

	uint8_t ind = 0;	//Index for addressing costMat-Elements

	float seg;
	for(seg = seg_start; seg <= seg_end; seg += HEADRESOLUTION) {
		float seg_compass = fmod(seg,(2*PI));

		//Get the cost and save it in the matrix
		costMat[ind] = total_cost(seg_compass,state,field);
		headMat[ind] = seg_compass;

		#if C_DEBUG == 1
			//printf("Total Cost: %f, %f\n",seg_compass*RAD2DEG,costMat[ind]);
		#endif

		//Update Index
		ind++;
	}

	//Smooth Cost-Matrix
	float mat[probeNum];
	smooth(costMat,probeNum,Config.WindowSize,mat);

	//Print the two arrays
	#if C_DEBUG == 1
	print_array(costMat,probeNum);
	print_array(mat,probeNum);
	#endif


	//Find minimum Index
	uint8_t minIndex = nh_findMin(costMat,probeNum);

	//Get corresponding minimum Heading
	float optHeading = headMat[minIndex];

	return optHeading;
}

