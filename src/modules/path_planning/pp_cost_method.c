/*
 * pathplanning_costfunction.c
 *
 * This file contains all functions/variables for calculating an optimal path to a given target position.
 *
 * Notes:
 * 	- The whole pathplanning-module uses radians for angles <=> GPS-Positions have to be stored in Radians and not in degrees
 * 	- all angles are wrt. true north => bearings/headings are element of [0...2pi].
 *
 *  Created on: 02.03.2015
 *      Author: Jonas Wirz (wirzjo@student.ethz.ch)
 */

#include "pp_cost_method.h"


/**********************************************************************************************/
/****************************  VARIABLES  *****************************************************/
/**********************************************************************************************/

#define HEADRESOLUTION 0.0872664625997f 	//Resolution for simulating the headings in [rad] (here 5°)
#define HEADRANGE	   1.74532925199f   	//Range for simulating the headings in [rad] (here [-100°...100°] wrt. boat-heading)


//Weights and other configuration parameters for the Cost-Function-Method
static struct {
	float Gw;          		//Weighting factor for sailing against the target while maximizing speed (was 0.5)
    float Go;          		//Weighting factor for avoiding obstacles (was 0.8)
	float Gm;          		//Weighting factor for avoiding maneuovres (was 0.5) (higher value <=> less maneuovres are allowed)
	float Gs;         		//Weighting factor for prefering courses that need no change in course
	float Gt;          		//Weighting factor for tactical considerations
	float GLee;       		//Weighting factor for passing Obstacles in Lee. (higher value <=> force boat to pass in Lee)
	float ObstSafetyRadius; //Safety Radius around an obstacle [m]
	float ObstHorizon; 		//Obstacle Horizon <=> inside this horizon obstacles are recognized [m]
	float WindowSize; 		//Size of the window for smoothing the Costfunction
} Config = {
		.Gw = 0.9,
		.Go = 0.8,
		.Gm = 0.4,
		.Gs = 0.05,
		.Gt = 0.1,
		.GLee = 0.15,
		.ObstSafetyRadius = 10,
		.ObstHorizon = 100,
		.WindowSize = 5
};


/* @brief Calculate the total cost for a given simulated heading */
float total_cost(float seg, struct nav_state_s *state, struct nav_field_s *field);

/* @brief Get the cost for the wind and the target */
float cost_target_wind(float seg, struct nav_state_s *state, struct nav_field_s *field);

/* @brief Get the cost for avoiding maneuvers */
float cost_maneuver(float seg, struct nav_state_s *state);

/* @brief Get the cost for optimazing tactical considerations */
float cost_tactical(float seg,struct nav_state_s *state, struct nav_field_s *field);

/* @brief Get the cost for preventing the boat from selecting different headings too often */
float cost_heading_change(float seg,struct nav_state_s *state);

/* @brief Get the cost for avoiding obstacles and pass obstacles in lee */
float cost_ostacle(float seg, struct nav_state_s *state, struct nav_field_s *field);

/* @brief Smooth an Array of variable size and with a variable windowSize */
void smooth(float *array, uint8_t arraySize , uint8_t windowSize);





/**********************************************************************************************/
/****************************  PUBLIC FUNCTIONS  **********************************************/
/**********************************************************************************************/

/**
 * Main Pathplanning Function
 * This Function returns the optimal heading according to the cost-function and
 * the corresponding Parameters.
 *
 * @param Pointer to the State-Struct
 * @param Pointer to the Field-Struct
 */
float cm_NewHeadingReference(struct nav_state_s *state, struct nav_field_s *field) {

	/*Iterate over the possible "probe" headings
	 * Note: In order to reduce computational costs only a range specified by the variables
	 * HEADRANGE and HEADRESOLUTION is used as possible headings. */
	float seg_start = (state->heading_cur-HEADRANGE);
	float seg_end = (state->heading_cur+HEADRANGE);

	uint8_t probeNum = 2*HEADRANGE/HEADRESOLUTION;	//Number of Probe-Headings
	float costMat[(int)(probeNum)];
	float headMat[(int)(probeNum)];

	uint8_t ind = 0;	//Index for addressing costMat-Elements

	for(float seg = seg_start; seg <= seg_end; seg += HEADRESOLUTION) {
		seg = fmod(seg,(2*PI));

		//Get the cost and save it in the matrix
		costMat[ind] = total_cost(seg,state,field);
		headMat[ind] = seg;

		//Update Index
		ind++;
	}

	//Smooth Cost-Matrix
	smooth(costMat,probeNum,5);

	//Find minimum Index
	uint8_t minIndex = nh_findMin(costMat,probeNum);

	//Get corresponding minimum Heading
	float optHeading = headMat[minIndex];

	return optHeading;
}



/**
 * Set the configuration parameters for the costfunction method
 * This function is called by QGroundControl
 *
 * @param Gw
 * @param Go
 * @param Gm
 * @param Gs
 * @param Gt
 * @param GLee
 * @param ObstSafetyRadius
 * @param ObstHorizon
*/
void cm_set_onfiguration(float Gw, float Go, float Gm, float Gs, float Gt, float GLee, float ObstSafetyRadius, float ObstHorizon) {

	Config.Gw = Gw;
	Config.Go = Go;
	Config.Gm = Gm;
	Config.Gs = Gs;
	Config.Gt = Gt;
	Config.GLee = GLee;
	Config.ObstSafetyRadius = ObstSafetyRadius;
	Config.ObstHorizon = ObstHorizon;
}




/**********************************************************************************************/
/****************************  PRIVATE FUNCTIONS  *********************************************/
/**********************************************************************************************/

/**
 * Calculate the cost for a given (simulated) heading.
 *
 * @param	seg: the angle (true heading), the cost is calculated for [rad]
 * @param   *state: Pointer to the State-Struct
 * @param   *field: Pointer to the Field-Struct
 * @return  the total cost for a given simulated heading
*/
float total_cost(float seg, struct nav_state_s *state, struct nav_field_s *field) {

	/*** TARGET/WIND COST ***/
	float Cw = cost_target_wind(seg,state,field);


	/*** MANEUVER COST ***/
	float Cm = cost_maneuver(seg,state);


	/*** TACTICAL COST ***/
	float Ct = cost_tactical(seg,state,field);


	/*** SMALL HEADING CHANGE COST ***/
	float Cs = cost_heading_change(seg,state);


	/*** OBSTACLE COST ***/
	float Co = cost_ostacle(seg,state,field);


	/*** TOTAL COST ***/
	return (Cw + Co + Cm + Cs + Ct + Co);
}



/**
 * Target/Wind Cost
 *
 * This cost takes the winddirection and the boats expected velocities at different relative angles towards
 * the wind into account. Meanwhile it minimizes the distance towards the target.
 *
 *  TODO: Potential Error-Source: Eventually check the implementation of the target-vector!
 *
 *  @param seg: Simulated heading [rad]
 *  @param *state: Pointer to the State-Struct
 *  @param *field: Pointer to the Field-Struct
 *  @return Cost for the given simulated Heading
 */
float cost_target_wind(float seg, struct nav_state_s *state, struct nav_field_s *field) {

	float appWind= nh_appWindDir(seg,state->wind_dir);		//Current apparent Wind Direction

	float targetDir = nh_bearing(state->position,field->targets[state->targetNum]); //Bearing to target

	float Tgx = cosf(targetDir);							//Create a vector pointing towards the target
	float Tgy = sinf(targetDir);
	float norm = sqrtf(Tgx*Tgx + Tgy*Tgy);
	Tgx = Tgx/norm;											//Create the unity-Vector
	Tgy = Tgy/norm;

	float boatspeed = nh_polardiagram(fabsf(appWind), state->wind_speed); //Create a vector representing the boat movement
	float Vhx = cosf(seg);
	float Vhy = sinf(seg);
	norm = sqrtf(Vhx*Vhx + Vhy*Vhy);
	Vhx = Vhx/norm * boatspeed;
	Vhy = Vhy/norm * boatspeed;

	float Vg = Vhx*Tgx + Vhy*Tgy;							//Scalar-Product of Boatmovement and Target Vector

	return (Vg*Config.Gw);									//Wind/Target Cost (weighted)

} //end of cost_target_wind




/**
 * Maneuver Cost
 *
 * This cost prevents the boat from tacking or gybing too often. Each tack or gybe slows down the boat.
 *
 *  @param seg: Simulated heading [rad]
 *  @param *state: Pointer to the State-Struct
 *  @return Cost for the given simulated Heading
 */
float cost_maneuver(float seg, struct nav_state_s *state) {

	float appWind= nh_appWindDir(seg,state->wind_dir);		//Current apparent Wind Direction

	uint8_t oldhull = 0;									//Current hull
	if(nh_appWindDir(state->heading_cur,state->wind_dir) < 0) {
		//Wind from Starboard
		oldhull = -1;
	} else {
		//Wind from Portside
		oldhull = 1;
	}

	uint8_t newhull = 0;									//New hull
	if(appWind < 0) {
		//Wind from Starboard
		newhull = -1;
	} else {
		//Wind from Portside
		newhull = 1;
	}

	//Calculate the Maneuver Cost
	if(oldhull == newhull) {
		//Boat sails on the same hull <=> assign low cost
		return 0;
	} else {
		//Boat changes hull <=> tack or gybe <=> assign high cost
		return Config.Gm * 1;
	}

} //end of cost_maneuver



/**
 * Tactical Cost
 *
 * This cost account for tactical considerations.
 * 	1) The boat should prefer sailing with wind from starboard. This way it does NOT have to give way to
 * 	   boats sailing with wind from portside (COLREGS). This is explicitly important when sailing upwind.
 * 	2) The boat should not get too close to the laylines. Because sailing at the laylines is risky, because
 * 	   of possible windshifts. Therefore, the boat should prefere sailing close to the centerline.
 *
 *  @param seg: Simulated heading [rad]
 *  @param *state: Pointer to the State-Struct
 *  @param *field: Pointer to the Field-Struct
 *  @return Cost for the given simulated Heading
 */
float cost_tactical(float seg,struct nav_state_s *state, struct nav_field_s *field) {

	float appWind= nh_appWindDir(seg,state->wind_dir);		//Current apparent Wind Direction

	float Ct = 0;											//Tactical Cost that is returned at the end of this function

	/* Case 1) */
    if(appWind < 0) {
        Ct = 0;
    } else {
        Ct = Config.Gt * 1;
    }


    /* Case 2) */
    //TODO


    return Ct;

} //end of cost_tactical



/**
 * Small heading change cost
 *
 * Every change in heading causes movements with the rudder. Each rudder movement slows down the boat.
 * Therefore, whenever possible choose the same heading as the new heading reference as the boat is
 * currently sailing.
 *
 *  @param seg: Simulated heading [rad]
 *  @param *state: Pointer to the State-Struct
 *  @param *field: Pointer to the Field-Struct
 *  @return Cost for the given simulated Heading
 */
float cost_heading_change(float seg,struct nav_state_s *state) {

	return Config.Gs * fabsf(seg-state->heading_cur)/720.0f;

} //end of cost_heading_change



/**
 * Obstacle Cost
 *
 * Cost for maximising the distance to obstacles and avoid them.
 * 	1) Avoid Obstacles by avoiding selecting headings that guide the boat too close
 * 	   to the obstacles.
 * 	2) Whenever possible pass obstacles in lee, since this leaves more space for maneuvers.
 * 	   Especially on upwind courses, passing an obstacle in windward side could lead to an
 * 	   additional maneuver that slows down the boat drastically.
 */
float cost_ostacle(float seg, struct nav_state_s *state, struct nav_field_s *field) {

	float Co = 0;
	float CLee = 0;

	//Loop over all Obstacles
	for (uint8_t i = 0; i < field->NumberOfObstacles; i++) {

		float C = 0;	//Cost for this Obstacle

		/* Only obstacles within a certain Horizon are taken into account. This means that only obstacles in our range
		 * affect the Pathplanning. We do not care about far away obstacles. */
		float distance = nh_dist(state->position,field->obstacles[i]);
		if(distance > Config.ObstHorizon) {
			//Obstacle is too far away => continue with the next obstacle
			continue;
		}


		/* Add the safety radius to the obstacle. An obstacle is modelled as a point on the map. To avoid the obstacle
		 * and compensate for uncertainties (e.g. sudden changes in wind) the obstacle is made larger virtually */
		float obst_bear = nh_bearing(state->position,field->obstacles[i]);

		float ang_correction = atanf(Config.ObstSafetyRadius/distance);
		//TODO: Be careful here, if distance is close to zero we have a DIVISION BY ZERO!!!

		float max_obst_bear = fmod(ang_correction+obst_bear,2*PI);
		float min_obst_bear = fmod(obst_bear-ang_correction,2*PI);


		/* Check, if the boat is on collision course with an obstacle. It is on collision course,
		 * min_obst_bear < seg < max_obst_bear    <=> the course to check directly guides us towards
		 * an obstacle. */
		if((min_obst_bear < seg) && (seg < max_obst_bear )) {
			//Boat is on collision course
			C = Config.Go;
		} else {
			//Boat is NOT on collision course
			C = 0;
		}

		/* Update the TOTAL obstacle cost */
		Co = Co + C;



		/************************************/
		/************ PASS IN LEE ***********/
		/************************************/
		/* Whenever possible pass an obstacle in Lee. This way there is more room left for accounting for windshifts
		 * and compensating other uncertainties. This is especially important when sailing upwind, because windshifts could
		 * force the boat to do another tack, what slows down the boat and therefore has to be avoided.*/

		//TODO: add this here

	} //for

	return (Co + CLee);

} //end of cost_ostacle




/**
 * Smooths an array of variable size using a moving averaging window of variable size
 *
 * @param *array: Pointer to an array. Note this array is directly modified!
 * @param windowSize: Size of the moving averaging window
 */
void smooth(float *array, uint8_t arraySize , uint8_t windowSize) {

	//TODO: Make this faster. The method used here is extremly slow and computationally inefficient!

	float arrayCpy[arraySize];

	for(uint8_t i = windowSize/2; i<arraySize-windowSize/2; i++) {
		float sum = 0;
		for(uint8_t j = -windowSize/2; j<windowSize/2; j++) {
			sum += array[i+j];
		} //for j

		arrayCpy[i] = sum/windowSize;
	} //for i


	//Copy the array *THIS IS VERY BAD*
	array = arrayCpy;

} //end of smooth









