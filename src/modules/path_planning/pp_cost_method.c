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
#include "pp_config.h"
#include "pp_navigation_helper.h"



/**********************************************************************************************/
/****************************  VARIABLES  *****************************************************/
/**********************************************************************************************/

#define MAXOBSTACLES 10  	 				//Maximum number of obstacles

#define ANG_UPWIND   0.7854f				//Maximum Angle for the Upwind-Controller (45°) [rad]
#define ANG_NORMAL   2.0944f 				//Maximum Angle for the Normal-Controller (120°) [rad]
#define ANG_DOWNWIND 2.6180f				//Maximum Angle for the Downwind-Controller (150°) [rad]

#define HEADRESOLUTION 0.0872664625997f 	//Resolution for simulating the headings in [rad] (here 5°)
#define HEADRANGE	   1.74532925199f   	//Range for simulating the headings in [rad] (here [-100°...100°] wrt. boat-heading)


//Weights and other configuration parameters for the Cost-Function-Method
struct ppc_config{
	float Gw;          		//Weighting factor for sailing against the target while maximizing speed (was 0.5)
    float Go;          		//Weighting factor for avoiding obstacles (was 0.8)
	float Gm;          		//Weighting factor for avoiding maneuovres (was 0.5) (higher value <=> less maneuovres are allowed)
	float Gs;         		//Weighting factor for prefering courses that need no change in course
	float Gt;          		//Weighting factor for tactical considerations
	float GLee;       		//Weighting factor for passing Obstacles in Lee. (higher value <=> force boat to pass in Lee)
	float ObstSafetyRadius; //Safety Radius around an obstacle [m]
	float ObstHorizon; 		//Obstacle Horizon <=> inside this horizon obstacles are recognized [m]
	float WindowSize; 		//Size of the window for smoothing the Costfunction
};

static struct ppc_config Config = {
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





//Race-Field-Definition
struct ppc_Field{
	Point target;			//Target to be reached represented as a GPS-Position
	Point obstacles[MAXOBSTACLES]; //Matrix containing the positions of the obstacles (represented as GPS-Positions)
	uint8_t NumberOfObstacles; 	//Number of Obstacles currently set
};

static struct ppc_Field Field;



//State of the System
struct ppc_State{
	Point position;			//Current Position
	float heading; 			//Current heading of the boat [rad]
	float windDir;			//Current Wind Direction [rad]
	float windSpeed; 		//Current Wind Speed [rad]
};

static struct ppc_State State;



/* @brief Calculate the cost for a given heading */
float cost(float seg);

/* @brief Smooth an Array of variable size and with a variable windowSize */
void smooth(float *array, uint8_t arraySize , uint8_t windowSize);





/**********************************************************************************************/
/****************************  PUBLIC FUNCTIONS  **********************************************/
/**********************************************************************************************/

/**
 * Main Pathplanning Function
 * This Function calculates the optimal heading according to the cost-function and calls the corresponding
 * Function dependent on the heading. It is simulates a navigator, as he would be present on a real boat.
 */
void navigator(void) {

	//Iterate over the possible "probe" headings
	float seg_start = (State.heading-HEADRANGE);
	float seg_end = (State.heading+HEADRANGE);

	uint8_t probeNum = 2*HEADRANGE/HEADRESOLUTION;	//Number of Probe-Headings
	float costMat[(int)(probeNum)];
	float headMat[(int)(probeNum)];

	uint8_t ind = 0;	//Index for addressing costMat-Elements

	for(float seg = seg_start; seg <= seg_end; seg += HEADRESOLUTION) {
		seg = fmod(seg,(2*PI));

		//Get the cost and save it in the matrix
		costMat[ind] = cost(seg);
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



	//****DECISION MAKING
	/* In the following section the decisions based on the optimal Heading are made. This means
	 * that the corresponding controller is selected and the order for doing a maneuver is generated */
	float NewWind = nh_appWindDir(optHeading,State.windDir); 		//New Apparent Winddirection
	float OldWind = nh_appWindDir(State.heading,State.windDir);		//Current Apparent Winddirection


	/*Decide if we have to do a tack or a gybe
	* A maneuver is necessary, iff we change the hull. A change of hull is representet as a change of sign of the
	* apparent Wind direction.
	*/
	if(!((NewWind < 0 && OldWind < 0) || (NewWind > 0 && OldWind > 0))) {
		//A Maneuver is Necessary

		if(fabsf(NewWind) > PIHALF) {
			//A tack is necessary to reach the new optimal heading

			//TODO: Add the function from Marco here
		} else {
			//A gybe is necessary to reach the new optimal heading

			//TODO: Add the function from Marco here
		}

	}


	//Decide if we are upwind sailing
	if(fabsf(NewWind) <= ANG_UPWIND) {
		//TODO: Call the Upwind-Sailing-Controller
	}

	//Decide if we are in the normal-sailing region
	if((fabsf(NewWind) > ANG_UPWIND) && (fabsf(NewWind) < ANG_NORMAL)) {
		//TODO: Call the Normal-Sailing-Controller
	}

	//Decide if we are downwind sailing
	if((fabsf(NewWind) >= ANG_NORMAL) && (fabsf(NewWind) < ANG_DOWNWIND)) {
		//TODO: Call the Downwind-Sailing-Controller
	}

}



/**
 * Set the target to be reached. (usually done by QGround Control)
 *
 * @param	lat: latitude of the target [°]
 * @param   lon: longitude of the target [°]
*/
void ppc_set_target(float lat, float lon) {

	//Convert to radians
	lat = lat * DEG2RAD;
	lon = lon * DEG2RAD;

	//Set the Field-Variable
	Field.target.lat = lat;
	Field.target.lon = lon;
}



/**
 * Set the position of the obstacle (from the worldserver, done by QGround Control)
 *
 * @param	lat: latitude of the obstacle [°]
 * @param   lon: longitude of the obstacle [°]
*/
void ppc_set_obstacle(float lat, float lon) {

	//Assume that the obstacle set over the worldserver is always stored at position 0 in the Matrix
	Field.obstacles[0].lat = lat;
	Field.obstacles[0].lon = lon;

}



/**
 * Set the configuration parameters for the costfunction method
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
void ppc_set_configuration(float Gw, float Go, float Gm, float Gs, float Gt, float GLee, float ObstSafetyRadius, float ObstHorizon) {

	Config.Gw = Gw;
	Config.Go = Go;
	Config.Gm = Gm;
	Config.Gs = Gs;
	Config.Gt = Gt;
	Config.GLee = GLee;
	Config.ObstSafetyRadius = ObstSafetyRadius;
	Config.ObstHorizon = ObstHorizon;
}



/**
 * Update the winddirection
 *
 * @param	lat: latitude of the target [°]
 * @param   lon: longitude of the target [°]
*/
void ppc_update_WSAI(const struct structs_topics_s *strs_p) {

	//TODO
}



/**
 * Update the current position and heading
 *
 * @param	lat: latitude of the target [°]
 * @param   lon: longitude of the target [°]
*/
void ppc_update_GPOS(const struct structs_topics_s *strs_p) {

	//Update the Position
	//TODO
	float lat = strs_p->vehicle_global_position.lat;
	float lon = strs_p->vehicle_global_position.lon;

	State.position.lat = lat * DEG2RAD;
	State.position.lon = lon * DEG2RAD;

}



/**
 * Update the current heading
 *
 * @param	TODO
*/
void ppc_update_HEADING(const struct structs_topics_s *strs_p) {

	//TODO

	State.heading = 0.0f; //Store the current heading in radians

}




/**********************************************************************************************/
/****************************  PRIVATE FUNCTIONS  *********************************************/
/**********************************************************************************************/

/**
 * Calculate the cost for a given (simulated) heading.
 *
 * @param	seg: the angle (true heading), the cost is calculated for [rad]
*/
float cost(float seg) {

	//Calculate apparent Wind direction for the the current simulated heading
	float appWind= nh_appWindDir(seg,State.windDir);


	/************************/
	/*** WIND/TARGET COST ***/
	/************************/
	/* This cost takes the winddirection and the boats expected velocities at different relative angles towards
	 * the wind into account. Meanwhile it minimizes the distance towards the target. */
	//TODO: Potential Error-Source: Eventually check the implementation of the target-vector!
	float targetDir = nh_bearing(State.position,Field.target); //Bearing to target

	float Tgx = cosf(targetDir);								//Create a vector pointing towards the target
	float Tgy = sinf(targetDir);
	float norm = sqrtf(Tgx*Tgx + Tgy*Tgy);

	Tgx = Tgx/norm;											//Create the unity-Vector
	Tgy = Tgy/norm;

	float boatspeed = nh_polardiagram(fabsf(appWind), State.windSpeed); //Create a vector representing the boat movement
	float Vhx = (float)cosf(seg)*boatspeed;
	float Vhy = (float)sinf(seg)*boatspeed;

	float Vg = Vhx*Tgx + Vhy*Tgy;									//Scalar-Product of Boatmovement and Target Vector

	float Cw = Vg*Config.Gw;								//Wind/Target Cost (weighted)



	/************************/
	/*** MANEUVRE COST    ***/
	/************************/
	/* This cost prevents the boat from tacking or gybing too often. Each tack or gybe slows down the boat. */

	//Calculate the current hull
	uint8_t oldhull = 0;
	if(nh_appWindDir(State.heading,State.windDir) < 0) {
		oldhull = -1;
	} else {
		oldhull = 1;
	}

	//Calculate the new hull
	uint8_t newhull = 0;
	if(appWind < 0) {
		//Wind from Starboard
		newhull = -1;
	} else {
		newhull = 1;
	}

	//Calculate the Maneuver Cost
	float Cm = 0;
	if(oldhull == newhull) {
		//Boat sails on the same hull <=> assign low cost
		Cm = 0;
	} else {
		//Boat changes hull <=> tack or gybe <=> assign high cost
		Cm = Config.Gm * 1;
	}



	/************************/
	/*** TACTICAL COST    ***/
	/************************/
	/* The boat should not get too close to the laylines. Therefore, sailing close to the centerline is favorable. */

	float Ct = 0;
	/* Prefere Sailing with Wind from Starboard, since then other boats have to give way then => COLREGS*/
    if(appWind < 0) {
        Ct = 0;
    } else {
        Ct = Config.Gt * 1;
    }

    //TODO: Add the "stay close to centerline" cost here


	/*********************************/
	/*** SMALL HEADING CHANGE COST ***/
	/*********************************/
	/* Each change of course slows down the boat. Whenever possible a heading close to the current heading should be
	 * selected. */
	float Cs = Config.Gs * fabsf(seg-State.heading)/720.0f;



	/************************/
	/*** OBSTACLE COST    ***/
	/************************/
	/* Cost for maximizing the distance to obstacles. */

	float Co = 0;
	float CLee = 0;

	//Loop over all Obstacles
	for (uint8_t i = 0; i < Field.NumberOfObstacles; i++) {

		float C = 0;	//Cost for this Obstacle

		/* Only obstacles within a certain Horizon are taken into account. This means that only obstacles in our range
		 * affect the Pathplanning. We do not care about far away obstacles. */
		float distance = nh_dist(State.position,Field.obstacles[i]);
		if(distance > Config.ObstHorizon) {
			//Obstacle is too far away => continue with the next obstacle
			continue;
		}


		/* Add the safety radius to the obstacle. An obstacle is modelled as a point on the map. To avoid the obstacle
		 * and compensate for uncertainties (e.g. sudden changes in wind) the obstacle is made larger virtually */
		float obst_bear = nh_bearing(State.position,Field.obstacles[i]);

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


	}






	/************************/
	/*** TOTAL COST       ***/
	/************************/
	/* Calculate the total Cost and return the value  */

	return Cw + Co + Cm + Cs + Ct + CLee;
}



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

}









