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

#include "pathplanning_costfunction.h"



/**********************************************************************************************/
/****************************  VARIABLES  *****************************************************/
/**********************************************************************************************/

#define MAXOBSTACLES 10  	 				//Maximum number of obstacles
#define DEG2RAD      0.0174532925199433f 	//pi/180
#define PI           3.14159265358979323846f //pi
#define PIHALF	     1.57079632679f 		//pi/2
#define EARTHRADIUS  6371000.0f				//Earth Radius [m]

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


//Type definition for a GPS-Point
typedef struct {		 	 //Contains the GPS-Coordinate of a point
	float lat;				 //Latitude of a point [rad]
	float lon;				 //Longitude of a point [rad]
} Point;


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

/* @brief Calculate the distance between two points */
float dist(Point point1, Point point2);

/* @brief Calculate the heading from a point to a target point */
float bearing(Point start, Point end);

/* @brief Apparent Wind direction */
float appWindDir(float heading);

/* @brief Smooth an Array of variable size and with a variable windowSize */
void smooth(float *array, uint8_t arraySize , uint8_t windowSize);

/* @brief Return the speed at a given windangle and windspeed */
float polardiagram(float AppWindDir, float AppWindSpeed);

/* @brief Find the minimum in a matrix */
uint8_t findMin(const float *array, uint8_t arraySize);




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
	uint8_t minIndex = findMin(costMat,probeNum);

	//Get corresponding minimum Heading
	float optHeading = headMat[minIndex];



	//****DECISION MAKING
	/* In the following section the decisions based on the optimal Heading are made. This means
	 * that the corresponding controller is selected and the order for doing a maneuver is generated */
	float NewWind = appWindDir(optHeading); 		//New Apparent Winddirection
	float OldWind = appWindDir(State.heading);		//Current Apparent Winddirection


	//Decide if we have to do a tack
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
	//TODO

	//Decide if we are in the normal-sailing region
	//TODO

	//Decide if we are downwind sailing
	//TODO

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
	float lat = strs_p->gps_filtered.lat;
	float lon = strs_p->gps_filtered.lon;

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
	float appWind= appWindDir(seg);


	/************************/
	/*** WIND/TARGET COST ***/
	/************************/
	/* This cost takes the winddirection and the boats expected velocities at different relative angles towards
	 * the wind into account. Meanwhile it minimizes the distance towards the target. */
	//TODO: Potential Error-Source: Eventually check the implementation of the target-vector!
	float targetDir = bearing(State.position,Field.target); //Bearing to target

	float Tgx = cos(targetDir);								//Create a vector pointing towards the target
	float Tgy = sin(targetDir);
	float norm = sqrt(Tgx*Tgx + Tgy*Tgy);

	Tgx = Tgx/norm;											//Create the unity-Vector
	Tgy = Tgy/norm;

	float boatspeed = polardiagram(fabsf(appWind), State.windSpeed); //Create a vector representing the boat movement
	float Vhx = (float)cos(seg)*boatspeed;
	float Vhy = (float)sin(seg)*boatspeed;

	float Vg = Vhx*Tgx + Vhy*Tgy;									//Scalar-Product of Boatmovement and Target Vector

	float Cw = Vg*Config.Gw;								//Wind/Target Cost (weighted)



	/************************/
	/*** MANEUVRE COST    ***/
	/************************/
	/* This cost prevents the boat from tacking or gybing too often. Each tack or gybe slows down the boat. */

	//Calculate the current hull
	uint8_t oldhull = 0;
	if(appWindDir(State.heading) < 0) {
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

	//TODO:
	float Ct = 0;


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






	/************************/
	/*** TOTAL COST       ***/
	/************************/
	/* Calculate the total Cost and return the value  */

	return Cw + Co + Cm + Cs + Ct + CLee;
}



/**
 * Calculate the distance between two points
 *
 * Note: The calculation is partly copied from: http://www.movable-type.co.uk/scripts/latlong.html
 *
 * @param point1: Startpoint for the distance measurement
 * @param point2: Endpoint for the distance measurement
 * @return Distance between the point1 and point2 in meters
 */
float dist(Point point1, Point point2) {

	float a = sin((point2.lat-point1.lat)/2) * sin((point2.lat-point1.lat)/2) +
	        cos(point1.lat) * cos(point2.lat) *
	        sin((point2.lon-point1.lon)/2) * sin((point2.lon-point1.lon)/2);
	float c = 2 * atan2(sqrt(a), sqrt(1-a));

	return (EARTHRADIUS * c);
}



/**
 * Calculate the bearing from a point to another point.
 *
 * Note: The calculation is partly copied from: http://www.movable-type.co.uk/scripts/latlong.html
 *
 * @param start: Startpoint for the bearing measurement
 * @param end:   Endpoint for the bearing measurement
 * @return Bearing from Start ot Endpoint in rad. A true bearing is returned (element of [0:2pi])
 */
float bearing(Point start, Point end) {

	//Calculate the bearing
	float dx = cos(start.lat)*sin(end.lat)-sin(start.lat)*cos(end.lat)*cos(end.lon-start.lon);
	float dy = sin(end.lon-start.lon)*cos(end.lat);
	float beari = atan2(dy,dx);
	/* Note: atan2() returns a value between [-pi,pi]*/

	//transform to a true bearing [0,2pi]
	beari = fmod((beari + 2*PI),(2*PI));

	return beari;
}



/**
 * Calculate the apparent Wind Direction
 * This is not the real apparent Wind Direction. It is the direction the boat would measure if it is not moving.
 *
 * @param heading: True heading of the boat
 * @return apparent Wind angle. If positive, the wind comes from Portside, else from Starboard-side
 */
float appWindDir(float heading) {

	//Calculate Wind-Vector
    float xw = cos(State.windDir);
    float yw = sin(State.windDir);
    float wnorm = sqrt(xw*xw + yw*yw);

    //Calculate Heading-Vector
    float xh = cos(PI-State.heading);
    float yh = sin(PI-State.heading);
    float hnorm = sqrt(xh*xh + yh*yh);


    //Calculate Hull (Wind from Starboard <=> -1 ; Wind from Portside <=> 1)
    float zc = xw*yh - yw*xh;

    if(zc < 0) {
    	zc = -1;
    } else {
    	zc = 1;
    }

    return zc*(float)acos((xw*xh + yw*yh)/(wnorm*hnorm));

}



/**
 * Returns the speed to be expected from a Polardiagram
 * Note: only forward Speed is returned
 *
 * @param AppWindDir: Apparent Wind Direction [rad]
 * @param AppWindSpeed: Apparent Wind Speed [m/s]
 */
float polardiagram(float AppWindDir, float AppWindSpeed) {

	//TODO: Add the boats Polardiagram as a lookup table for different Windspeeds.

	return 1.0f;
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


	//Copy the array
	array = arrayCpy;

}



/**
 * Search for the minimum element in an array
 *
 * @param *array: Pointer to an array.
 * @return index of the minimum element in the array
 */
uint8_t findMin(const float *array, uint8_t arraySize) {

	uint8_t minInd = 0;
	float minimum = array[0];

	for(uint8_t i = 1; i < arraySize; i++) {
		if(array[i] < minimum) {
			minimum = array[i];
			minInd = i;
		}
	}

	return minInd;
}





