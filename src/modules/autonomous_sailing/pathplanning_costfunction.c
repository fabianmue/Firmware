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

#define MAXOBSTACLES 10  	 //Maximum number of obstacles
#define DEG2RAD      0.0174532925199433f //pi/180
#define PI           3.14159265358979323846f //pi
#define EARTHRADIUS  6371000.0f		//Earth Radius [m]


//Weights and other configuration parameters for the Cost-Function-Method
static struct {
	float Gw = 0.9;          //Weighting factor for sailing against the target while maximizing speed (was 0.5)
    float Go = 0.8;          //Weighting factor for avoiding obstacles (was 0.8)
	float Gm = 0.4;          //Weighting factor for avoiding maneuovres (was 0.5) (higher value <=> less maneuovres are allowed)
	float Gs = 0.05;         //Weighting factor for prefering courses that need no change in course
	float Gt = 0.1;          //Weighting factor for tactical considerations
	float GLee = 0.15;       //Weighting factor for passing Obstacles in Lee. (higher value <=> force boat to pass in Lee)
	float ObstSafetyRadius = 10; //Safety Radius around an obstacle [m]
	float ObstHorizon = 100; //Obstacle Horizon <=> inside this horizon obstacles are recognized [m]
	float HeadResolution = 0.0872664625997f; //Resolution for simulating the headings in [rad] (here 5°)
	float HeadRange = 1.74532925199f; //Range for simulating the headings in [rad] (here [-100°...100°] wrt. boat-heading)
} Config;



//Type definition for a GPS-Point
typedef struct {		 	 //Contains the GPS-Coordinate of a point
	float lat;				 //Latitude of a point [rad]
	float lon;				 //Longitude of a point [rad]
} Point;


//Race-Field-Definition
static struct {
	Point target;			//Target to be reached represented as a GPS-Position
	Point obstacles[MAXOBSTACLES]; //Matrix containing the positions of the obstacles (represented as GPS-Positions)
	uint8_t NumberOfObstacles; 	//Number of Obstacles currently set
} Field;


//State of the System
static struct {
	Point position;			//Current Position
	float heading; 			//Current heading of the boat [rad]
	float windDir;			//Current Wind Direction [rad]
	float windSpeed; 		//Current Wind Speed [rad]
} State;



/* @brief Calculate the cost for a given heading */
float cost(float seg);

/* @brief Calculate the distance between two points */
float dist(Point point1, Point point2);

/* @brief Calculate the heading from a point to a target point */
float bearing(Point start, Point end);

/* @brief Apparent Wind direction */
float appWindDir(float heading);

/* @brief Smooth an Array of variable size and with a variable windowSize */
void smooth(float *array, uint8_t windowSize);




/**********************************************************************************************/
/****************************  PUBLIC FUNCTIONS  **********************************************/
/**********************************************************************************************/

/**
 * Main Pathplanning Function
 * This Function calculates the optimal heading according to the cost-function and calls the corresponding
 * Function dependent on the heading. It is simulates a navigator, as he would be present on a real boat.
 */
void navigator() {

	//Iterate over the possible "probe" headings
	float seg_start = (State.heading-Config.HeadRange);
	float seg_end = (State.heading+Config.HeadRange);

	float costMat[]

	for(float seg = seg_start; seg <= seg_start; seg += Config.HeadResolution) {
		seg = seg%(2*PI);

		cost(seg);

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
 * Set the position of the obstacle (from the worldserver)
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
 * @param HeadResolution
 * @param HeadRange
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
	float appWindDir = appWindDir(seg);


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

	float boatspeed = polardiagram(abs(appWindDir), State.windSpeed); //Create a vector representing the boat movement
	float Vhx = cos(seg)*boatspeed;
	float Vhy = sin(seg)*boatspeed;

	float Vg = Vhx*Tgx + Vhy*Tgy;									//Scalar-Product of Boatmovement and Target Vector

	float Cw = Vg*Config.Gw;								//Wind/Target Cost (weighted)



	/************************/
	/*** MANEUVRE COST    ***/
	/************************/
	/* This cost prevents the boat from tacking or gybing too often. Each tack or gybe slows down the boat. */

	//Calculate the current hull
	float oldhull = appWindDir(State.heading);
	if(oldhull < 0) {
		oldhull = -1;
	} else {
		oldhull = 1;
	}

	//Calculate the new hull
	uint8_t newhull = 0;
	if(appWindDir < 0) {
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
	float Ct;


	/*********************************/
	/*** SMALL HEADING CHANGE COST ***/
	/*********************************/
	/* Each change of course slows down the boat. Whenever possible a heading close to the current heading should be
	 * selected. */
	float Cs = Config.Gs * abs(seg-State.heading)/720.0f;



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
	float bearing = atan2(dy,dx);
	/* Note: atan2() returns a value between [-pi,pi]*/

	//transform to a true bearing [0,2pi]
	bearing = (bearing + 2*PI) % (2*PI);

	return bearing;
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

    return zc*acos((xw*xh + yw*yh)/(wnorm*hnorm));

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
 * @param &array: Pointer to an array. Note this array is directly modified!
 * @param windowSize: Size of the moving averaging window
 */
void smooth(float *array, uint8_t windowSize) {

	//Define Filter-Matrix
	float filter = 1/windowSize;

	//TODO: Convolution Array with filter

}





