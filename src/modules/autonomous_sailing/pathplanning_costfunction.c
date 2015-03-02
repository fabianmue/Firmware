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


//Weights for the Cost-Function
static struct Weights {
	float Gw = 0.9;          //Weighting factor for sailing against the target while maximizing speed (was 0.5)
    float Go = 0.8;          //Weighting factor for avoiding obstacles (was 0.8)
	float Gm = 0.4;          //Weighting factor for avoiding maneuovres (was 0.5) (higher value <=> less maneuovres are allowed)
	float Gs = 0.05;         //Weighting factor for prefering courses that need no change in course
	float Gt = 0.1;          //Weighting factor for tactical considerations
	float GLee = 0.15;       //Weighting factor for passing Obstacles in Lee. (higher value <=> force boat to pass in Lee)
};


//Type definition for a GPS-Point
typedef struct {		 	 //Contains the GPS-Coordinate of a point
	float lat;				 //Latitude of a point [rad]
	float lon;				 //Longitude of a point [rad]
} Point;


//Race-Field-Definition
static struct {
	Point target;			//Target to be reached represented as a GPS-Position
	Point obstacles[MAXOBSTACLES]; //Matrix containing the positions of the obstacles (represented as GPS-Positions)
} Field;



static struct State {
	Point position;			//Current Position
	float windDir;			//Current Wind Direction
};



/* @brief Calculate the cost for a given heading */
float cost(float seg);

/* @brief Calculate the distance between two points */
float dist(Point point1, Point point2);

/* @brief Calculate the heading from a point to a target point */
float bearing(Point start, Point end);

/* @brief Apparent Wind direction */
float appWindDir(float heading);



/**********************************************************************************************/
/****************************  PUBLIC FUNCTIONS  **********************************************/
/**********************************************************************************************/

/**
 * Set the target to be reached.
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




/**********************************************************************************************/
/****************************  PRIVATE FUNCTIONS  *********************************************/
/**********************************************************************************************/

/**
 * Calculate the cost for a given (simulated) heading.
 *
 * @param	seg: the angle (true heading), the cost is calculated for
*/
float cost(float seg) {
	return 0.0f;
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
 *
 * @param start: Startpoint for the bearing measurement
 * @param end:   Endpoint for the bearing measurement
 * @return Bearing from Start ot Endpoint in rad. A true bearing is returned (element of [0:2pi])
 */
float appWindDir(float heading) {

}




