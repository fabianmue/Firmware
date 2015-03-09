/*
 * navigate.c
 *
 * This file contains functions needed for navigation. E.g. it contains functions for measuring bearings and distances
 *
 *  Created on: 03.03.2015
 *      Author: Jonas Wirz (wirzjo@student.ethz.ch)
 */


#include "pp_navigation_helper.h"
#include "pp_config.h"


#define EARTHRADIUS  6371000.0f				//Earth Radius [m]


/**
 * Calculate the bearing from a point to another point.
 *
 * Note: The calculation is partly copied from: http://www.movable-type.co.uk/scripts/latlong.html
 *
 * @param start: Startpoint for the bearing measurement
 * @param end:   Endpoint for the bearing measurement
 * @return Bearing from Start ot Endpoint in rad. A true bearing is returned (element of [0:2pi])
 *
 * Debug State: Function tested in PC-Test-Environment => working
 */
float nh_geo_bearing(Point start, Point end) {

	//Calculate the bearing
	float dx = cos(start.lat)*sin(end.lat)-sin(start.lat)*cos(end.lat)*cos(end.lon-start.lon);
	float dy = sin(end.lon-start.lon)*cos(end.lat);
	float beari = atan2f(dy,dx);
	/* Note: atan2() returns a value between [-pi,pi]*/

	//transform to a true bearing [0,2pi]
	beari = fmod((beari + 2*PI),(2*PI));

	return beari;
}



/**
 * Calculate the distance between two points
 *
 * Note: The calculation is partly copied from: http://www.movable-type.co.uk/scripts/latlong.html
 *
 * @param point1: Startpoint for the distance measurement
 * @param point2: Endpoint for the distance measurement
 * @return Distance between the point1 and point2 in meters
 *
 *  * Debug State: Function tested in PC-Test-Environment => needs to be validated, but seems to be OK
 */
float nh_geo_dist(Point point1, Point point2) {

	float a = sinf((point2.lat-point1.lat)/2) * sinf((point2.lat-point1.lat)/2) +
	        cosf(point1.lat) * cosf(point2.lat) *
	        sinf((point2.lon-point1.lon)/2) * sinf((point2.lon-point1.lon)/2);
	float c = 2 * atan2f(sqrt(a), sqrtf(1-a));

	return (EARTHRADIUS * c);
}



/**
 * Calculate the apparent Wind Direction
 * This is not the real apparent Wind Direction. It is the direction the boat would measure if it is NOT moving.
 *
 * @param heading: True heading of the boat
 * @return apparent Wind angle. If positive, the wind comes from Portside, else from Starboard-side
 *
 * Debug State: Function tested in PC-Test-Environment => working
 */
float nh_appWindDir(float heading, float windDir) {

	//Calculate Wind-Vector
    float xw = cosf(windDir);
    float yw = sinf(windDir);
    float wnorm = sqrtf(xw*xw + yw*yw);

    //Calculate Heading-Vector
    float xh = cosf(PI-heading);
    float yh = sinf(PI-heading);
    float hnorm = sqrtf(xh*xh + yh*yh);


    //Calculate Hull (Wind from Starboard <=> -1 ; Wind from Portside <=> 1)
    float zc = xw*yh - yw*xh;

    if(zc < 0) {
    	zc = -1;
    } else {
    	zc = 1;
    }

    return zc*acosf((xw*xh + yw*yh)/(wnorm*hnorm));

}



/**
 * Search for the minimum element in an array
 *
 * Note: The size of the Array is limited to 256 entries!
 *
 * @param *array: Pointer to an array.
 * @return index of the minimum element in the array
 */
uint8_t nh_findMin(const float *array, uint8_t arraySize) {

	uint8_t minInd = 0;
	float minimum = array[0];

	uint8_t i;
	for(i = 1; i < arraySize; i++) {
		if(array[i] < minimum) {
			minimum = array[i];
			minInd = i;
		}
	}

	return minInd;
}



/**
 * Convert from Compass to Dumas' Frame
 *
 * @param compass: Boat's heading in Compass-Frame [rad] element of [0...2pi]
 * @return Boat's heading in Dumas' Frame [rad]
 * */
float nh_compass2dumas(float compass) {

	//1) Convert from Compass to Sensor Frame
	if(compass > PI) {
		compass = compass - 2*PI;
	}

	//2) Convert from Sensor to Dumas' Frame
	return nh_sensor2dumas(compass);
}


/** Convert from Dumas' to Compass Frame
 *
 * @param dumas: Boat's heading in Dumas'-Frame [rad] element of [pi...0...-pi]
 * @return Boat's heading in Compass Frame [rad]
 * */
float nh_dumas2compass(float dumas) {

	//1) Convert from Duma to Sensor
	float sensor = nh_dumas2sensor(dumas);

	//2) Convert from Sensor to Compass
    if(sensor < 0) {
    	return 2*PI+sensor;
    } else {
    	return sensor;
    }
}


/** Convert from Sensor Frame to Dumas' Frame
 *
 * @param sensor: Boat's heading in Sensor-Frame [rad] element of [-pi...0...pi]
 * @return Boat's heading in Dumas' Frame
 */
float nh_sensor2dumas(float sensor) {
	//Transformation is a simple switch of Signs.
	return -sensor;
}


/** Convert from Dumas' Frame to Sensor Frame
 *
 * @param dumas: Boat's heading in Dumas'-Frame [rad] element of [pi...0...-pi]
 * @return Boat's heading in Sensor Frame
 */
float nh_dumas2sensor(float dumas) {
	//Transformation is a simple switch of Signs.
	return -dumas;
}

