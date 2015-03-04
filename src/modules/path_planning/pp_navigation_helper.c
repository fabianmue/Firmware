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
 */
float nh_bearing(Point start, Point end) {

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
 */
float nh_dist(Point point1, Point point2) {

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

	for(uint8_t i = 1; i < arraySize; i++) {
		if(array[i] < minimum) {
			minimum = array[i];
			minInd = i;
		}
	}

	return minInd;
}



/**
 * Returns the speed to be expected from a Polardiagram
 * Note: only forward Speed is returned
 *
 * @param AppWindDir: Apparent Wind Direction [rad]
 * @param AppWindSpeed: Apparent Wind Speed [m/s]
 */
float nh_polardiagram(float AppWindDir, float AppWindSpeed) {

	//TODO: Add the boats Polardiagram as a lookup table for different Windspeeds.

	return 1.0f;
}


