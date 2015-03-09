/*
 * navigation_helper.h
 *
 *  Created on: 03.03.2015
 *      Author: Jonas Wirz (wirzjo@student.ethz.ch)
 */

#ifndef NAVIGATION_HELPER_H_
#define NAVIGATION_HELPER_H_

#include <stdint.h>
#include <math.h>


/* Type Definition for a GPS-Point defined by the Latitude and Longitude */
typedef struct {		 	 //Contains the GPS-Coordinate of a point
	float lat;				 //Latitude of a point [rad]
	float lon;				 //Longitude of a point [rad]
} Point;


/* @brief Calculate the bearing from one point to another */
float nh_geo_bearing(Point start, Point end);


/* @brief Calculate the distance between two points */
float nh_geo_dist(Point point1, Point point2);


/* @brief Calculate an approximated apparent Wind direction */
float nh_appWindDir(float heading, float windDir);


/* @brief Find the smallest element in an array */
uint8_t nh_findMin(const float *array, uint8_t arraySize);



/* @brief Convert from Compass to Dumas' Frame */
float nh_compass2dumas(float compass);


/* @brief Convert from Dumas' to Compass Frame */
float nh_dumas2compass(float dumas);


/* @brief Convert from Sensor to Dumas' Frame */
float nh_sensor2dumas(float sensor);


/* @brief Convert from Dumas' Frame to Sensor Frame */
float nh_dumas2sensor(float dumas);


#endif /* NAVIGATION_HELPER_H_ */
