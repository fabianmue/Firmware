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
typedef struct Point_s{		 	 //Contains the GPS-Coordinate of a point
	float lat;				 //Latitude of a point [rad]
	float lon;				 //Longitude of a point [rad]

	float debug;
} Point;


/**Define a Position in NED-Coordinates */
typedef struct NEDpoint_s{
	float northx;			    //North Component of the NED-Point
	float easty;				//East Component of the NED-Point
	float downz;				//Down Component of the NED-Point
} NEDpoint;


/* @brief Calculate the bearing from one point to another (in GEO-Frame) */
float nh_geo_bearing(Point start, Point end);


/* @brief Calculate the distance between two points in GEO-Frame */
float nh_geo_dist(Point point1, Point point2);


/* @brief Calculate the distance between two points in NED-Frame */
float nh_ned_dist(NEDpoint point1, NEDpoint point2);


/* @brief Calcualte the bearing from one point to another (in GEO-Frame) */
float nh_ned_bearing(NEDpoint start, NEDpoint end);


/* @brief Calculate an approximated apparent Wind direction */
float nh_appWindDir(float heading, float windDir);


/* @brief Find the smallest element in an array */
uint8_t nh_findMin(const float *array, uint8_t arraySize);


/* @brief Convert a Point in the geo-Frame to the NED-Frame */
NEDpoint nh_geo2ned(Point geo);


/* @brief Convert from Compass to Dumas' Frame */
float nh_compass2dumas(float compass);


/* @brief Convert from Dumas' to Compass Frame */
float nh_dumas2compass(float dumas);


/* @brief Convert from Sensor to Dumas' Frame */
float nh_sensor2dumas(float sensor);


/* @brief Convert from Dumas' Frame to Sensor Frame */
float nh_dumas2sensor(float dumas);


/* @brief Convert from Sensor Frame to Compass Frame */
float nh_sensor2compass(float sensor);


/* @brief Convert from Compass Frame to Sensor Frame */
float nh_compass2sensor(float compass);


#endif /* NAVIGATION_HELPER_H_ */
