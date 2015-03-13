/*
 * navigate.c
 *
 * This file contains functions needed for navigation. E.g. it contains functions for measuring bearings and distances
 *
 *  Created on: 03.03.2015
 *      Author: Jonas Wirz (wirzjo@student.ethz.ch)
 */

#include "pp_config.h"
#include "pp_navigation_helper.h"

#if C_DEBUG == 0
#include "pp_navigation_module.h"
#endif


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
 * Calculate the distance between two points (NED-Frame)
 *
 *
 * @param point1: Startpoint for the distance measurement
 * @param point2: Endpoint for the distance measurement
 * @return Distance between the point1 and point2 in meters
 *
 * Debug State: Function tested in PC-Test-Environment. => working
 */
float nh_ned_dist(NEDpoint point1, NEDpoint point2) {

	float dx = point2.northx - point1.northx;
	float dy = point2.easty - point1.easty;

	float dist = sqrtf(dx * dx + dy * dy);

	/* Check distance for zero
	 * Otherwise, this could lead to a devision by zero! */
	if(dist <= 0.0000000001f) {
		dist = 0.0000000001f;
	}

	return dist;
}



/**
 * Calculate the bearing from one point to another point (NED-Frame)
 *
 * @param start: Startpoint for the bearing measurement
 * @param end: Endpoint for the bearing measurement
 * @return bearing from point start to point end [rad]
 *
 * Debug State: Function tested in PC-Test-Environment. => working
 */
float nh_ned_bearing(NEDpoint start, NEDpoint end) {

	float dx = end.northx - start.northx;
	float dy = end.easty - start.easty;

	/* atan2f() returns a result in Sensor-Convention => Convert to Compass-Convention */
	return nh_sensor2compass(atan2f(dy,dx));
}




/**
 * Calculate the apparent Wind Direction
 * This is not the real apparent Wind Direction. It is the direction the boat would measure if it is NOT moving.
 *
 * @param heading: True heading of the boat
 * @param windDir: Direction of where the wind is coming from [rad] (Sensor-Frame)
 * @return apparent Wind angle. If positive, the wind comes from Portside, else from Starboard-side
 *
 * Debug State: Function tested in PC-Test-Environment => working
 */
float nh_appWindDir(float heading, float windDir) {

	//Calculate Wind-Vector
    float xw = cosf(PI-windDir);
    float yw = sinf(PI-windDir);
    float wnorm = sqrtf(xw*xw + yw*yw);

    //Calculate Heading-Vector
    float xh = cosf(PI-heading);
    float yh = sinf(PI-heading);
    float hnorm = sqrtf(xh*xh + yh*yh);


    //Calculate Hull (Wind from Starboard <=> -1 ; Wind from Portside <=> 1)
    float zc = xw*yh - yw*xh;

    if(zc < 0) {
    	zc = 1;
    } else {
    	zc = -1;
    }

    return zc*acosf((xw*xh + yw*yh)/(wnorm*hnorm));

}


/*
 * Convert a point in geo-frame to NED-Frame
 *
 * @param geo: Point in geo-frame
 * @return Point in NED-Frame
 */
NEDpoint nh_geo2ned(Point geo) {
	NEDpoint result;

	int32_t north_dm;
	int32_t east_dm;
	int32_t down_dm;

	/*struct vehicle_global_position_s pos;
	pos.lat = geo.lat;
	pos.lon = geo.lon;
	pos.alt = geo.alt;*/

	//double lat = (double)geo.lat;
	//double lon = (double)geo.lon;
	//float alt = (float)geo.alt;

	n_geo_to_ned(geo.lat,geo.lon,geo.alt,&north_dm,&east_dm,&down_dm);

	result.northx = north_dm/10.0f;
	result.easty = east_dm/10.0f;
	result.downz = down_dm/10.0f;

	return result;
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



/*
 * Calculate the absolute difference between two headings in Compass Frame
 *
 * Note: This function accounts for the discontinuity from 360° -> 0°
 *
 * @param head1: First heading [rad]
 * @param head2: Second heading [rad]
 *
 * @return difference between the two headings [rad]
 */
float nh_heading_diff(float head1, float head2) {

	if(head1 > head2) {
		float temp = head2;
		head2 = head1;
		head1 = temp;
	}

	float dirA = head1-head2;		//Clockwise
	float dirB = 2*PI-head2 + head1;//Counterclockwise

	printf("dirA: %f / dirB: %f\n",dirA*RAD2DEG,dirB*RAD2DEG);

	float result = 0;

	if(fabs(dirA) <= fabs(dirB)) {
		result = -dirA;
	} else {
		result = dirB;
	}

	return result;
}




/**
 * Convert from Compass to Dumas' Frame
 *
 * @param compass: Boat's heading in Compass-Frame [rad] element of [0...2pi]
 * @return Boat's heading in Dumas' Frame [rad]
 * */
float nh_compass2dumas(float compass) {

	//1) Convert from Compass to Sensor Frame
	compass = nh_compass2sensor(compass);

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
	dumas = nh_dumas2sensor(dumas);

	//2) Convert from Sensor to Compass
    return nh_sensor2compass(dumas);
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



/** Convert from Sensor Frame to Compass Frame
 *
 * @param sensor: Boat's heading in Sensor-Frame [rad] element of [-pi...0...pi]
 * @return Boat's heading in Compass Frame
 */
float nh_sensor2compass(float sensor) {

	if(sensor < 0) {
	   	return 2*PI+sensor;
	} else {
	   	return sensor;
	}

}



/** Convert from Compass Frame to Sensor Frame
 *
 * @param compass: Boat's heading in Compass-Frame [rad] element of [0...2pi]
 * @return Boat's heading in Sensor Frame
 */
float nh_compass2sensor(float compass) {

	if(compass > PI) {
		return compass - 2*PI;
	} else {
		return compass;
	}
}




