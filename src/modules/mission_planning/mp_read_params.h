/*
 * mp_parameters.h
 *
 *  Created on: 29.10.2015
 *      Author: Fabian
 */

#ifndef MP_PARAMETERS_H_
#define MP_PARAMETERS_H_

#include <systemlib/param/param.h>

#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#define DEG2RAD      0.0174532925199433f
#define PI           3.14159265358979323846f

#define NAME_LEN 20
#define VALUE_LEN 20

/***********************************************************************************/
/*****  V A R I A B L E S  *********************************************************/
/***********************************************************************************/

extern double cf_o1n;
extern double cf_o1e;
extern double cf_dist;
extern double cf_rot;

typedef struct NEDpoint_s {
	float northx;	//north component
	float easty;	//east component
	float downz;	//down component
} NEDpoint;

typedef struct frame_s {
	NEDpoint O1; 	// position of the first buoy (topleft)
	NEDpoint O2;
	NEDpoint O3;
	NEDpoint O4;
	float dist;  	// distance between two buoys [m]
	float rotation; // rotation of the field around buoy O1 [rad]
} frame;

typedef struct SD_params_s {
	char name[NAME_LEN];
	double value;
} SD_params;

typedef struct pointers_param_qgc_mp_s{

	// geo reference
    param_t lat0_pointer;	// pointer to parameter ASP_R_LAT0_E7
    param_t lon0_pointer;	// pointer to parameter ASP_R_LON0_E7
    param_t alt0_pointer;	// pointer to parameter ASP_R_ALT0_E3

	// competition frame
	param_t cf_o1n;			// north component of O1
	param_t cf_o1e;			// east component of O1
	param_t cf_dist;		// buoy distance
	param_t cf_rotation;	// rotation of frame around O1

	// mission data
	param_t mp_mission;

} pointers_param_qgc_mp;

/***********************************************************************************/
/*****  F U N C T I O N   D E C L A R A T I O N S  *********************************/
/***********************************************************************************/


/* @brief read parameters from SD card */
void mp_read_param_SD(char file_path[]);

/* @brief initialize parameters from qgc */
void mp_read_param_QGC(void);

// @brief update parameters from qgc */
void mp_param_update_qgc(bool update_path_param);

/* @brief set GPS reference of buoy 1 */
void mp_set_ref0(const int32_t *lat, const int32_t *lon, const int32_t *alt);

/* @brief rotate NEDpoint angle around other NEDpoint */
NEDpoint mp_rotateNED(NEDpoint torot, NEDpoint center, float angle);

/* @brief set race field parameters */
void mp_set_racefield(float O1n, float O1e, float dist, float rotation);


#endif /* MP_PARAMETERS_H_ */
