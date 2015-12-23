/*
 * mp_params.h
 *
 *  Created on: 29.10.2015
 *      Author: Fabian
 */

#ifndef MP_PARAMS_H_
#define MP_PARAMS_H_

#include <systemlib/param/param.h>

#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#define DEG2RAD      0.0174532925199433f
#define PI           3.14159265358979323846f

#define MAX_CHAR_LINE 50
#define MAX_NUM_BU 4
#define MAX_NUM_WP 20
#define MAX_NUM_OB 20

#define SD_DEBUG 0

/***********************************************************************************/
/*****  V A R I A B L E S  *********************************************************/
/***********************************************************************************/

typedef struct waypoint_s {
	double latitude;
	double longitude;
} waypoint;

typedef struct obstacle_s {
	waypoint center;
	double radius;
} obstacle;

typedef struct buoy_s {
	obstacle body;
	int rotation;
} buoy;

typedef struct frame_s {
	char *name;
	int id;
	buoy buoys[MAX_NUM_BU];
	int buoy_count;
} frame;

typedef struct mission_s {
	char *name;
	int id;
	int type;
	int fr_id;
	waypoint waypoints[MAX_NUM_WP];
	int waypoint_count;
	obstacle obstacles[MAX_NUM_OB];
	int obstacle_count;
} mission;

/***********************************************************************************/
/*****  F U N C T I O N   D E C L A R A T I O N S  *********************************/
/***********************************************************************************/

/* @brief initialize parameters from QGC */
void mp_param_init(void);

/* @brief update parameters from QGC */
void mp_param_update(bool update_param);

/* @brief get parameters from SD card */
void mp_get_params_SD(char file_path[]);

/* @brief get parameters from serial port */
void mp_get_params_SP(void);

/* @brief send paramaters to serial port */
void mp_send_params_SP(char buffer[]);

waypoint* read_wp(int fd);

obstacle* read_ob(int fd);

buoy* read_bu(int fd);

frame* read_fr(int fd);

mission* read_mi(int fd);

char* read_line(int fd, int maxChar);

void disp_fr(frame *fr);

void disp_mi(mission *mi);

#endif /* MP_PARAMS_H_ */
