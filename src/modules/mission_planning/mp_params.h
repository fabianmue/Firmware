/*
 * mp_params.h
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

#define MAX_CHAR_LINE 50
#define MAX_ELEM 20

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
	buoy buoys[4];
} frame;

typedef struct mission_s {
	char *name;
	int id;
	int type;
	int fr_id;
	waypoint waypoints[MAX_ELEM];
	obstacle obstacles[MAX_ELEM];
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

static int read_wp(int fd, waypoint wp);

static int read_ob(int fd, obstacle ob);

static int read_bu(int fd, buoy bu);

static int read_fr(int fd, frame fr);

static int read_mi(int fd, mission mi);

static char* read_line(int fd, int maxChar);

void disp_fr(frame fr);

void disp_mi(mission mi);

#endif /* MP_PARAMETERS_H_ */
