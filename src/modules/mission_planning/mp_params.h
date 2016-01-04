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

#define MAX_CHAR_LINE 50
#define MAX_NUM_BU 4
#define MAX_NUM_WP 20
#define MAX_NUM_OB 10

#define SD_DEBUG 0

/***********************************************************************************/
/*****  V A R I A B L E S  *********************************************************/
/***********************************************************************************/

typedef struct rd_waypoint_s {
	char *name;
	double latitude;
	double longitude;
} rd_waypoint;

typedef struct rd_obstacle_s {
	char *name;
	rd_waypoint center;
	double radius;
} rd_obstacle;

typedef struct rd_buoy_s {
	char *name;
	rd_obstacle body;
	int rotation;
} rd_buoy;

typedef struct rd_frame_s {
	char *name;
	int id;
	rd_buoy buoys[MAX_NUM_BU];
	int buoy_count;
} rd_frame;

typedef struct rd_mission_base_s {
	char *name;
	int id;
	int type;
	int fr_id;
} rd_mission_base;

typedef struct rd_mission_uc_s {
	rd_mission_base mi_base;
	rd_waypoint waypoints[MAX_NUM_WP];
	int waypoint_count;
	rd_obstacle obstacles[MAX_NUM_OB];
	int obstacle_count;
} rd_mission_uc;

typedef struct rd_mission_tr_s {
	rd_mission_base mi_base;
	rd_buoy startB1;
	rd_buoy startB2;
	rd_buoy courseB1;
	rd_buoy courseB2;
	rd_buoy finishB1;
	rd_buoy finishB2;
} rd_mission_tr;

typedef struct rd_mission_sk_s {
	rd_mission_base mi_base;
	float time;
	float windDirAngle;
} rd_mission_sk;

typedef struct rd_mission_as_s {
	rd_mission_base mi_base;
	int subdivision;
	float windDirAngle;
} rd_mission_as;

/***********************************************************************************/
/*****  F U N C T I O N   D E C L A R A T I O N S  *********************************/
/***********************************************************************************/

/* @brief initialize parameters from QGC */
void mp_param_QGC_init(void);

/* @brief get parameters from QGC */
void mp_param_QGC_get(void);

/* @brief set parameters in QGC */
void mp_param_QGC_set(int *done);

// @brief update transfer to path_planning module
void mp_mission_update(int wp_ack, int ob_ack);

/* @brief get parameters from SD card */
void mp_get_mission_from_SD(char file_path[]);


rd_waypoint* read_wp(int fd);

rd_obstacle* read_ob(int fd);

rd_buoy* read_bu(int fd);

rd_frame* read_frame(int fd);

rd_mission_base* read_mission_base(int fd);

rd_mission_uc* read_mission_uc(int fd, rd_mission_base *rd_mi_base);

rd_mission_tr* read_mission_tr(int fd, rd_mission_base *rd_mi_base);

rd_mission_sk* read_mission_sk(int fd, rd_mission_base *rd_mi_base);

rd_mission_as* read_mission_as(int fd, rd_mission_base *rd_mi_base);

char* read_line(int fd, int maxChar);

#endif /* MP_PARAMS_H_ */
