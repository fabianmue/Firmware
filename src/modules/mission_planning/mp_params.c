/*
 * mp_parameters.c
 *
 *  Created on: 29.10.2015
 *      Author: Fabian
 */

#include <systemlib/param/param.h>

#include <ctype.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include "mp_params.h"

#include <nuttx/config.h>
#include <fcntl.h>

#include "../path_planning/pp_navigation_module.h"
#include "../path_planning/pp_navigation_helper.h"

/***********************************************************************************/
/*****  V A R I A B L E S  *********************************************************/
/***********************************************************************************/

/**
 *  mission planner QGC variables
 */
PARAM_DEFINE_INT32(MP_DATA_SRC, 0);		// data source, 0 = telemetry, 1 = SD card

PARAM_DEFINE_INT32(MP_TAR_LAT, 0);		// current target latitude
PARAM_DEFINE_INT32(MP_TAR_LON, 0);		// current target longitude
PARAM_DEFINE_FLOAT(MP_TAR_NED_N, 0);	// current target NED north
PARAM_DEFINE_FLOAT(MP_TAR_NED_E, 0);	// current target NED east

PARAM_DEFINE_INT32(MP_CUR_FR_ID, 0);	// current frame id
PARAM_DEFINE_INT32(MP_CUR_MI_ID, 0);	// current mission id
PARAM_DEFINE_INT32(MP_CUR_MI_TYPE, 0);	// current mission type

PARAM_DEFINE_INT32(MP_OB_NUM, 0);		// number of obstacles currently set

static struct pointers_mp_param_qgc_s {

	param_t mp_data_src;

	param_t mp_tar_lat;
	param_t mp_tar_lon;
	param_t mp_tar_ned_n;
	param_t mp_tar_ned_e;

	param_t mp_cur_fr_id;
	param_t mp_cur_mi_id;
	param_t mp_cur_mi_type;

	param_t mp_ob_num;

	param_t mp_start1_lat;
	param_t mp_start1_lon;
	param_t mp_start2_lat;
	param_t mp_start2_lon;

} pointers_mp_param_qgc;

/***********************************************************************************/
/*****  F U N C T I O N   D E F I N I T I O N S  ***********************************/
/***********************************************************************************/

void mp_param_init(void) {

    pointers_mp_param_qgc.mp_data_src = param_find("MP_DATA_SRC");

    pointers_mp_param_qgc.mp_tar_lat = param_find("MP_TAR_LAT");
    pointers_mp_param_qgc.mp_tar_lon = param_find("MP_TAR_LON");
    pointers_mp_param_qgc.mp_tar_ned_n = param_find("MP_TAR_NED_N");
    pointers_mp_param_qgc.mp_tar_ned_e = param_find("MP_TAR_NED_E");

    pointers_mp_param_qgc.mp_cur_fr_id = param_find("MP_CUR_FR_ID");
    pointers_mp_param_qgc.mp_cur_mi_id = param_find("MP_CUR_MI_ID");
    pointers_mp_param_qgc.mp_cur_mi_type = param_find("MP_CUR_MI_TYPE");

    pointers_mp_param_qgc.mp_ob_num = param_find("MP_OB_NUM");

    pointers_mp_param_qgc.mp_start1_lat = param_find("MP_START1_LAT");
    pointers_mp_param_qgc.mp_start1_lon = param_find("MP_START1_LON");
    pointers_mp_param_qgc.mp_start2_lat = param_find("MP_START2_LAT");
    pointers_mp_param_qgc.mp_start2_lon = param_find("MP_START2_LON");

    mp_param_update(false);
};

void mp_param_update(bool update_param) {

	int32_t data_src;
	PointE7 target;
	NEDpoint target_ned;
	mission cur_mission;
	uint8_t ob_num;
	int32_t altitude;

	param_get(pointers_mp_param_qgc.mp_data_src, &data_src);

   	param_get(pointers_mp_param_qgc.mp_tar_lat, &(target.lat));
   	param_get(pointers_mp_param_qgc.mp_tar_lon, &(target.lon));
   	param_get(pointers_mp_param_qgc.mp_tar_ned_n, &(target_ned.northx));
   	param_get(pointers_mp_param_qgc.mp_tar_ned_e, &(target_ned.easty));

   	param_get(pointers_mp_param_qgc.mp_cur_fr_id, &(cur_mission.fr_id));
   	param_get(pointers_mp_param_qgc.mp_cur_mi_id, &(cur_mission.id));
   	param_get(pointers_mp_param_qgc.mp_cur_mi_type, &(cur_mission.type));

   	param_get(pointers_mp_param_qgc.mp_ob_num, &ob_num);

   	target.alt = altitude;

	#if LDEBUG_USEMISSION == 0
		nav_set_target(t_num, target);
		nav_set_target_ned(t_num, target_ned);	//NOTE: Because of this the target is always set in NED-Coordinates
		nav_set_obstacle(o_num,obstacle);
		nav_set_obstacle_ned(o_num,obstacle_ned);	//NOTE: Because of this the obstacle is always set in NED-Coordinates
	#endif

	PointE7 start[2];
	param_get(pointers_mp_param_qgc.mp_start1_lat, &(start[0].lat));
	param_get(pointers_mp_param_qgc.mp_start1_lon, &(start[0].lon));
	param_get(pointers_mp_param_qgc.mp_start2_lat, &(start[1].lat));
	param_get(pointers_mp_param_qgc.mp_start2_lon, &(start[1].lon));
	start[0].alt = altitude;
	start[1].alt = altitude;

	nav_set_startline(start[0],start[1]);

};

// read parameters from SD card
void mp_get_params_SD(char file_path[], frame frames[], int fr_count, mission missions[], int mi_count) {
	// open file
	int fd = open(file_path, O_RDONLY);
	if (fd < 0 | fd == NULL) {
		printf("unable to open file %s \n", file_path);
	} else {
		printf("parameter file found at %s\n", file_path);
		char *line;
		int sc;
		do {
			line = read_line(fd, MAX_CHAR_LINE);
			if (strstr(line, "<frame>") != NULL) {
				sc = read_fr(fd, frames[fr_count]);
				fr_count++;
				if (sc != 0) printf("successfully read frame");
			}
			if (strstr(line, "<mission>") != NULL) {
				sc = read_mi(fd, missions[mi_count]);
				mi_count++;
				if (sc != 0) printf("successfully read mission");
			}
		} while (line != NULL);
	}
	close(fd);
};

// get parameters from serial port
void mp_get_params_SP(){

};

// send paramaters to serial port
void mp_send_params_SP(char buffer[]){

};

static int read_wp(int fd, waypoint wp) {
	char *line, *pch;
	line = read_line(fd, MAX_CHAR_LINE);
	if (line == NULL) {
		printf("error reading waypoint");
		return 0;
	}
	pch = strtok(line, " ");
	pch = strtok(NULL, " ");
	wp.latitude = atof(pch);
	line = read_line(fd, MAX_CHAR_LINE);
	if (line == NULL) {
		printf("error reading waypoint");
		return 0;
	}
	pch = strtok(line, " ");
	pch = strtok(NULL, " ");
	wp.longitude = atof(pch);
	line = read_line(fd, MAX_CHAR_LINE);
	if (line == NULL) {
		printf("error reading waypoint");
		return 0;
	}
	if (strstr(line, "<end_waypoint>") != NULL) {
		// success
		return 1;
	}
	return 0;
};

static int read_ob(int fd, obstacle ob) {
	char *line, *pch;
	int sc;
	line = read_line(fd, MAX_CHAR_LINE);
	if (line == NULL) {
		printf("error reading obstacle");
		return 0;
	}
	if (strstr(line, "<waypoint>") != NULL) {
		sc = read_wp(fd, ob.center);
	}
	line = read_line(fd, MAX_CHAR_LINE);
	if (line == NULL) {
		printf("error reading obstacle");
		return 0;
	}
	pch = strtok(line, " ");
	pch = strtok(NULL, " ");
	ob.radius = atof(pch);
	line = read_line(fd, MAX_CHAR_LINE);
	if (line == NULL) {
		printf("error reading obstacle");
		return 0;
	}
	if (strstr(line, "<end_obstacle>") != NULL & sc != 0) {
		return 1;
	}
	return 0;
};

static int read_bu(int fd, buoy bu) {
	char *line, *pch;
	int sc;
	line = read_line(fd, MAX_CHAR_LINE);
	if (line == NULL) {
		printf("error reading buoy");
		return 0;
	}
	if (strstr(line, "<obstacle>") != NULL) {
		sc = read_ob(fd, bu.body);
	}
	line = read_line(fd, MAX_CHAR_LINE);
	if (line == NULL) {
		printf("error reading buoy");
		return 0;
	}
	pch = strtok(line, " ");
	pch = strtok(NULL, " ");
	bu.rotation = atoi(pch);
	line = read_line(fd, MAX_CHAR_LINE);
	if (line == NULL) {
		printf("error reading buoy");
		return 0;
	}
	if (strstr(line, "<end_buoy>") != NULL & sc != 0) {
		return 1;
	}
	return 0;
};

static int read_fr(int fd, frame fr) {
	char *line, *pch;
	int sc;
	line = read_line(fd, MAX_CHAR_LINE);
	if (line == NULL) {
		printf("error reading frame");
		return 0;
	}
	pch = strtok(line, " ");
	pch = strtok(NULL, " ");
	fr.name = pch;
	line = read_line(fd, MAX_CHAR_LINE);
	if (line == NULL) {
		printf("error reading frame");
		return 0;
	}
	pch = strtok(line, " ");
	pch = strtok(NULL, " ");
	fr.id = atoi(pch);
	int bu_count = 0;
	do {
		line = read_line(fd, MAX_CHAR_LINE);
		if (line == NULL) {
			printf("error reading frame");
			return 0;
		}
		sc = read_bu(fd, fr.buoys[bu_count]);
		bu_count++;
	} while (strstr(line, "<buoy>") != NULL & bu_count < MAX_ELEM);
	line = read_line(fd, MAX_CHAR_LINE);
	if (line == NULL) {
		printf("error reading frame");
		return 0;
	}
	if (strstr(line, "<end_frame>") != NULL & sc != 0) {
		return 1;
	}
	return 0;
};

static int read_mi(int fd, mission mi) {
	char *line, *pch;
	int sc;
	line = read_line(fd, MAX_CHAR_LINE);
	if (line == NULL) {
		printf("error reading mission");
		return 0;
	}
	pch = strtok(line, " ");
	pch = strtok(NULL, " ");
	mi.name = pch;
	line = read_line(fd, MAX_CHAR_LINE);
	if (line == NULL) {
		printf("error reading mission");
		return 0;
	}
	pch = strtok(line, " ");
	pch = strtok(NULL, " ");
	mi.id = atoi(pch);
	line = read_line(fd, MAX_CHAR_LINE);
	if (line == NULL) {
		printf("error reading mission");
		return 0;
	}
	pch = strtok(line, " ");
	pch = strtok(NULL, " ");
	mi.type = atoi(pch);
	line = read_line(fd, MAX_CHAR_LINE);
	if (line == NULL) {
		printf("error reading mission");
		return 0;
	}
	pch = strtok(line, " ");
	pch = strtok(NULL, " ");
	mi.fr_id = atoi(pch);
	int wp_count = 0, ob_count = 0;
	do {
		line = read_line(fd, MAX_CHAR_LINE);
		if (line == NULL) {
			printf("error reading mission");
			return 0;
		}
		sc = read_wp(fd, mi.waypoints[wp_count]);
		wp_count++;
	} while (strstr(line, "<waypoint>") != NULL & wp_count < MAX_ELEM);
	do {
		line = read_line(fd, MAX_CHAR_LINE);
		if (line == NULL) {
			printf("error reading mission");
			return 0;
		}
		sc = read_ob(fd, mi.obstacles[ob_count]);
		ob_count++;
	} while (strstr(line, "<obstacle>") != NULL & ob_count < MAX_ELEM);
	line = read_line(fd, MAX_CHAR_LINE);
	if (line == NULL) {
		printf("error reading mission");
		return 0;
	}
	if (strstr(line, "<end_mission>") != NULL & sc != 0) {
		return 1;
	}
	return 0;
};

// read line from opened file descriptor
static char* read_line(int fd, int maxChar) {
	// printf("START readline\n");
	int ch, num_read = 0, size = 1;
	int *line, *readbuffer;
	line = (char*) calloc (maxChar+1, sizeof(char));
	readbuffer = (char*) calloc (size+1, sizeof(char));
	do {
		ch = read(fd, readbuffer, size); // <0: error, =0: EOF, >0: success
		num_read += size;
		if (ch < 0) {
			perror("error when reading: ");
			return NULL;
		}
		if (ch == 0) {
			return NULL;
		}
		strcat(line, readbuffer);
		// printf("readbuffer: %s\n", readbuffer);
	} while (num_read <= maxChar & strchr(readbuffer, '\n') == NULL);
	realloc(line, num_read+1);
	// printf("END readline\n");
	// printf("line: %s", line);
	return line;
};

void disp_fr(frame fr) {
	printf("---frame---\n");
	printf("name: %s\n", fr.name);
	printf("id: %d\n", fr.id);
	int bu_count = 0;
	while (1) {
		if (fr.buoys[bu_count].body.center.latitude != 0 & bu_count < MAX_ELEM) {
			printf("buoy%d, lat: %.5f, lon: %.5f, rad: %.5f, rot: %d\n", bu_count, fr.buoys[bu_count].body.center.latitude, fr.buoys[bu_count].body.center.longitude, fr.buoys[bu_count].body.radius, fr.buoys[bu_count].rotation);
			bu_count++;
		} else {
			break;
		}
	}
	printf("---end frame---\n");
};

void disp_mi(mission mi) {
	printf("---mission---\n");
	printf("name: %s\n", mi.name);
	printf("id: %d\n", mi.id);
	printf("type: %d\n", mi.type);
	printf("frame id: %d\n", mi.fr_id);
	int wp_count = 0, ob_count = 0;
	while (1) {
		if (mi.waypoints[wp_count].latitude != 0 & wp_count < MAX_ELEM) {
			printf("waypoint%d, lat: %.5f, lon: %.5f\n", wp_count, mi.waypoints[wp_count].latitude, mi.waypoints[wp_count].longitude);
			wp_count++;
		} else {
			break;
		}
	}
	while (1) {
		if (mi.obstacles[ob_count].center.latitude != 0 & ob_count < MAX_ELEM) {
			printf("obstacle%d, lat: %.5f, lon: %.5f, rad: %.5f\n", ob_count, mi.obstacles[ob_count].center.latitude, mi.obstacles[ob_count].center.longitude, mi.obstacles[ob_count].radius);
			ob_count++;
		} else {
			break;
		}
	}
	printf("---end mission---\n");
};
