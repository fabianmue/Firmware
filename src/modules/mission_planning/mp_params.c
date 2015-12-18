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

#include "mp_mission.h"
#include "../path_planning/pp_navigator.h"

/***********************************************************************************/
/*****  V A R I A B L E S  *********************************************************/
/***********************************************************************************/

bool sd_read = false;

char file_path[] = "/fs/microsd/params.txt";
char buffer_pa[50];

int temp = 1;

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
	PointE7 target, start[2];
	NEDpoint target_ned;
	mission cur_mission;
	uint8_t ob_num;
	int32_t altitude;

	param_get(pointers_mp_param_qgc.mp_data_src, &data_src);

	if ((data_src == 1 | SD_DEBUG == 1) & sd_read == false) {

		// read parameters from SD card
		mp_get_params_SD(file_path);
		sd_read = true;
	}

   	param_get(pointers_mp_param_qgc.mp_tar_lat, &(target.lat));
   	param_get(pointers_mp_param_qgc.mp_tar_lon, &(target.lon));
   	target.alt = altitude;
   	param_get(pointers_mp_param_qgc.mp_tar_ned_n, &(target_ned.northx));
   	param_get(pointers_mp_param_qgc.mp_tar_ned_e, &(target_ned.easty));

   	param_get(pointers_mp_param_qgc.mp_cur_fr_id, &(cur_mission.fr_id));
   	param_get(pointers_mp_param_qgc.mp_cur_mi_id, &(cur_mission.id));
   	param_get(pointers_mp_param_qgc.mp_cur_mi_type, &(cur_mission.type));

   	param_get(pointers_mp_param_qgc.mp_ob_num, &ob_num);

	#if LDEBUG_USEMISSION == 0
		// nav_set_target(t_num, target);
		nav_set_target_ned(t_num, target_ned);			//NOTE: Because of this the target is always set in NED-Coordinates
		// nav_set_obstacle(o_num, obstacle);
		// nav_set_obstacle_ned(o_num, obstacle_ned);	//NOTE: Because of this the obstacle is always set in NED-Coordinates
	#endif

	param_get(pointers_mp_param_qgc.mp_start1_lat, &(start[0].lat));
	param_get(pointers_mp_param_qgc.mp_start1_lon, &(start[0].lon));
	param_get(pointers_mp_param_qgc.mp_start2_lat, &(start[1].lat));
	param_get(pointers_mp_param_qgc.mp_start2_lon, &(start[1].lon));
};

// read parameters from SD card
void mp_get_params_SD(char file_path[]) {
	// open file
	int fd = open(file_path, O_RDONLY);
	if (fd < 0 | fd == NULL) {
		sprintf(buffer_pa, "unable to open file %s \n", file_path);
		printf(buffer_pa);
		mp_send_log_info(buffer_pa);
	} else {
		sprintf(buffer_pa, "parameter file found at %s\n", file_path);
		printf(buffer_pa);
		mp_send_log_info(buffer_pa);
		char *line;
		do {
			line = read_line(fd, MAX_CHAR_LINE);
			if (strstr(line, "<frame>") != NULL) {

				// read frame from file descriptor
				frame *rd_frame;
				rd_frame = read_fr(fd);
				if (rd_frame == NULL) {
					sprintf(buffer_pa, "error reading frame\n");
					printf(buffer_pa);
					mp_send_log_info(buffer_pa);
				} else {
					sprintf(buffer_pa, "successfully read frame\n");	// advertise
					printf(buffer_pa);
					mp_send_log_info(buffer_pa);
					mp_add_fr_to_queue(rd_frame);					// add frame to list
					// disp_fr(rd_frame);							// for test purpose only
				}
			}
			if (strstr(line, "<mission>") != NULL) {

				// read mission from file descriptor
				mission *rd_mission;
				rd_mission = read_mi(fd);
				if (rd_mission == NULL) {
					sprintf(buffer_pa, "error reading mission\n");
					printf(buffer_pa);
					mp_send_log_info(buffer_pa);
				} else {
					sprintf(buffer_pa, "successfully read mission\n");	// advertise
					printf(buffer_pa);
					mp_send_log_info(buffer_pa);
					mp_add_mi_to_queue(rd_mission);					// add mission to list
					// disp_mi(rd_mission);							// for test purpose only
				}
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

static waypoint* read_wp(int fd) {
	char *line, *pch;

	// allocate memory
	waypoint *wp;
	wp = (waypoint*) calloc (1, sizeof(waypoint));

	// latitude
	line = read_line(fd, MAX_CHAR_LINE);
	if (line == NULL) {
		sprintf(buffer_pa, "error reading waypoint");
		printf(buffer_pa);
		mp_send_log_info(buffer_pa);
		return NULL;
	}
	pch = strtok(line, " ");
	pch = strtok(NULL, " ");
	wp->latitude = atof(pch);

	// longitude
	line = read_line(fd, MAX_CHAR_LINE);
	if (line == NULL) {
		sprintf(buffer_pa, "error reading waypoint");
		printf(buffer_pa);
		mp_send_log_info(buffer_pa);
		return NULL;
	}
	pch = strtok(line, " ");
	pch = strtok(NULL, " ");
	wp->longitude = atof(pch);

	// end marker
	line = read_line(fd, MAX_CHAR_LINE);
	if (line == NULL) {
		sprintf(buffer_pa, "error reading waypoint");
		printf(buffer_pa);
		mp_send_log_info(buffer_pa);
		return NULL;
	}
	if (strstr(line, "<end_waypoint>") == NULL) {
		return NULL;
	}
	return wp;
};

static obstacle* read_ob(int fd) {
	char *line, *pch;

	// allocate memory
	obstacle *ob;
	ob = (obstacle*) calloc (1, sizeof(obstacle));

	// waypoint
	line = read_line(fd, MAX_CHAR_LINE);
	if (line == NULL) {
		sprintf(buffer_pa, "error reading obstacle");
		printf(buffer_pa);
		mp_send_log_info(buffer_pa);
		return NULL;
	}
	if (strstr(line, "<waypoint>") != NULL) {
		waypoint *wp;
		wp = read_wp(fd);
		if (wp == NULL) {
			return NULL;
		} else {
			ob->center = *wp;
		}
	}

	// radius
	line = read_line(fd, MAX_CHAR_LINE);
	if (line == NULL) {
		sprintf(buffer_pa, "error reading obstacle");
		printf(buffer_pa);
		mp_send_log_info(buffer_pa);
		return NULL;
	}
	pch = strtok(line, " ");
	pch = strtok(NULL, " ");
	ob->radius = atof(pch);

	// end marker
	line = read_line(fd, MAX_CHAR_LINE);
	if (line == NULL) {
		sprintf(buffer_pa, "error reading obstacle");
		printf(buffer_pa);
		mp_send_log_info(buffer_pa);
		return NULL;
	}
	if (strstr(line, "<end_obstacle>") == NULL) {
		return NULL;
	}
	return ob;
}

static buoy* read_bu(int fd) {
	char *line, *pch;

	// allocate memory
	buoy *bu;
	bu = (buoy*) calloc (1, sizeof(buoy));

	// obstacle
	line = read_line(fd, MAX_CHAR_LINE);
	if (line == NULL) {
		sprintf(buffer_pa, "error reading buoy");
		printf(buffer_pa);
		mp_send_log_info(buffer_pa);
		return NULL;
	}
	if (strstr(line, "<obstacle>") != NULL) {
		obstacle *ob;
		ob = read_ob(fd);
		if (ob == NULL) {
			return NULL;
		} else {
			bu->body = *ob;
		}
	}

	// rotation
	line = read_line(fd, MAX_CHAR_LINE);
	if (line == NULL) {
		sprintf(buffer_pa, "error reading buoy");
		printf(buffer_pa);
		mp_send_log_info(buffer_pa);
		return NULL;
	}
	pch = strtok(line, " ");
	pch = strtok(NULL, " ");
	bu->rotation = atoi(pch);

	// end marker
	line = read_line(fd, MAX_CHAR_LINE);
	if (line == NULL) {
		sprintf(buffer_pa, "error reading buoy");
		printf(buffer_pa);
		mp_send_log_info(buffer_pa);
		return NULL;
	}
	if (strstr(line, "<end_buoy>") == NULL) {
		return NULL;
	}
	return bu;
};

static frame* read_fr(int fd) {
	char *line, *pch;

	// allocate memory
	frame *fr;
	fr = (frame*) calloc (1, sizeof(frame));

	// name
	line = read_line(fd, MAX_CHAR_LINE);
	if (line == NULL) {
		sprintf(buffer_pa, "error reading frame");
		printf(buffer_pa);
		mp_send_log_info(buffer_pa);
		return NULL;
	}
	pch = strtok(line, " ");
	pch = strtok(NULL, " ");
	fr->name = pch;

	// id
	line = read_line(fd, MAX_CHAR_LINE);
	if (line == NULL) {
		sprintf(buffer_pa, "error reading frame");
		printf(buffer_pa);
		mp_send_log_info(buffer_pa);
		return NULL;
	}
	pch = strtok(line, " ");
	pch = strtok(NULL, " ");
	fr->id = atoi(pch);

	// buoys
	int bu_count = 0;
	do {
		line = read_line(fd, MAX_CHAR_LINE);
		if (line == NULL) {
			sprintf(buffer_pa, "error reading frame");
			printf(buffer_pa);
			mp_send_log_info(buffer_pa);
			return NULL;
		}
		if (strstr(line, "<end_frame>") != NULL) {
			return fr;
		}
		if (strstr(line, "<buoy>") != NULL) {
			if (bu_count < MAX_NUM_BU) {
				buoy *bu;
				bu = read_bu(fd);
				if (bu == NULL) {
					return NULL;
				} else {
					fr->buoys[bu_count] = *bu;
					bu_count++;
				}
			} else {
				sprintf(buffer_pa, "maximum number of buoys in frame reached\n");
				printf(buffer_pa);
				mp_send_log_info(buffer_pa);
				read_bu(fd);
			}
		}
	} while (true);
	return fr;
};

static mission* read_mi(int fd) {
	char *line, *pch;

	// allocate memory
	mission *mi;
	mi = (mission*) calloc (1, sizeof(mission));

	// name
	line = read_line(fd, MAX_CHAR_LINE);
	if (line == NULL) {
		sprintf(buffer_pa, "error reading mission");
		printf(buffer_pa);
		mp_send_log_info(buffer_pa);
		return NULL;
	}
	pch = strtok(line, " ");
	pch = strtok(NULL, " ");
	mi->name = pch;

	// id
	line = read_line(fd, MAX_CHAR_LINE);
	if (line == NULL) {
		sprintf(buffer_pa, "error reading mission");
		printf(buffer_pa);
		mp_send_log_info(buffer_pa);
		return NULL;
	}
	pch = strtok(line, " ");
	pch = strtok(NULL, " ");
	mi->id = atoi(pch);

	// type
	line = read_line(fd, MAX_CHAR_LINE);
	if (line == NULL) {
		sprintf(buffer_pa, "error reading mission");
		printf(buffer_pa);
		mp_send_log_info(buffer_pa);
		return NULL;
	}
	pch = strtok(line, " ");
	pch = strtok(NULL, " ");
	mi->type = atoi(pch);

	// frame id
	line = read_line(fd, MAX_CHAR_LINE);
	if (line == NULL) {
		sprintf(buffer_pa, "error reading mission");
		printf(buffer_pa);
		mp_send_log_info(buffer_pa);
		return NULL;
	}
	pch = strtok(line, " ");
	pch = strtok(NULL, " ");
	mi->fr_id = atoi(pch);

	// waypoints / obstacles
	int wp_count = 0, ob_count = 0;
	do {
		line = read_line(fd, MAX_CHAR_LINE);
		if (line == NULL) {
			sprintf(buffer_pa, "error reading mission");
			printf(buffer_pa);
			mp_send_log_info(buffer_pa);
			return NULL;
		}
		if (strstr(line, "<end_mission>") != NULL) {
			return mi;
		}
		if (strstr(line, "<waypoint>") != NULL) {
			if (wp_count < MAX_NUM_WP) {
				waypoint *wp;
				wp = read_wp(fd);
				if (wp == NULL) {
					return NULL;
				} else {
					mi->waypoints[wp_count] = *wp;
					wp_count++;
				}
			} else {
				sprintf(buffer_pa, "maximum number of waypoints in mission reached\n");
				printf(buffer_pa);
				mp_send_log_info(buffer_pa);
				read_bu(fd);
			}
		}
		if (strstr(line, "<obstacle>") != NULL) {
			if (ob_count < MAX_NUM_OB) {
				obstacle *ob;
				ob = read_ob(fd);
				if (ob == NULL) {
					return NULL;
				} else {
					mi->obstacles[ob_count] = *ob;
					ob_count++;
				}
			} else {
				sprintf(buffer_pa, "maximum number of obstacles in mission reached\n");
				printf(buffer_pa);
				mp_send_log_info(buffer_pa);
				read_bu(fd);
			}
		}
	} while (true);
	return mi;
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
	} while (num_read <= maxChar & strchr(readbuffer, '\n') == NULL);
	realloc(line, num_read+1);
	return line;
};

void disp_fr(frame *fr) {
	printf("---frame---\n");
	printf("name: %s\n", fr->name);
	printf("id: %d\n", fr->id);
	int bu_count = 0;
	while (1) {
		if (fr->buoys[bu_count].body.center.latitude != 0 & bu_count < MAX_NUM_BU) {
			printf("buoy%d, lat: %.5f, lon: %.5f, rad: %.5f, rot: %d\n", bu_count, fr->buoys[bu_count].body.center.latitude, fr->buoys[bu_count].body.center.longitude, fr->buoys[bu_count].body.radius, fr->buoys[bu_count].rotation);
			bu_count++;
		} else {
			break;
		}
	}
	printf("---end frame---\n");
};

void disp_mi(mission *mi) {
	printf("---mission---\n");
	printf("name: %s\n", mi->name);
	printf("id: %d\n", mi->id);
	printf("type: %d\n", mi->type);
	printf("frame id: %d\n", mi->fr_id);
	int wp_count = 0, ob_count = 0;
	while (1) {
		if (mi->waypoints[wp_count].latitude != 0 & wp_count < MAX_NUM_WP) {
			printf("waypoint%d, lat: %.5f, lon: %.5f\n", wp_count, mi->waypoints[wp_count].latitude, mi->waypoints[wp_count].longitude);
			wp_count++;
		} else {
			break;
		}
	}
	while (1) {
		if (mi->obstacles[ob_count].center.latitude != 0 & ob_count < MAX_NUM_OB) {
			printf("obstacle%d, lat: %.5f, lon: %.5f, rad: %.5f\n", ob_count, mi->obstacles[ob_count].center.latitude, mi->obstacles[ob_count].center.longitude, mi->obstacles[ob_count].radius);
			ob_count++;
		} else {
			break;
		}
	}
	printf("---end mission---\n");
};
