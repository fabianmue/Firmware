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
#include <string.h>

#include <nuttx/config.h>
#include <fcntl.h>

#include "mp_params.h"
#include "mp_mission.h"

/***********************************************************************************/
/*****  V A R I A B L E S  *********************************************************/
/***********************************************************************************/

int sd_read = 0;

int mi_select_prev = 0;

char file_path[] = "/fs/microsd/mission_data/mission.txt";
char buffer_pa[50];

int32_t data_src = 0, mi_select = 0;

/**
 *  mission planner QGC variables
 */
PARAM_DEFINE_INT32(MP_DATA_SRC, 0);		// data source, 0 = telemetry, 1 = SD card
PARAM_DEFINE_INT32(MP_MI_SELECT, 0);	// mission selected

static struct pointers_mp_param_qgc_s {

	param_t mp_data_src;
	param_t mp_mi_select;

} pointers_mp_param_qgc;

/***********************************************************************************/
/*****  F U N C T I O N   D E F I N I T I O N S  ***********************************/
/***********************************************************************************/

void mp_param_init(void) {

	memset(&pointers_mp_param_qgc, 0, sizeof(pointers_mp_param_qgc));

    pointers_mp_param_qgc.mp_data_src = param_find("MP_DATA_SRC");
    pointers_mp_param_qgc.mp_mi_select = param_find("MP_MI_SELECT");

    mp_param_update();
};

void mp_param_update(void) {

	param_get(pointers_mp_param_qgc.mp_data_src, &data_src);
	param_get(pointers_mp_param_qgc.mp_mi_select, &mi_select);

	if (data_src == 1 & sd_read == 0) {

		// read parameters from SD card
		mp_get_params_SD(file_path);
		mp_cb_sd_read();
		sd_read = 1;
	}

	if (mi_select != mi_select_prev) {

		// new mission selected, call the mission handler
		mi_select_prev = mi_select;
		// mp_mi_handler(mi_select);
	}
}

// read parameters from SD card
void mp_get_params_SD(char *file_path) {
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
		mp_reset_lists();
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
					mp_add_fr_to_list(rd_frame);					// add frame to list
					// disp_fr(rd_frame);							// debug
				}
				free(rd_frame);
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
					mp_add_mi_to_list(rd_mission);					// add mission to list
					// disp_mi(rd_mission);							// debug
				}
				free(rd_mission);
			}
		} while (line != NULL);
		free(line);
	}
	close(fd);
};

// get parameters from serial port
void mp_get_params_SP(){

};

// send paramaters to serial port
void mp_send_params_SP(char buffer[]){

};

waypoint* read_wp(int fd) {
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
	free(line);
	return wp;
};

obstacle* read_ob(int fd) {
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
		free(wp);
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
	free(line);
	return ob;
}

buoy* read_bu(int fd) {
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
		free(ob);
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
	free(line);
	return bu;
};

frame* read_fr(int fd) {
	char *line, *pch;

	// allocate memory
	frame *fr;
	fr = (frame*) calloc (1, sizeof(frame));
	fr->buoy_count = 0;

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
			buoy *bu;
			bu = read_bu(fd);
			if (fr->buoy_count < MAX_NUM_BU) {
				if (bu == NULL) {
					return NULL;
				} else {
					fr->buoys[fr->buoy_count] = *bu;
					fr->buoy_count++;
				}

			} else {
				sprintf(buffer_pa, "maximum number of buoys in frame reached\n");
				printf(buffer_pa);
				mp_send_log_info(buffer_pa);
			}
			free(bu);
		}
	} while (true);
	free(line);
	return fr;
};

mission* read_mi(int fd) {
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
			waypoint *wp;
			wp = read_wp(fd);
			if (mi->waypoint_count < MAX_NUM_WP) {
				if (wp == NULL) {
					return NULL;
				} else {
					mi->waypoints[mi->waypoint_count] = *wp;
					mi->waypoint_count++;
				}
			} else {
				sprintf(buffer_pa, "maximum number of waypoints in mission reached\n");
				printf(buffer_pa);
				mp_send_log_info(buffer_pa);
			}
			free(wp);
		}
		if (strstr(line, "<obstacle>") != NULL) {
			obstacle *ob;
			ob = read_ob(fd);
			if (mi->obstacle_count < MAX_NUM_OB) {
				if (ob == NULL) {
					return NULL;
				} else {
					mi->obstacles[mi->obstacle_count] = *ob;
					mi->obstacle_count++;
				}
			} else {
				sprintf(buffer_pa, "maximum number of obstacles in mission reached\n");
				printf(buffer_pa);
				mp_send_log_info(buffer_pa);
			}
			free(ob);
		}
	} while (true);
	free(line);
	return mi;
};

// read line from opened file descriptor
char* read_line(int fd, int maxChar) {
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
	free(readbuffer);
	realloc(line, num_read+1);
	return line;
};

void disp_fr(frame *fr) {
	printf("---frame---\n");
	printf("name: %s\n", fr->name);
	printf("id: %d\n", fr->id);
	for (int i = 0; i < fr->buoy_count; i++) {
		printf("buoy %d, lat: %.5f, lon: %.5f, rad: %.5f, rot: %d\n", i, fr->buoys[i].body.center.latitude, fr->buoys[i].body.center.longitude, fr->buoys[i].body.radius, fr->buoys[i].rotation);
	}
	printf("---end frame---\n");
};

void disp_mi(mission *mi) {
	printf("---mission---\n");
	printf("name: %s\n", mi->name);
	printf("id: %d\n", mi->id);
	printf("type: %d\n", mi->type);
	printf("frame id: %d\n", mi->fr_id);
	for (int i = 0; i < mi->waypoint_count; i++) {
		printf("waypoint %d, lat: %.5f, lon: %.5f\n", i, mi->waypoints[i].latitude, mi->waypoints[i].longitude);
	}
	for (int j = 0; j < mi->obstacle_count; j++) {
		printf("obstacle %d, lat: %.5f, lon: %.5f, rad: %.5f\n", j, mi->obstacles[j].center.latitude, mi->obstacles[j].center.longitude, mi->obstacles[j].radius);
	}
	printf("---end mission---\n");
};
