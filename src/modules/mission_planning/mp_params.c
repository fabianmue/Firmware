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

int mi_select_prev = 0;

char file_path[] = "/fs/microsd/mission_data/mission_data.txt";
char buffer_pa[60];

// file variables
int32_t data_src = 0, mi_select = 0, sd_read = 0;
int32_t acc = 0;

// variables received from QGC
PARAM_DEFINE_INT32(MP_DATA_SRC, 0);		// data source, 0 = telemetry, 1 = SD card
PARAM_DEFINE_INT32(MP_MI_SELECT, 0);	// mission selected

// variables sent to QGC
PARAM_DEFINE_INT32(MP_SD_READ, 0);
PARAM_DEFINE_INT32(MP_TF_DONE, 0);

// test variables
PARAM_DEFINE_INT32(MP_TEST, 0);
PARAM_DEFINE_INT32(MP_TEST2, 7);

static struct pointers_mp_param_qgc_s {

	// from QGC
	param_t mp_data_src;
	param_t mp_mi_select;

	// to QGC
	param_t mp_sd_read;
	param_t mp_tf_done;

	param_t mp_test;

} pointers_mp_param_qgc;

/***********************************************************************************/
/*****  F U N C T I O N   D E F I N I T I O N S  ***********************************/
/***********************************************************************************/

void mp_param_QGC_init(void) {

	// from QGC
    pointers_mp_param_qgc.mp_data_src = param_find("MP_DATA_SRC");
    pointers_mp_param_qgc.mp_mi_select = param_find("MP_MI_SELECT");
    mp_param_QGC_get();

    // to QGC
    pointers_mp_param_qgc.mp_sd_read = param_find("MP_SD_READ");
    pointers_mp_param_qgc.mp_tf_done = param_find("MP_TF_DONE");

    pointers_mp_param_qgc.mp_test = param_find("MP_TEST");

};

void mp_param_QGC_get(void) {
	param_get(pointers_mp_param_qgc.mp_data_src, &data_src);
	param_get(pointers_mp_param_qgc.mp_mi_select, &mi_select);
	if ((data_src == 1 | SD_DEBUG == 1) & sd_read == 0) {

		// read parameters from SD card
		mp_get_mission_from_SD(file_path);

		// set parameter in QGC
		sd_read = 1;
		param_set(pointers_mp_param_qgc.mp_sd_read, &sd_read);
	}
}

void mp_param_QGC_set(int done) {
	param_set(pointers_mp_param_qgc.mp_tf_done, &done);
}

void mp_mission_update(int wp_ack, int ob_ack) {
	if (sd_read != 0) {
		mp_mi_handler(mi_select, wp_ack, ob_ack);
	}
}

// @brief read parameters from SD card
void mp_get_mission_from_SD(char *file_path) {
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
		mp_list_reset();
		char *line;
		do {
			line = read_line(fd, MAX_CHAR_LINE);
			if (strstr(line, "<frame>") != NULL) {

				// read frame from file descriptor
				rd_frame *rd_fr = read_frame(fd);
				if (rd_fr == NULL) {
					sprintf(buffer_pa, "error reading frame\n");
					printf(buffer_pa);
					mp_send_log_info(buffer_pa);
				} else {
					sprintf(buffer_pa, "successfully read frame\n");	// advertise
					printf(buffer_pa);
					mp_send_log_info(buffer_pa);
					mp_fr_convert(rd_fr);
				}
				free(rd_fr);
			}
			if (strstr(line, "<mission>") != NULL) {

				// read mission from file descriptor
				rd_mission_base *rd_mi_base = read_mission_base(fd);
				if (rd_mi_base == NULL) {
					sprintf(buffer_pa, "error reading mission\n");
					printf(buffer_pa);
					mp_send_log_info(buffer_pa);
				} else {
					if (rd_mi_base->type == 0) {
						rd_mission_uc *rd_mi_uc = read_mission_uc(fd, rd_mi_base);
						if (rd_mi_uc == NULL) {
							sprintf(buffer_pa, "error reading unconstrained mission\n");
							printf(buffer_pa);
							mp_send_log_info(buffer_pa);
						} else {
							sprintf(buffer_pa, "successfully read unconstrained mission\n");
							printf(buffer_pa);
							mp_send_log_info(buffer_pa);
							mp_mi_uc_convert(rd_mi_uc);
						}
						free(rd_mi_uc);
					}
					if (rd_mi_base->type == 1) {
						rd_mission_tr *rd_mi_tr = read_mission_tr(fd, rd_mi_base);
						if (rd_mi_tr == NULL) {
							sprintf(buffer_pa, "error reading triangular mission\n");
							printf(buffer_pa);
							mp_send_log_info(buffer_pa);
						} else {
							sprintf(buffer_pa, "successfully read triangular mission\n");
							printf(buffer_pa);
							mp_send_log_info(buffer_pa);
							//
						}
						free(rd_mi_tr);
					}
					if (rd_mi_base->type == 2) {
						rd_mission_sk *rd_mi_sk = read_mission_sk(fd, rd_mi_base);
						if (rd_mi_sk == NULL) {
							sprintf(buffer_pa, "error reading station keeping mission\n");
							printf(buffer_pa);
							mp_send_log_info(buffer_pa);
						} else {
							sprintf(buffer_pa, "successfully read station keeping mission\n");
							printf(buffer_pa);
							mp_send_log_info(buffer_pa);
							//
						}
						free(rd_mi_sk);
					}
					if (rd_mi_base->type == 3) {
						rd_mission_as *rd_mi_as = read_mission_as(fd, rd_mi_base);
						if (rd_mi_as == NULL) {
							sprintf(buffer_pa, "error reading area scanning mission\n");
							printf(buffer_pa);
							mp_send_log_info(buffer_pa);
						} else {
							sprintf(buffer_pa, "successfully read area scanning mission\n");
							printf(buffer_pa);
							mp_send_log_info(buffer_pa);
							//
						}
						free(rd_mi_as);
					}
				}
			}
		} while (line != NULL);
		free(line);
	}
	close(fd);
};


rd_waypoint* read_wp(int fd) {
	char *line, *pch;

	// allocate memory
	rd_waypoint *rd_wp;
	rd_wp = (rd_waypoint*) calloc (1, sizeof(rd_waypoint));

	// name
	line = read_line(fd, MAX_CHAR_LINE);
	if (line == NULL) {
		sprintf(buffer_pa, "error reading waypoint");
		printf(buffer_pa);
		mp_send_log_info(buffer_pa);
		free(rd_wp);
		free(line);
		return NULL;
	}
	pch = strtok(line, " ");
	pch = strtok(NULL, " ");
	rd_wp->name = pch;

	// latitude
	line = read_line(fd, MAX_CHAR_LINE);
	if (line == NULL) {
		sprintf(buffer_pa, "error reading waypoint");
		printf(buffer_pa);
		mp_send_log_info(buffer_pa);
		free(rd_wp);
		free(line);
		return NULL;
	}
	pch = strtok(line, " ");
	pch = strtok(NULL, " ");
	rd_wp->latitude = atof(pch);

	// longitude
	line = read_line(fd, MAX_CHAR_LINE);
	if (line == NULL) {
		sprintf(buffer_pa, "error reading waypoint");
		printf(buffer_pa);
		mp_send_log_info(buffer_pa);
		free(rd_wp);
		free(line);
		return NULL;
	}
	pch = strtok(line, " ");
	pch = strtok(NULL, " ");
	rd_wp->longitude = atof(pch);

	// end marker
	line = read_line(fd, MAX_CHAR_LINE);
	if (line == NULL) {
		sprintf(buffer_pa, "error reading waypoint");
		printf(buffer_pa);
		mp_send_log_info(buffer_pa);
		free(rd_wp);
		free(line);
		return NULL;
	}
	if (strstr(line, "<end_waypoint>") == NULL) {
		free(rd_wp);
		free(line);
		return NULL;
	}
	free(line);
	return rd_wp;
};

rd_obstacle* read_ob(int fd) {
	char *line, *pch;

	// allocate memory
	rd_obstacle *rd_ob;
	rd_ob = (rd_obstacle*) calloc (1, sizeof(rd_obstacle));

	// name
	line = read_line(fd, MAX_CHAR_LINE);
	if (line == NULL) {
		sprintf(buffer_pa, "error reading waypoint");
		printf(buffer_pa);
		mp_send_log_info(buffer_pa);
		free(rd_ob);
		free(line);
		return NULL;
	}
	pch = strtok(line, " ");
	pch = strtok(NULL, " ");
	rd_ob->name = pch;

	// waypoint
	line = read_line(fd, MAX_CHAR_LINE);
	if (line == NULL) {
		sprintf(buffer_pa, "error reading obstacle");
		printf(buffer_pa);
		mp_send_log_info(buffer_pa);
		free(rd_ob);
		free(line);
		return NULL;
	}
	if (strstr(line, "<waypoint>") != NULL) {
		rd_waypoint *rd_wp;
		rd_wp = read_wp(fd);
		if (rd_wp == NULL) {
			free(rd_wp);
			free(rd_ob);
			free(line);
			return NULL;
		} else {
			rd_ob->center = *rd_wp;
		}
		free(rd_wp);
	}

	// radius
	line = read_line(fd, MAX_CHAR_LINE);
	if (line == NULL) {
		sprintf(buffer_pa, "error reading obstacle");
		printf(buffer_pa);
		mp_send_log_info(buffer_pa);
		free(rd_ob);
		free(line);
		return NULL;
	}
	pch = strtok(line, " ");
	pch = strtok(NULL, " ");
	rd_ob->radius = atof(pch);

	// end marker
	line = read_line(fd, MAX_CHAR_LINE);
	if (line == NULL) {
		sprintf(buffer_pa, "error reading obstacle");
		printf(buffer_pa);
		mp_send_log_info(buffer_pa);
		free(rd_ob);
		free(line);
		return NULL;
	}
	if (strstr(line, "<end_obstacle>") == NULL) {
		free(rd_ob);
		free(line);
		return NULL;
	}
	free(line);
	return rd_ob;
}

rd_buoy* read_bu(int fd) {
	char *line, *pch;

	// allocate memory
	rd_buoy *bu;
	bu = (rd_buoy*) calloc (1, sizeof(rd_buoy));

	// name
	line = read_line(fd, MAX_CHAR_LINE);
	if (line == NULL) {
		sprintf(buffer_pa, "error reading waypoint");
		printf(buffer_pa);
		mp_send_log_info(buffer_pa);
		free(bu);
		free(line);
		return NULL;
	}
	pch = strtok(line, " ");
	pch = strtok(NULL, " ");
	bu->name = pch;

	// obstacle
	line = read_line(fd, MAX_CHAR_LINE);
	if (line == NULL) {
		sprintf(buffer_pa, "error reading buoy");
		printf(buffer_pa);
		mp_send_log_info(buffer_pa);
		free(bu);
		free(line);
		return NULL;
	}
	if (strstr(line, "<obstacle>") != NULL) {
		rd_obstacle *rd_ob;
		rd_ob = read_ob(fd);
		if (rd_ob == NULL) {
			free(rd_ob);
			free(bu);
			free(line);
			return NULL;
		} else {
			bu->body = *rd_ob;
		}
		free(rd_ob);
	}

	// rotation
	line = read_line(fd, MAX_CHAR_LINE);
	if (line == NULL) {
		sprintf(buffer_pa, "error reading buoy");
		printf(buffer_pa);
		mp_send_log_info(buffer_pa);
		free(bu);
		free(line);
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
		free(bu);
		free(line);
		return NULL;
	}
	if (strstr(line, "<end_buoy>") == NULL) {
		free(bu);
		free(line);
		return NULL;
	}
	free(line);
	return bu;
};

rd_frame* read_frame(int fd) {
	char *line, *pch;

	// allocate memory
	rd_frame *rd_fr;
	rd_fr = (rd_frame*) calloc (1, sizeof(rd_frame));
	rd_fr->buoy_count = 0;

	// name
	line = read_line(fd, MAX_CHAR_LINE);
	if (line == NULL) {
		sprintf(buffer_pa, "error reading frame");
		printf(buffer_pa);
		mp_send_log_info(buffer_pa);
		free(rd_fr);
		free(line);
		return NULL;
	}
	pch = strtok(line, " ");
	pch = strtok(NULL, " ");
	rd_fr->name = pch;

	// id
	line = read_line(fd, MAX_CHAR_LINE);
	if (line == NULL) {
		sprintf(buffer_pa, "error reading frame");
		printf(buffer_pa);
		mp_send_log_info(buffer_pa);
		free(rd_fr);
		free(line);
		return NULL;
	}
	pch = strtok(line, " ");
	pch = strtok(NULL, " ");
	rd_fr->id = atoi(pch);

	// buoys
	do {
		line = read_line(fd, MAX_CHAR_LINE);
		if (line == NULL) {
			sprintf(buffer_pa, "error reading frame");
			printf(buffer_pa);
			mp_send_log_info(buffer_pa);
			free(rd_fr);
			free(line);
			return NULL;
		}
		if (strstr(line, "<end_frame>") != NULL) {
			free(line);
			return rd_fr;
		}
		if (strstr(line, "<buoy>") != NULL) {
			rd_buoy *rd_bu;
			rd_bu = read_bu(fd);
			if (rd_fr->buoy_count < MAX_NUM_BU) {
				if (rd_bu == NULL) {
					free(rd_bu);
					free(rd_fr);
					free(line);
					return NULL;
				} else {
					rd_fr->buoys[rd_fr->buoy_count] = *rd_bu;
					rd_fr->buoy_count++;
				}
			} else {
				sprintf(buffer_pa, "maximum number of buoys in frame reached\n");
				printf(buffer_pa);
				mp_send_log_info(buffer_pa);
			}
			free(rd_bu);
		}
	} while (true);
	free(line);
	return rd_fr;
};

rd_mission_base* read_mission_base(int fd) {
	char *line, *pch;

	// allocate memory
	rd_mission_base *rd_mi_base;
	rd_mi_base = (rd_mission_base*) calloc (1, sizeof(rd_mission_base));

	// name
	line = read_line(fd, MAX_CHAR_LINE);
	if (line == NULL) {
		sprintf(buffer_pa, "error reading mission");
		printf(buffer_pa);
		mp_send_log_info(buffer_pa);
		free(rd_mi_base);
		free(line);
		return NULL;
	}
	pch = strtok(line, " ");
	pch = strtok(NULL, " ");
	rd_mi_base->name = pch;

	// id
	line = read_line(fd, MAX_CHAR_LINE);
	if (line == NULL) {
		sprintf(buffer_pa, "error reading mission");
		printf(buffer_pa);
		mp_send_log_info(buffer_pa);
		free(rd_mi_base);
		free(line);
		return NULL;
	}
	pch = strtok(line, " ");
	pch = strtok(NULL, " ");
	rd_mi_base->id = atoi(pch);

	// type
	line = read_line(fd, MAX_CHAR_LINE);
	if (line == NULL) {
		sprintf(buffer_pa, "error reading mission");
		printf(buffer_pa);
		mp_send_log_info(buffer_pa);
		free(rd_mi_base);
		free(line);
		return NULL;
	}
	pch = strtok(line, " ");
	pch = strtok(NULL, " ");
	rd_mi_base->type = atoi(pch);

	// frame id
	line = read_line(fd, MAX_CHAR_LINE);
	if (line == NULL) {
		sprintf(buffer_pa, "error reading mission");
		printf(buffer_pa);
		mp_send_log_info(buffer_pa);
		free(rd_mi_base);
		free(line);
		return NULL;
	}
	pch = strtok(line, " ");
	pch = strtok(NULL, " ");
	rd_mi_base->fr_id = atoi(pch);

	// free memory
	free(line);
	return rd_mi_base;
};

rd_mission_uc* read_mission_uc(int fd, rd_mission_base *rd_mi_base) {
	char *line;

	// allocate memory
	rd_mission_uc *rd_mi_uc;
	rd_mi_uc = (rd_mission_uc*) calloc (1, sizeof(rd_mission_uc));

	// assign mission base
	rd_mi_uc->mi_base = *rd_mi_base;
	free(rd_mi_base);

	// waypoints / obstacles
	rd_waypoint *rd_wp;
	rd_obstacle *rd_ob;
	do {
		line = read_line(fd, MAX_CHAR_LINE);
		if (line == NULL) {
			sprintf(buffer_pa, "error reading unconstrained mission");
			printf(buffer_pa);
			mp_send_log_info(buffer_pa);
			free(rd_mi_uc);
			free(line);
			return NULL;
		}
		if (strstr(line, "<end_mission>") != NULL) {
			free(line);
			return rd_mi_uc;
		}
		if (strstr(line, "<waypoint>") != NULL) {
			rd_wp = read_wp(fd);
			if (rd_mi_uc->waypoint_count < MAX_NUM_WP) {
				if (rd_wp == NULL) {
					free(rd_wp);
					free(rd_mi_uc);
					free(line);
					return NULL;
				} else {
					rd_mi_uc->waypoints[rd_mi_uc->waypoint_count] = *rd_wp;
					rd_mi_uc->waypoint_count++;
				}
			} else {
				sprintf(buffer_pa, "maximum number of waypoints in unconstrained mission reached\n");
				printf(buffer_pa);
				mp_send_log_info(buffer_pa);
			}
			free(rd_wp);
		}
		if (strstr(line, "<obstacle>") != NULL) {
			rd_ob = read_ob(fd);
			if (rd_mi_uc->obstacle_count < MAX_NUM_OB) {
				if (rd_ob == NULL) {
					free(rd_ob);
					free(rd_mi_uc);
					free(line);
					return NULL;
				} else {
					rd_mi_uc->obstacles[rd_mi_uc->obstacle_count] = *rd_ob;
					rd_mi_uc->obstacle_count++;
				}
			} else {
				sprintf(buffer_pa, "maximum number of obstacles in unconstrained mission reached\n");
				printf(buffer_pa);
				mp_send_log_info(buffer_pa);
			}
			free(rd_ob);
		}
	} while (true);
	free(line);
	return rd_mi_uc;
}

rd_mission_tr* read_mission_tr(int fd, rd_mission_base *rd_mi_base) {
	char *line;

	// allocate memory
	rd_mission_tr *rd_mi_tr;
	rd_mi_tr = (rd_mission_tr*) calloc (1, sizeof(rd_mission_tr));

	// assign mission base
	rd_mi_tr->mi_base = *rd_mi_base;
	free(rd_mi_base);

	rd_buoy *rd_bu;

	// start buoy 1
	line = read_line(fd, MAX_CHAR_LINE);
	if (line == NULL) {
		sprintf(buffer_pa, "error reading triangular mission");
		printf(buffer_pa);
		mp_send_log_info(buffer_pa);
		free(rd_bu);
		free(rd_mi_tr);
		free(line);
		return NULL;
	}
	if (strstr(line, "<buoy>") != NULL) {
		rd_bu = read_bu(fd);
		if (rd_bu == NULL) {
			free(rd_bu);
			free(rd_mi_tr);
			free(line);
			return NULL;
		} else {
			rd_mi_tr->startB1 = *rd_bu;
		}
	} else {
		sprintf(buffer_pa, "error reading triangular mission");
		printf(buffer_pa);
		mp_send_log_info(buffer_pa);
		free(rd_bu);
		free(rd_mi_tr);
		free(line);
		return NULL;
	}

	// start buoy 2
	line = read_line(fd, MAX_CHAR_LINE);
	if (line == NULL) {
		sprintf(buffer_pa, "error reading triangular mission");
		printf(buffer_pa);
		mp_send_log_info(buffer_pa);
		free(rd_bu);
		free(rd_mi_tr);
		free(line);
		return NULL;
	}
	if (strstr(line, "<buoy>") != NULL) {
		rd_bu = read_bu(fd);
		if (rd_bu == NULL) {
			free(rd_bu);
			free(rd_mi_tr);
			free(line);
			return NULL;
		} else {
			rd_mi_tr->startB2 = *rd_bu;
		}
	} else {
		sprintf(buffer_pa, "error reading triangular mission");
		printf(buffer_pa);
		mp_send_log_info(buffer_pa);
		free(rd_bu);
		free(rd_mi_tr);
		free(line);
		return NULL;
	}

	// course buoy 1
	line = read_line(fd, MAX_CHAR_LINE);
	if (line == NULL) {
		sprintf(buffer_pa, "error reading triangular mission");
		printf(buffer_pa);
		mp_send_log_info(buffer_pa);
		free(rd_bu);
		free(rd_mi_tr);
		free(line);
		return NULL;
	}
	if (strstr(line, "<buoy>") != NULL) {
		rd_bu = read_bu(fd);
		if (rd_bu == NULL) {
			free(rd_bu);
			free(rd_mi_tr);
			free(line);
			return NULL;
		} else {
			rd_mi_tr->courseB1 = *rd_bu;
		}
	} else {
		sprintf(buffer_pa, "error reading triangular mission");
		printf(buffer_pa);
		mp_send_log_info(buffer_pa);
		free(rd_bu);
		free(rd_mi_tr);
		free(line);
		return NULL;
	}

	// course buoy 2
	line = read_line(fd, MAX_CHAR_LINE);
	if (line == NULL) {
		sprintf(buffer_pa, "error reading triangular mission");
		printf(buffer_pa);
		mp_send_log_info(buffer_pa);
		free(rd_bu);
		free(rd_mi_tr);
		free(line);
		return NULL;
	}
	if (strstr(line, "<buoy>") != NULL) {
		rd_bu = read_bu(fd);
		if (rd_bu == NULL) {
			free(rd_bu);
			free(rd_mi_tr);
			free(line);
			return NULL;
		} else {
			rd_mi_tr->courseB2 = *rd_bu;
		}
	} else {
		sprintf(buffer_pa, "error reading triangular mission");
		printf(buffer_pa);
		mp_send_log_info(buffer_pa);
		free(rd_bu);
		free(rd_mi_tr);
		free(line);
		return NULL;
	}

	// finish buoy 1
	line = read_line(fd, MAX_CHAR_LINE);
	if (line == NULL) {
		sprintf(buffer_pa, "error reading triangular mission");
		printf(buffer_pa);
		mp_send_log_info(buffer_pa);
		free(rd_bu);
		free(rd_mi_tr);
		free(line);
		return NULL;
	}
	if (strstr(line, "<buoy>") != NULL) {
		rd_bu = read_bu(fd);
		if (rd_bu == NULL) {
			free(rd_bu);
			free(rd_mi_tr);
			free(line);
			return NULL;
		} else {
			rd_mi_tr->finishB1 = *rd_bu;
		}
	} else {
		sprintf(buffer_pa, "error reading triangular mission");
		printf(buffer_pa);
		mp_send_log_info(buffer_pa);
		free(rd_bu);
		free(rd_mi_tr);
		free(line);
		return NULL;
	}

	// finish buoy 2
	line = read_line(fd, MAX_CHAR_LINE);
	if (line == NULL) {
		sprintf(buffer_pa, "error reading triangular mission");
		printf(buffer_pa);
		mp_send_log_info(buffer_pa);
		free(rd_bu);
		free(rd_mi_tr);
		free(line);
		return NULL;
	}
	if (strstr(line, "<buoy>") != NULL) {
		rd_bu = read_bu(fd);
		if (rd_bu == NULL) {
			free(rd_bu);
			free(rd_mi_tr);
			free(line);
			return NULL;
		} else {
			rd_mi_tr->finishB1 = *rd_bu;
		}
	} else {
		sprintf(buffer_pa, "error reading triangular mission");
		printf(buffer_pa);
		mp_send_log_info(buffer_pa);
		free(rd_bu);
		free(rd_mi_tr);
		free(line);
		return NULL;
	}

	// end marker
	line = read_line(fd, MAX_CHAR_LINE);
	if (line == NULL) {
		sprintf(buffer_pa, "error reading triangular mission");
		printf(buffer_pa);
		mp_send_log_info(buffer_pa);
		free(rd_bu);
		free(rd_mi_tr);
		free(line);
		return NULL;
	}
	if (strstr(line, "<end_mission>") == NULL) {
		free(rd_bu);
		free(rd_mi_tr);
		free(line);
		return NULL;
	}
	free(rd_bu);
	free(line);
	return rd_mi_tr;
}

rd_mission_sk* read_mission_sk(int fd, rd_mission_base *rd_mi_base) {
	char *line, *pch;

	// allocate memory
	rd_mission_sk *rd_mi_sk;
	rd_mi_sk = (rd_mission_sk*) calloc (1, sizeof(rd_mission_sk));

	// assign mission base
	rd_mi_sk->mi_base = *rd_mi_base;
	free(rd_mi_base);

	// time
	line = read_line(fd, MAX_CHAR_LINE);
	if (line == NULL) {
		sprintf(buffer_pa, "error reading station keeping mission");
		printf(buffer_pa);
		mp_send_log_info(buffer_pa);
		free(rd_mi_sk);
		free(line);
		return NULL;
	}
	pch = strtok(line, " ");
	pch = strtok(NULL, " ");
	rd_mi_sk->time = atoi(pch);

	// wind direction
	line = read_line(fd, MAX_CHAR_LINE);
	if (line == NULL) {
		sprintf(buffer_pa, "error reading station keeping mission");
		printf(buffer_pa);
		mp_send_log_info(buffer_pa);
		free(rd_mi_sk);
		free(line);
		return NULL;
	}
	pch = strtok(line, " ");
	pch = strtok(NULL, " ");
	rd_mi_sk->windDirAngle = atof(pch);

	// end marker
	line = read_line(fd, MAX_CHAR_LINE);
	if (line == NULL) {
		sprintf(buffer_pa, "error reading station keeping mission");
		printf(buffer_pa);
		mp_send_log_info(buffer_pa);
		free(rd_mi_sk);
		free(line);
		return NULL;
	}
	if (strstr(line, "<end_mission>") == NULL) {
		free(rd_mi_sk);
		free(line);
		return NULL;
	}
	free(line);
	return rd_mi_sk;
}

rd_mission_as* read_mission_as(int fd, rd_mission_base *rd_mi_base) {
	char *line, *pch;

	// allocate memory
	rd_mission_as *rd_mi_as;
	rd_mi_as = (rd_mission_as*) calloc (1, sizeof(rd_mission_as));

	// assign mission base
	rd_mi_as->mi_base = *rd_mi_base;
	free(rd_mi_base);

	// subdivision
	line = read_line(fd, MAX_CHAR_LINE);
	if (line == NULL) {
		sprintf(buffer_pa, "error reading area scanning mission");
		printf(buffer_pa);
		mp_send_log_info(buffer_pa);
		free(rd_mi_as);
		free(line);
		return NULL;
	}
	pch = strtok(line, " ");
	pch = strtok(NULL, " ");
	rd_mi_as->subdivision = atoi(pch);

	// wind direction
	line = read_line(fd, MAX_CHAR_LINE);
	if (line == NULL) {
		sprintf(buffer_pa, "error reading area scanning mission");
		printf(buffer_pa);
		mp_send_log_info(buffer_pa);
		free(rd_mi_as);
		free(line);
		return NULL;
	}
	pch = strtok(line, " ");
	pch = strtok(NULL, " ");
	rd_mi_as->windDirAngle = atof(pch);

	// end marker
	line = read_line(fd, MAX_CHAR_LINE);
	if (line == NULL) {
		sprintf(buffer_pa, "error reading area scanning mission");
		printf(buffer_pa);
		mp_send_log_info(buffer_pa);
		free(rd_mi_as);
		free(line);
		return NULL;
	}
	if (strstr(line, "<end_mission>") == NULL) {
		free(rd_mi_as);
		free(line);
		return NULL;
	}
	free(line);
	return rd_mi_as;
}

// read line from opened file descriptor
char* read_line(int fd, int maxChar) {
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
