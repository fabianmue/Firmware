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
#include "mp_read_params.h"

#include <nuttx/config.h>
#include <fcntl.h>

#include "../path_planning/pp_navigation_module.h"

#define NAME_LEN 20
#define VALUE_LEN 20
/***********************************************************************************/
/*****  V A R I A B L E S  *********************************************************/
/***********************************************************************************/

char temp[VALUE_LEN];

static frame comp_frame;

static pointers_param_qgc_mp params_qgc;

static int32_t x0_dm = 0; ///x coordinate in ECEF of origin of NED system, in decimeters

static int32_t y0_dm = 0; ///y coordinate in ECEF of origin of NED system, in decimeters

static int32_t z0_dm = 0; ///z coordinate in ECEF of origin of NED system, in decimeters

PARAM_DEFINE_FLOAT(CF_O1N, 0.0f);
PARAM_DEFINE_FLOAT(CF_O1E, 0.0f);
PARAM_DEFINE_FLOAT(CF_DIST, 0.0f);
PARAM_DEFINE_FLOAT(CF_ROTATION, 0.0f);
PARAM_DEFINE_INT32(MP_MISSION, 0);


/***********************************************************************************/
/*****  F U N C T I O N   D E F I N I T I O N S  ***********************************/
/***********************************************************************************/


// read parameters from SD card
void mp_readSD(char file_path[])
{

  char buffer[1];

  // open file
  int fd = open(file_path, O_RDONLY);
  if (fd < 0) {
	  printf("unable to open file %s \n", file_path);
  } else {
	  printf("file at %s found \n", file_path);

	  /* allocate memory for each parameter */
	  int ch = 0, number_of_lines = 1;
	  do {
		  ch = read(fd, buffer, 1);
		  if (ch <= 0) {
			  perror("error when reading: ");
		  }
		  printf("buffer: %s\n", buffer);
		  if (buffer[0] == '\n') number_of_lines++;
	  } while (ch > 0);
	  struct SD_params_s SD_params[number_of_lines];
	  printf("number of lines in file: %i \n", number_of_lines);

	  // reset file stream position
	  lseek(fd, 0, SEEK_SET);

	  // read parameter names and values
	  int ch2;
	  int i = 0, j = 0, k = 0;
	  do {
		  ch2 = read(fd, buffer, 1);
		  if (isalpha(buffer[0])) {
			  j = 0;
			  do {
				  SD_params[i].name[j] = buffer[0];
				  ch2 = read(fd, buffer, 1);
				  j++;
			  } while (isalpha(buffer[0]) && j < NAME_LEN-1);
			  SD_params[i].name[j] = '\0';
			  printf("read parameter %i, name: %s\n", i, SD_params[i].name);
		  }	else if (isdigit(buffer[0])) {
			  k = 0;
			  do {
				  temp[k] = buffer[0];
				  ch2 = read(fd, buffer, 1);
				  k++;
			  } while (isdigit(buffer[0]) && k < VALUE_LEN-1);
			  temp[k] = '\0';
			  SD_params[i].value = atof(temp);
			  printf("read parameter %i, value: %f\n", i, SD_params[i].value);
		  } else if (buffer[0] == '\n') {
			  i++;
		  };
	  } while (ch2 > 0);
	  printf("end of file reached");
  }
};

// initialize parameters
void mp_param_init_qgc(void){

    // geo reference
	params_qgc.lat0_pointer = param_find("ASP_R_LAT0_E7");
	params_qgc.lon0_pointer = param_find("ASP_R_LON0_E7");
	params_qgc.alt0_pointer = param_find("ASP_R_ALT0_E3");

	// competition frame
	params_qgc.cf_o1n = param_find("CF_O1N");
	params_qgc.cf_o1e = param_find("CF_O1E");
	params_qgc.cf_dist = param_find("CF_DIST");
	params_qgc.cf_rotation = param_find("CF_ROTATION");

	// mission data
	params_qgc.mp_mission = param_find("MP_MISSION");

    p_param_update(false);
}

// update parameters
void mp_param_update_qgc(bool update_path_param){

    //  geo reference
    int32_t lat0;
    int32_t lon0;
    int32_t alt0;

    // latitude
    param_get(params_qgc.lat0_pointer, &lat0);

    // longitude
    param_get(params_qgc.lon0_pointer, &lon0);

    // altitude
    param_get(params_qgc.alt0_pointer, &alt0);

    // update NED origin in pp_navigation_module
    mp_set_ref0(&lat0, &lon0, &alt0);

   	// competition frame
   	float cf_dist = 0.0f;
   	float cf_o1n = 0.0f;
   	float cf_o1e = 0.0f;
   	float cf_rotation = 0.0f;
   	param_get(params_qgc.cf_o1n, &cf_o1n);
   	param_get(params_qgc.cf_o1e, &cf_o1e);
   	param_get(params_qgc.cf_dist, &cf_dist);
   	param_get(params_qgc.cf_rotation, &cf_rotation);

   	// mission
   	int32_t mp_mission = 0;
   	param_get(params_qgc.mp_mission, &mp_mission);

	#if LDEBUG_USEMISSION == 1
   	mp_set_racefield(cf_o1n, cf_o1e, cf_dist, cf_rotation);
	#endif
}

// set GPS reference of buoy 1
void mp_set_ref0(const int32_t *lat, const int32_t *lon, const int32_t *alt){

    //set ecef reference of NED origin
    geo_to_ecef(lat, lon, alt, &x0_dm, &y0_dm, &z0_dm);

}

// rotate NEDpoint around another NEDpoint
NEDpoint nh_rotate(NEDpoint torot, NEDpoint center, float angle) {


	NEDpoint result;

	result.northx = center.northx + cosf(angle)*(center.northx-torot.northx) + sinf(angle)*(center.easty-torot.easty);
	result.easty = center.easty + cosf(angle)*(center.easty - torot.easty) - sinf(angle)*(center.northx-torot.northx);

	return result;
}

// set the configuration of the race-field
void mp_set_racefield(float O1n, float O1e, float dist, float rotation) {

	// frame parameters
	comp_frame.dist = dist;
	comp_frame.rotation = DEG2RAD * rotation;

	// frame reference buoy
	comp_frame.O1.northx = O1n;
	comp_frame.O1.easty = O1e;

	// other frame buoys
	comp_frame.O2.northx = comp_frame.O1.northx;
	comp_frame.O2.easty =  comp_frame.O1.easty + comp_frame.dist;
	comp_frame.O3.northx = comp_frame.O1.northx - comp_frame.dist;
	comp_frame.O3.easty = comp_frame.O1.easty;
	comp_frame.O4.northx = comp_frame.O1.northx - comp_frame.dist;
	comp_frame.O4.easty = comp_frame.O1.easty + comp_frame.dist;

	// rotate around reference buoy
	comp_frame.O2 = nh_rotate(comp_frame.O2, comp_frame.O1, comp_frame.rotation);
	comp_frame.O3 = nh_rotate(comp_frame.O3, comp_frame.O1, comp_frame.rotation);
	comp_frame.O4 = nh_rotate(comp_frame.O4, comp_frame.O1, comp_frame.rotation);
}
