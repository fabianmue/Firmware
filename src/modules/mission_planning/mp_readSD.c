/*
 * mp_readSD.c
 *
 *  Created on: 09.10.2015
 *      Author: Fabian
 */

#include <stdlib.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <ctype.h>

#include "mp_readSD.h"

/***********************************************************************************/
/*****  V A R I A B L E S  *********************************************************/
/***********************************************************************************/

struct in_param {
	char name[20];
	double value;
};

int name_len = 20;

char temp[20];

int value_len = sizeof(temp)/sizeof(char);

/***********************************************************************************/
/*****  P U B L I C    F U N C T I O N S  ******************************************/
/***********************************************************************************/

bool mp_readSD(char *filename)
{

  FILE *in_stream;
  in_stream = fopen(filename, 'r');

  if (in_stream != NULL) {

	  printf("file %s opened successfully", filename);

	  /* allocate memory for each parameter */
	  int ch, number_of_lines = 0;
	  do {
		  ch = fgetc(in_stream);
		  if (ch == '\n') number_of_lines++;
	  } while (ch != EOF);
	  struct in_param params[number_of_lines];

	  /* read parameter names and values */
	  int ch2;
	  int i = 0, j = 0, k = 0;
	  do {
		  ch2 = fgetc(in_stream);
		  if (isalpha(ch2)) {
			  j = 0;
			  do {
				  params[i].name[j] = ((char) ch2);
				  ch2 = fgetc(in_stream);
				  j++;
			  } while (isalpha(ch2) && j < name_len-1);
			  params[i].name[j] = '\0';
			  printf("read parameter %i, name: %s", i, params[i].name);
		  }	else if (isdigit(ch2)) {
			  k = 0;
			  do {
				  temp[k] = ((char) ch2);
				  ch2 = fgetc(in_stream);
				  k++;
			  } while (isdigit(ch2) && k < value_len-1);
			  temp[k] = '\0';
			  params[i].value = atof(temp);
			  printf("read parameter %i, value: %f", i, params[i].value);
		  } else if (((char) ch2) == '\n') {
			  i++;
		  };
	  } while (ch2 != EOF && ch2 != NULL);

	  return true;
  }
  else {

	  printf("unable to open file %s", filename);
	  return false;
  }
}
