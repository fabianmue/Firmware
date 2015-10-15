/*
 * mp_readSD.c
 *
 *  Created on: 09.10.2015
 *      Author: Fabian
 */

#include <stdbool.h>
#include <stdio.h>

int length = 100;
char content[100];

bool main ()
{
  FILE *GPSCoords;
  GPSCoords = fopen("test.txt", "r");
  if (GPSCoords != NULL)
  {
	  if (fgets(content, length, GPSCoords) != NULL)
	  return true;
  }
  else
  {
	  return false;
  }
}
