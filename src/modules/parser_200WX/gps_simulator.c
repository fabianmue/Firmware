/****************************************************************************
 *
 *   Copyright (c) 2012, 2013 PX4 Development Team. All rights reserved.
 *   Author: Marco Tranzatto <marco.tranzatto@gmail.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file gps_simulator.c
 *
 * Interface for common utility functions.
 *
 *
 * @author Marco Tranzatto <marco.tranzatto@gmail.com>
 */

#include "gps_simulator.h"

static int counter = 0;
static int elap_sec = 0;//start from midnight


double degree2nmea_ndeg(double input)
{
    double deg;
    double min;
    double secOver60;
    double app;

    deg = (int)input;
    app =((input - deg) * 60.0);
    min = (int) app;

    secOver60 = (app - ((int)app));
   // printf("deg: %f \t min %f \t secOver60: %f \n",deg, min, secOver60);

    return deg * 100.0 + min + secOver60;
}

void sim_steady_pos(char *buf, int *lgt){

    double geo[3];
    double lat;
    double lon;
    int32_t alt;

    //force time to increase of 5 seconds
    elap_sec += 5;
    int32_t hh = elap_sec / 3600;
    int32_t mm = (elap_sec - hh * 3600) /60;
    double ss = (elap_sec - hh * 3600 - mm * 60);

    //double time = hh * 1e4 + mm * 1e2 + ss;

    switch(counter){
        case 0:
            geo[0] = 8.554268;
            geo[1] = 47.378619;
            geo[2] = 593;
            break;

        case 1:
            geo[0] = 8.552529;
            geo[1] = 47.378729;
            geo[2] = 593;
            break;

        case 2:
            geo[0] = 8.552644;
            geo[1] = 47.378414;
            geo[2] = 593;
            break;

        case 3:
            geo[0] = 8.552391;
            geo[1] = 47.378563;
            geo[2] = 593;
            break;

    }

    lat = degree2nmea_ndeg(geo[1]);
    lon = degree2nmea_ndeg(geo[0]);
    alt = (int)geo[2];

    counter++;
    if(counter >= 3)
        counter = 0;

    sprintf(buf, "GPGGA,hhmmss.ss,%4.4f,N,%4.4f,E,3,8,9.3,%d,M,********************* \
            GPVTG,182.9,T,181.0,M,0.0,N,15.9,K,A,*,\
            $,GPGSA,A,3,11,17,20,4,,,,,,,,,14.0,9.3,10.5,*,55555...\
            $GPZDA,%02u%02u%06.3f,10,11,2014,00,00*4...5555555555555555555555555555555555\0\0",
            lat, lon, alt, hh, mm, ss);

    *lgt = strlen(buf);
}


void sim_gps(char *buf, int *lgt){

    double geo[3];
    double lat;
    double lon;
    int32_t alt;

    //force time to increase of 5 seconds
    elap_sec += 5;
    int32_t hh = elap_sec / 3600;
    int32_t mm = (elap_sec - hh * 3600) /60;
    double ss = (elap_sec - hh * 3600 - mm * 60);

    //double time = hh * 1e4 + mm * 1e2 + ss;

    switch(counter){
        case 0:
            geo[0] = 8.552155927209634;
            geo[1] = 47.38180958406829;
            geo[2] = 593;
            break;

        case 1:
            geo[0] = 8.552204460405656;
            geo[1] = 47.3818752359201;
            geo[2] = 593;
            break;

        case 2:
            geo[0] = 8.552636675577009;
            geo[1] = 47.38210219435015;
            geo[2] = 593;
            break;

        case 3:
            geo[0] = 8.552924073292317;
            geo[1] = 47.38219828452652;
            geo[2] = 593;
            break;

        case 4:
            geo[0] = 8.553872591945817;
            geo[1] = 47.38202083356003;
            geo[2] = 593;
            break;

        case 5:
            geo[0] = 8.553965106110406;
            geo[1] = 47.38188730804277;
            geo[2] = 593;
            break;

        case 6:
            geo[0] = 8.553901551437921;
            geo[1] = 47.38109377741429;
            geo[2] = 593;
            break;

        case 7:
            geo[0] = 8.5538986756408;
            geo[1] = 47.38092957749006;
            geo[2] = 593;
            break;

        case 8:
            geo[0] = 8.554550975126267;
            geo[1] = 47.38046219491586;
            geo[2] = 593;
            break;

        case 9:
            geo[0] = 8.554738780698042;
            geo[1] = 47.38042710032785;
            geo[2] = 593;
            break;
    }

    lat = degree2nmea_ndeg(geo[1]);
    lon = degree2nmea_ndeg(geo[0]);
    alt = (int)geo[2];

    counter++;
    if(counter >= 9)
        counter = 0;

    sprintf(buf, "GPGGA,hhmmss.ss,%4.4f,N,%4.4f,E,3,8,9.3,%d,M,********************* \
            GPVTG,182.9,T,181.0,M,0.0,N,15.9,K,A,*,\
            $,GPGSA,A,3,11,17,20,4,,,,,,,,,14.0,9.3,10.5,*,55555...\
            $GPZDA,%02u%02u%06.3f,10,11,2014,00,00*4...5555555555555555555555555555555555\0\0",
            lat, lon, alt, hh, mm, ss);

    *lgt = strlen(buf);
}
