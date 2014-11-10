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

    double geo[50][3] = {{8.5526864,47.3785312,496.7000122},{8.552644,47.3785408,502.2000122},{8.5526152,47.3785312,507.9000244},{8.5526208,47.3784672,506.5000305},{8.552644,47.3784832,506.7000122},{8.5526336,47.3784608,508.8000183},{8.552616,47.3784992,508.5000305},{8.5526184,47.3784992,507.8000183},{8.5525712,47.3784736,502.9000244},{8.55252,47.3785056,503.5000305},{8.55264,47.3784992,506.6000366},{8.5526,47.3784992,499.2000122},{8.5526136,47.3784672,504.6000366},{8.5526712,47.3785056,508.9000244},{8.552652,47.3784832,502.9000244},{8.5526704,47.3784672,502.8000183},{8.5525688,47.3784832,504.8000183},{8.5526992,47.3784736,500.9000244},{8.5527264,47.3785056,504.5000305},{8.5526944,47.3784832,505.5000305},{8.5526896,47.3785056,502.4000244},{8.5526712,47.3785056,500.9000244},{8.5527128,47.3785056,501.7000122},{8.5526664,47.3784992,503.9000244},{8.5526816,47.3784896,504.9000244},{8.552676,47.3785184,504.1000366},{8.5526584,47.3784832,504.1000366},{8.5526736,47.3784608,504.7000122},{8.5526752,47.3784736,504.7000122},{8.5526296,47.3784608,503.8000183},{8.5526656,47.378432,507.7000122},{8.5526464,47.3784992,510.9000244},{8.5526352,47.3784832,507.2000122},{8.55266,47.3784896,509.6000366},{8.5527016,47.3785056,503.8000183},{8.5527136,47.3785248,504.4000244},{8.5527016,47.3785056,504.9000244},{8.5527184,47.3784992,504.4000244},{8.5527016,47.3784992,506.3000183},{8.5526968,47.3785248,507.3000183},{8.5526808,47.3785184,508.4000244},{8.552676,47.3784896,506.2000122},{8.5527,47.3784896,505.4000244},{8.5526848,47.3784992,505.6000366},{8.5526584,47.3785312,505.7000122},{8.5526968,47.3785184,502.4000244},{8.5526816,47.3785408,507.7000122},{8.5526992,47.3784992,505.6000366},{8.5526704,47.3784832,504.5000305},{8.5527016,47.3784992,503.3000183}};

    double lat;
    double lon;
    int32_t alt;

    //force time to increase of 5 seconds
    elap_sec += 5;
    int32_t hh = elap_sec / 3600;
    int32_t mm = (elap_sec - hh * 3600) /60;
    double ss = (elap_sec - hh * 3600 - mm * 60);


//    switch(counter){
//        case 0:
//            geo[0] = 8.55252;
//            geo[1] = 47.3784992;
//            geo[2] = 503.6;
//            break;

//        case 1:
//            geo[0] = 8.5527424;
//            geo[1] = 47.3784992;
//            geo[2] = 505.5;
//            break;

//        case 2:
//            geo[0] = 8.5526808;
//            geo[1] = 47.3785472;
//            geo[2] = 502.6;
//            break;

//        case 3:
//            geo[0] = 8.5526552;
//            geo[1] = 47.3784192;
//            geo[2] = 507.1;
//            break;

//    }

    lat = degree2nmea_ndeg(geo[counter][1]);
    lon = degree2nmea_ndeg(geo[counter][0]);
    alt = (int)geo[counter][2];

    counter++;
    if(counter >= 50)
        counter = 0;

    sprintf(buf, "GPGGA,hhmmss.ss,%4.4f,N,%4.4f,E,3,8,2,%d,M,********************* \
            GPVTG,182.9,T,181.0,M,0.0,N,0.0,K,A,*,\
            $,GPGSA,A,3,11,17,20,4,,,,,,,,,14.0,9.3,2*,55555...\
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
            GPVTG,182.9,T,181.0,M,0.0,N,2.5,K,A,*,\
            $,GPGSA,A,3,11,17,20,4,,,,,,,,,14.0,9.3,10.5,*,55555...\
            $GPZDA,%02u%02u%06.3f,10,11,2014,00,00*4...5555555555555555555555555555555555\0\0",
            lat, lon, alt, hh, mm, ss);

    *lgt = strlen(buf);
}
