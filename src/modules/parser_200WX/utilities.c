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
 * @file utilities.c
 *
 * Implementation of common utility functions.
 *
 *
 * @author Marco Tranzatto <marco.tranzatto@gmail.com>
 */

#include "utilities.h"

/**
* Find if string is in buffer starting from start_index and going til the end.
*
* @param start_index    index where start to find string
* @param buffer         buffer where search for string
* @param buffer_length  length of buffer
* @param str         string to find in buffer
* @return 	the index in the buffer where 'string' begins in the buffer, -1 if not found
*/
int find_string_everywhere(const int start_index, const char *buffer, const int buffer_length, const char *str){

    int i;
    int str_len = strlen(str);
    char temp_str[10]; /**< str cannot be greater than 9 charachter! */
    int stop_index = buffer_length - str_len +1;

    for(i = start_index; i < stop_index; i++){

        strncpy(temp_str, &buffer[i], str_len);
        //add null-characther at the end
        temp_str[str_len] = '\0';

        if(!strcmp(temp_str, str)){
            //found str in buffer, starting from i
            return i;
        }
    }

    //str not found in buffer
    return -1;
}

/**
* Find if string is in buffer starting exatcly from start_index.
*
* @param start_index    index where start to find string
* @param buffer         buffer where search for string
* @param buffer_length  length of buffer
* @param str         string to find in buffer
* @return 	the index in the buffer where 'string' begins in the buffer ret == start_index on succes, -1 if not found
*/
int find_string_here(const int start_index, const char *buffer, const int buffer_length, const char *str){

    int str_len = strlen(str);
    char temp_str[15]; /**< str cannot be greater than 14 charachter! */

    if(start_index + str_len >= buffer_length)
        return -1; //not enough characters in the buffer

    strncpy(temp_str, &buffer[start_index], str_len);
    //add null-characther at the end
    temp_str[str_len] = '\0';

    if(!strcmp(temp_str, str)){
        //found str in buffer, starting from start_index
        return start_index;
    }

    //str not found in buffer
    return -1;
}

/**
* Extract data from buffer, starting from index, until a coma is found. Update index. Returns a double
*
* @param index_pointer      pointer to index to be updated at the end of the function, if no error, buffer[i] = ','
* @param buffer             buffer
* @param buffer_length      length of buffer
* @param ret_val_pointer    pointer to variable with the final result
* @return 	true if no error
*/
bool d_extract_until_coma(int *index_pointer, const char *buffer, const int buffer_length, double *ret_val_pointer){

    int counter = 0;
    char temp_char[SAFETY_COUNTER_EXTRACT];
    int i = *index_pointer;

    while(i < buffer_length && buffer[i] != ','){

        temp_char[counter] = buffer[i];

        i++;
        counter++;
        if(counter >= SAFETY_COUNTER_EXTRACT){
            *ret_val_pointer = 0;
            return false;
        }
    }

    //check if we exited from while loop because we have a valid data or not
    if(i == *index_pointer || i >= buffer_length){
        *ret_val_pointer = 0;
        return false; //not found valid data
    }

    //null terminate string
    temp_char[counter] = '\0';

    //update index
    *index_pointer = i;
    *ret_val_pointer = atof(temp_char);

    return true;
}

/**
* Extract data from buffer, starting from index, until a coma is found. Update index. Returns a float
*
* @param index_pointer      pointer to index to be updated at the end of the function, if no error, buffer[i] = ','
* @param buffer             buffer
* @param buffer_length      length of buffer
* @param ret_val_pointer    pointer to variable with the final result
* @return 	true if no error
*/
bool f_extract_until_coma(int *index_pointer, const char *buffer, const int buffer_length, float *ret_val_pointer){

    int counter = 0;
    char temp_char[SAFETY_COUNTER_EXTRACT];
    int i = *index_pointer;

    while(i < buffer_length && buffer[i] != ','){

        temp_char[counter] = buffer[i];

        i++;
        counter++;
        if(counter >= SAFETY_COUNTER_EXTRACT){
            *ret_val_pointer = 0;
            return false;
        }
    }

    //check if we exited from while loop because we have a valid data or not
    if(i == *index_pointer || i >= buffer_length){
        *ret_val_pointer = 0;
        return false; //not found valid data
    }

    //null terminate string
    temp_char[counter] = '\0';

    //update index
    *index_pointer = i;
    *ret_val_pointer = (float)atof(temp_char);

    return true;
}

/**
* Extract data from buffer, starting from index, until a coma is found. Update index. Returns an int.
*
* @param index_pointer      pointer to index to be updated at the end of the function, if no error, buffer[i] = ','
* @param buffer             buffer
* @param buffer_length      length of buffer
* @param ret_val_pointer    pointer to variable with the final result
* @return 	true if no error
*/
bool i_extract_until_coma(int *index_pointer, const char *buffer, const int buffer_length, int *ret_val_pointer){

    int counter = 0;
    char temp_char[SAFETY_COUNTER_EXTRACT];
    int i = *index_pointer;

    while(i < buffer_length && buffer[i] != ','){

        temp_char[counter] = buffer[i];

        i++;
        counter++;
        if(counter >= SAFETY_COUNTER_EXTRACT){
            *ret_val_pointer = 0;
            return false;
        }
    }

    //check if we exited from while loop because we have a valid data or not
    if(i == *index_pointer || i >= buffer_length){
        *ret_val_pointer = 0;
        return false; //not found valid data
    }

    //null terminate string
    temp_char[counter] = '\0';

    //update index
    *index_pointer = i;
    *ret_val_pointer = atoi(temp_char);

    return true;
}

/**
* Extract data from buffer, starting from index, until a '*' is found. Update index. Returns an int.
*
* @param index_pointer      pointer to index to be updated at the end of the function, if no error, buffer[i] = ','
* @param buffer             buffer
* @param buffer_length      length of buffer
* @param ret_val_pointer    pointer to variable with the final result
* @return 	true if no error
*/
bool i_extract_until_star(int *index_pointer, const char *buffer, const int buffer_length, int *ret_val_pointer){

    int counter = 0;
    char temp_char[SAFETY_COUNTER_EXTRACT];
    int i = *index_pointer;

    while(i < buffer_length && buffer[i] != '*'){

        temp_char[counter] = buffer[i];

        i++;
        counter++;
        if(counter >= SAFETY_COUNTER_EXTRACT){
            *ret_val_pointer = 0;
            return false;
        }
    }

    //check if we exited from while loop because we have a valid data or not
    if(i == *index_pointer || i >= buffer_length){
        *ret_val_pointer = 0;
        return false; //not found valid data
    }

    //null terminate string
    temp_char[counter] = '\0';

    //update index
    *index_pointer = i;
    *ret_val_pointer = atoi(temp_char);

    return true;
}

/**
 * Jump to next the ',' in buffer. -1 on error.
 *
 * If buffer[start_index] is a ',' returns start_index.
 * @return -1 if no ',' found, index of ','in the buffer on succes.
*/
int jump_to_next_coma(const int start_index, const char *buffer, const int buffer_length){

    int i = start_index;

    while(i < buffer_length && buffer[i] != ','){
        i++;
    }

    //check if we exited from while loop because we have a ',' or because we arraived at the end
    if(i >= buffer_length)
        return -1;

    return i;
}

/**
 * Jump n ',' in buffer. -1 on error.
 *
 * If buffer[start_index] is a ',' returns start_index.
 * @return -1 if no ',' found. On succes index of n-th ',' in the buffer (Counting starting from start_index).
*/
int jump_to_next_n_coma(const int start_index, const char *buffer, const int buffer_length, int n){

    int coma_index = -1;
    int new_start = start_index;
    int j;

    for(j = 0; j < n; j++){
        coma_index = jump_to_next_coma(new_start, buffer, buffer_length);
        if(coma_index == -1)
            return -1;
        else{
            //coma_index is ',', go ahead with new start
            new_start = coma_index + 1;
        }
    }
    return coma_index;
}


/**
 * Print data in buffer from start to end (or end of buffer). buffer[end] is not printed.
*/
void debug_print_nchar(const char *buffer, const int length, const int start, const int end){
    char str[301];
    int i;

    for(i = 0; (i + start) < length && (i + start) <= end && i < 300; i++){

        str[i] = buffer[start+i];
    }

    str[i] = '\0';

    warnx("buf_len %d; start %d end %d real_end %d \n %s \n", length, start, end, start + i-1, str);
}

/**
 * Print data in buffer from start untile stop_char is found (or end of buffer). stop_char is not printed
*/
void debug_print_until_char(const char *buffer, const int length, const int start, const char stop_char){
    char str[301];
    int i;

    for(i = 0; (i + start) < length && buffer[start+i] != stop_char && i < 300; i++){

        str[i] = buffer[start+i];
    }

    str[i] = '\0';

    warnx("buf_len %d; start %d  real_end %d \n %s \n", length, start, start + i-1, str);
}

