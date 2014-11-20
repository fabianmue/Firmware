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
 * @file path_planning_data.c
 *
 * Store all optimal path planning data.
 *
 * @author Marco Tranzatto <marco.tranzatto@gmail.com>
 */

#include "path_planning_data.h"

#ifndef NULL
    #define NULL 0
#endif

//grid lines data
static struct{
    int32_t *x_cm_p;    ///array of x coordinates [cm] of grid lines, in Race frame
    int16_t size;       ///size of array x_cm_p
    int16_t next_goal;  ///index of next grid line to reach
}grid_lines;

/**
 * Initialize the grid lines struct. Delete all the old grid lines (if exsist).
*/
void init_grids(void){

    grid_lines.x_cm_p = NULL;
    grid_lines.size = 0;

    //set to 1 the number of grid lines before a real number is used
    set_grids_number(1);

}

/**
 * Set the new number of total grid lines. Calling this function you will delete
 * all the previous grid lines. Next grid line to be reach is that one with index = 0
 *
 * @param size  total number of new grid lines
*/
void set_grids_number(int16_t size){

    if(grid_lines.size == size || size <= 0)
        return; //nothing to do

    if(grid_lines.x_cm_p != NULL)
        free(grid_lines.x_cm_p);

    grid_lines.x_cm_p = malloc(sizeof(int32_t) * size);
    grid_lines.size = size;

    for(int16_t i = 0; i < size; i++)
        grid_lines.x_cm_p[i] = 0;

    grid_lines.next_goal = 0;

}

/**
 * Set the x coordinate in Race frame of a grid line.
 *
 * @param index     index of the grid line to be set
 * @param x_cm      x coordinate in Race frame of the new grid line
*/
void set_grid(int16_t index, int32_t x_cm){

    if(index >= grid_lines.size || index < 0)
        return; //wrong index

    grid_lines.x_cm_p[index] = x_cm;
}

/**
 * Read next grid line to reach, if any.
 *
 * @param next_grid_p   pointer to x coordinate in Race frame of the next grid line to reach
 * @return              true if there is a new grid line to reach, false otherwise.
*/
bool read_nex_grid(int32_t *next_grid_p){

    //if there are others grid lines to reach, return the first one of them
    if(grid_lines.next_goal < grid_lines.size){
        *next_grid_p = grid_lines.x_cm_p[grid_lines.next_goal];
        return true;
    }

    //if every grid lines has been reached, return false
    *next_grid_p = 0;
    return false;
}

void reached_current_grid(void){
    //update next_goal
    if(grid_lines.next_goal < grid_lines.size)
        grid_lines.next_goal++;
}
