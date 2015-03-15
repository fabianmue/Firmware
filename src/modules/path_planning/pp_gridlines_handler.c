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
 * @file pp_gridlines_handler.c
 *
 * handler for gir lines system.
 *
 * @author Marco Tranzatto <marco.tranzatto@gmail.com>
 */

#include "pp_gridlines_handler.h"


#ifndef NULL
    #define NULL 0
#endif

#define M_PI_F 3.14159265358979323846f

static float next_grid;

static char txt_msg[150]; //used to send messages to QGC

//grid lines data
static struct{
    float *x_m_p;           ///array of rays [m] of grid lines, in Race frame
    int16_t size;           ///size of array x_m_p
    int16_t current_goal;   ///index of current grid line to reach
    int16_t last_goal;      ///index of the last grid line to reach
    int16_t grid_counter;      ///number of grid lines inserted
}grid_lines =   {
                    .x_m_p = NULL,
                    .size = 0,
                    .current_goal = -1,
                    .last_goal = -1,
                    .grid_counter = 0
                };

/** @brief advise that the current goal grid line has been reached*/
void reached_current_grid(void);

/** @brief set number of grid lines*/
void set_grids_number(int16_t size);

/** @brief set the ray of a grid line*/
void set_grid(float ray_m);

/** @brief check if there is at least one valid grid line to reach */
bool is_there_grid_line(void);

/**
 * Initialize the grid lines struct. Delete all the old grid lines (if any).
*/
void gh_init_grids(void){

    grid_lines.x_m_p = NULL;
    grid_lines.size = 0;
    grid_lines.grid_counter = 0;

    //set to 1 the number of grid lines before a real number is used
    set_grids_number(1);
}

/**
 * Set the new number of total grid lines. Calling this function you will delete
 * all the previous grid lines. Next grid line to be reach is the one with index = 0
 *
 * @param size  total number of new grid lines
*/
void set_grids_number(int16_t size){

    if(grid_lines.size == size || size <= 0)
        return; //nothing to do

    if(grid_lines.x_m_p != NULL)
        free(grid_lines.x_m_p);//delete old data

    //update grid_lines
    grid_lines.x_m_p = malloc(sizeof(float) * size);
    grid_lines.size = size;

    //set to 0, default
    for(int16_t i = 0; i < size; i++)
        grid_lines.x_m_p[i] = 0.0f;

    //we still do not have a valid next grid line to reach
    grid_lines.current_goal = -1;
    grid_lines.last_goal = -1;

    //send a message to QGC to tell that grid lines queue has been reset
    sprintf(txt_msg, "Grid lines queue reset, new dim: %d.", size);
    smq_send_log_info(txt_msg);
}

/**
 * Set the new number of total grid lines. Calling this function you will delete
 * all the previous grid lines if the the new size is different from the previous one.
 * Attention: if the size is equal to the actual size of the grid lines array, NO action
 * will be performed!
 * The next grid line to be reach is the one with index = 0
 * Grid line number can be set by QGroundControl.
 *
 * @param size  total number of new grid lines
*/
void gh_set_grids_number_qgc(int16_t size){
        set_grids_number(size);
}

/**
 * Set the ray of a grid line.
 * Grid lines are inserted with a FIFO policy.
 *
 * @param ray_m       ray [m] of the new circular grid line
*/
void set_grid(float ray_m){

    //if last_goal != -1, it is the position of the last grid line inserted
    if((grid_lines.last_goal + 1) >= grid_lines.size){
        //send msg to QGC via mavlink
        strcpy(txt_msg, "Not enough space to add a grid line.");
        smq_send_log_info(txt_msg);

        return; //not enough space to add a new grid line
    }

    //enough space, add the new grid line
    grid_lines.last_goal = grid_lines.last_goal + 1;
    grid_lines.x_m_p[grid_lines.last_goal] = ray_m;
    grid_lines.grid_counter++;

    //if this is the first grid line inserted, it's even the current goal grid line
    if(grid_lines.last_goal == 0){
        grid_lines.current_goal = 0;
    }

    //send a message to QGC to tell that a new grid line has been added
    sprintf(txt_msg, "Added grid number %d at %3.2f meters.", grid_lines.last_goal, (double)ray_m);
    smq_send_log_info(txt_msg);
}

/**
 * Set the the ray of a grid line.
 * Grid lines are inserted with a FIFO policy, if there is enough space.
 *
 * @param x_m       ray [m] of the new circular grid line
*/
void gh_set_grid_qgc(float ray_m){
        set_grid(ray_m);
}

/**
 * Check if there is at least one grid line to reach
 *
 * @return true if there is at least one grid line to reach
*/
bool is_there_grid_line(void){

    return (grid_lines.current_goal != -1) ? true : false;
}


/**
 * Notify that the current grid line has been reached.
*/
void reached_current_grid(void){
    //update current_goal index if there is at least one new grid line to reach
    if(grid_lines.current_goal < grid_lines.last_goal){
        //we have other grid line to reach
        grid_lines.current_goal++;
    }
    else{
        //we have reached the last grid line, empty the buffer
        grid_lines.current_goal = -1;
        grid_lines.last_goal = -1;
    }

    // send a message to QGC to tell that a new grid line has been reached
    sprintf(txt_msg, "Grid line reached.");
    smq_send_log_info(txt_msg);
}

/**
 * Use all the grid lines set in the past.
 *
 * @param use   true if you want to use all the grid lines stored in the buffer.
*/
void gh_reuse_last_grids(bool use){

    if(use){
        //use all the grid lines set previously
        grid_lines.current_goal = 0;
        grid_lines.last_goal = grid_lines.grid_counter - 1;

        //send a message to QGC
        sprintf(txt_msg, "Reinserted previous grid lines, first at %0.1f [m]",
                (double) grid_lines.x_m_p[grid_lines.current_goal]);

        smq_send_log_info(txt_msg);
    }
}

/**
 * Get the next grid line the boat should reach, if there is at least one.
 *
 * @param next_grid_p   *next_grid_p <-- next grid line [m]
 * @return  true if there is at least one grid line to reach
*/
bool gh_get_next_gridline(float *next_grid_p){

    bool valid_grid;

    if(is_there_grid_line()){
        *next_grid_p = grid_lines.x_m_p[grid_lines.current_goal];
        valid_grid = true;
    }
    else
        valid_grid = false;

    return valid_grid;
}

/**
 * Set what should be the next reference actions if we are using grid lines.
 *
 * Grid lines are used to tell the boat when to tack. When the boat
 * reaches a valid grid line (that is not the last one),
 * it should tack as soon as possibile and start sailing with the new haul.
 * When the last inserted grid line is reached, the boat should turn
 * and start sailing downwind.
*/
void gh_gridlines_handler(void){

    //check if we have a valid grid line to reach, if so, take it
    if(gh_get_next_gridline(&next_grid)){
        //see if we have reached or exceeded our goal
        if(n_get_dist_m() >= next_grid){

            //Advise we have reached the current target grid line
            reached_current_grid();

            //take the new grid line, if any
            float tmp_grid;
            if(gh_get_next_gridline(&tmp_grid)){
                // there is at least another grid line to reach, tack now
                cb_tack_now();
            }
            else{
                //this was the last grid line we had to reach
                cb_reached_last_griline();
            }
        }
    }
}

