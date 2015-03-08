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

static char txt_msg[150]; ///used to send messages to QGC

//grid lines data
static struct{
    float *x_m_p;           ///array of x coordinates [m] of grid lines, in Race frame
    int16_t size;           ///size of array x_m_p
    int16_t current_goal;   ///index of current grid line to reach
    int16_t last_goal;      ///index of the last grid line to reach
    int16_t grids_inserted; /// number of grid lines inserted
}grid_lines;

static float current_grid_goal_x_m = 0.0f;//current x coordinate of grid line to reach [m]
static bool current_grid_valid = false;

//static bool make_boat_tack = false; //true if the boat should tack now

/** @brief read next grid line to reach*/
bool read_nex_grid(float *next_grid_p);

/** @brief advise that the current goal grid line has been reached*/
void reached_current_grid(void);

/** @brief set number of grid lines*/
void set_grids_number(int16_t size);

/** @brief set the x coordinate of a grid line*/
void set_grid(float x_m);

/**
 * Initialize the grid lines struct. Delete all the old grid lines (if any).
*/
void gh_init_grids(void){

    grid_lines.x_m_p = NULL;
    grid_lines.size = 0;
    grid_lines.grids_inserted = 0;

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
    current_grid_goal_x_m = 0;
    current_grid_valid = false;
    grid_lines.grids_inserted = 0;

    //send a message to QGC to tell that grid lines queue has been reset
    sprintf(txt_msg, "Grid lines queue reset, new dim: %d.", size);
    smq_send_log_info(txt_msg);
}

/**
 * Set the new number of total grid lines. Calling this function you will delete
 * all the previous grid lines. Next grid line to be reach is the one with index = 0
 * Grid line number can be set by QGroundControl.
 *
 * @param size  total number of new grid lines
*/
void gh_set_grids_number_qgc(int16_t size){
        set_grids_number(size);
}

/**
 * Set the x coordinate in Race frame of a grid line.
 * Grid lines are inserted with a FIFO policy.
 *
 * @param x_m       x coordinate [m] in Race frame of the new grid line
*/
void set_grid(float x_m){

    //if last_goal != -1, it is the position of the last grid line inserted
    if((grid_lines.last_goal + 1) >= grid_lines.size){
        //send msg to QGC via mavlink
        strcpy(txt_msg, "Not enough space to add a grid line.");
        smq_send_log_info(txt_msg);

        return; //not enough space to add a new grid line
    }

    //enough space, add the new grid line
    grid_lines.last_goal = grid_lines.last_goal + 1;
    grid_lines.x_m_p[grid_lines.last_goal] = x_m;

    //if this is the first grid line inserted, it's the current goal grid line
    if(grid_lines.last_goal == 0){
        grid_lines.current_goal = grid_lines.last_goal;
        current_grid_goal_x_m = x_m;
        current_grid_valid = true;
    }

    grid_lines.grids_inserted++;

    //send a message to QGC to tell that a new grid line has been added
    sprintf(txt_msg, "Added grid number %d at %3.2f meters.", grid_lines.last_goal, (double)x_m);
    smq_send_log_info(txt_msg);
}

/**
 * Set the x coordinate in Race frame of a grid line.
 * Grid lines are inserted with a FIFO policy.
 *
 * @param x_m       x coordinate [m] in Race frame of the new grid line
*/
void gh_set_grid_qgc(float x_m){
        set_grid(x_m);
}

/**
 * Read next grid line to reach, if any.
 *
 * @param next_grid_p   pointer to x coordinate in Race frame of the next grid line to reach
 * @return              true if there is a new grid line to reach, false otherwise.
*/
bool read_nex_grid(float *next_grid_p){

    //if there are others grid lines to reach, return the first one of them
    if(grid_lines.current_goal != -1){
        *next_grid_p = grid_lines.x_m_p[grid_lines.current_goal];
        return true;
    }

    //if every grid lines has been reached, return false
    *next_grid_p = 0.0f;
    return false;
}

/**
 * Notify that the current grid line has been reached
*/
void reached_current_grid(void){
    //update current_goal index if there is at least one new grid line to reach
    if(grid_lines.current_goal < grid_lines.last_goal){
        //we have other grid lines to reach
        grid_lines.current_goal++;
    }
    else{
        //we have reached the last grid line, empty the buffer
        grid_lines.current_goal = -1;
        grid_lines.last_goal = -1;
    }

    /*send a message to QGC to tell that a new grid line has been reached
    */
    sprintf(txt_msg, "Grid line reached.");
    smq_send_log_info(txt_msg);
}

void gh_reuse_last_grids(bool use){
    //TODO: change this!!!
    if(use){
        //copy all the grid lines inserted
        float *old_grids = malloc(sizeof(float) * grid_lines.grids_inserted);
        int16_t old_real_size = grid_lines.grids_inserted;

        for(int16_t i = 0; i < old_real_size; i++)
            old_grids[i] = grid_lines.x_m_p[i];

        /*reset grid lines number to grid_lines.size + 1
         * (so you are sure that current and last goal will be reset),
         * then allocate a proper size of grid lines and insert them
        */
        set_grids_number(grid_lines.size + 1);
        set_grids_number(old_real_size);

        for(int16_t i = 0; i < old_real_size; i++)
            set_grid(old_grids[i]);

        //delete old_grids
        free(old_grids);

        //send a message to QGC
        sprintf(txt_msg, "Reinserted previous grid lines.");
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

    if(current_grid_valid){
        *next_grid_p = current_grid_goal_x_m;
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

    //check if we have a valid grid line
    if(current_grid_valid){
        //see if we have reached or exceeded our goal
        if(cb_get_x_race_m() <= current_grid_goal_x_m){

            //Advise we have reached the current target grid line
            reached_current_grid();

            //take the new grid line, if any
            float tmp_grid;
            if(read_nex_grid(&tmp_grid)){
                // there is at least another grid line to reach, tack now
                cb_tack_now();
                //update the next goal
                current_grid_goal_x_m = tmp_grid;
                current_grid_valid = true;
            }
            else{
                //this was the last grid line to reach
                //TODO add here commands to sail downwind

                //no new grid line to reach
                current_grid_valid = false;
            }
        }
    }
}

