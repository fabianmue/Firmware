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

#include "path_planning.h"

#ifndef NULL
    #define NULL 0
#endif

#define M_PI_F 3.14159265358979323846f

static char txt_msg[250]; ///used to send messages to QGC

//grid lines data
static struct{
    float *x_m_p;           ///array of x coordinates [m] of grid lines, in Race frame
    int16_t size;           ///size of array x_m_p
    int16_t current_goal;   ///index of current grid line to reach
    int16_t last_goal;      ///index of the last grid line to reach
    int16_t grids_inserted; /// number of grid lines inserted
}grid_lines;


static struct reference_actions_s ref_act = {.alpha_star = 0.5f, .should_tack = false};

static float current_grid_goal_x_m = 0.0f;//current x coordinate of grid line to reach [m]
static bool current_grid_valid = false;

static bool make_boat_tack = false; //true if the boat should tack now

/** @brief read next grid line to reach*/
bool read_nex_grid(float *next_grid_p);

/** @brief advise that current goal grid line has been reached*/
void reached_current_grid(void);

/** @brief set number of grid lines*/
void set_grids_number(int16_t size);

/** @brief set the x coordinate of a grid line*/
void set_grid(float x_m);

/**
 * Initialize the grid lines struct. Delete all the old grid lines (if any).
*/
void init_grids(void){

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
    send_log_info(txt_msg);

}

/**
 * Set the new number of total grid lines. Calling this function you will delete
 * all the previous grid lines. Next grid line to be reach is that one with index = 0
 * Grid line number can be set by QGroundControl.
 *
 * @param size  total number of new grid lines
*/
void set_grids_number_qgc(int16_t size){
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
        send_log_info(txt_msg);

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
    send_log_info(txt_msg);
}

/**
 * Set the x coordinate in Race frame of a grid line.
 * Grid lines are inserted with a FIFO policy.
 *
 * @param x_m       x coordinate [m] in Race frame of the new grid line
*/
void set_grid_qgc(float x_m){
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
    send_log_info(txt_msg);
}

/**
 * Set new value for reference alpha.
*/
void set_alpha_star(float val){

    ref_act.alpha_star = val;
}

/**
 * Notify the end of the tack maneuver.
 * Only guidance_module should use this function when a tack action has been completed.
*/
void notify_tack_completed(void){
    ref_act.should_tack = false;
}


void reuse_last_grids(bool use){
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
        send_log_info(txt_msg);
    }
}

/**
 * Specify if the boat should tack as soon as possibile.
 *
 * This function can be used to make the boat tack if you wish not to use
 * grid lines.
 * If the boat should tack now, the value of alpha star will be changed
 * auotnomously by the path_planning module.
 *
 * @param tack_now: true if the boat should tack now, false otherwise.
*/
void boat_should_tack(int32_t tack_now){
    //update make_boat_tack only if we are not taking
    if(ref_act.should_tack == false)
        make_boat_tack = (tack_now == 0) ? false : true;
}


/**
 * Based on a new global position estimate, see if there is a new reference action to perform.
 *
 * If we are using the grid lines to specify where a boat should tack:
 * convert the global position coordinate in a coordinate in Race frame.
 * Check if we've passed a grid line, if so, see which is the next reference action
 * to pass to guidance_module.
 *
 * In any case, look if the user told us to tack from QGC.
 *
 * @param ref_act_p     pointer to struct which will contain next reference action to perform
 * @param strs_p        pointer to struct with data
*/
void path_planning(struct reference_actions_s *ref_act_p,
                   struct structs_topics_s *strs_p){

    struct local_position_race_s local_pos;
    #if USE_GRID_LINES == 1
    //convert geodedical coordinate into Race frame coordinate
    navigation_module(strs_p, &local_pos);

    #endif

    //check if we are using grid lines to tell the boat where to tack
    //if the next grid line to reach is valid
    if(current_grid_valid){
        //see if we have reached or exceeded our goal
        if(local_pos.x_race_m <= current_grid_goal_x_m){

            //reached grid line, tack now!
            ref_act.should_tack = true;
            //change alpha to change haul
            ref_act.alpha_star = -ref_act.alpha_star;

            //Advise we have reached the current target grid line
            reached_current_grid();

            //take the new grid line, if any
            float tmp_grid;
            if(read_nex_grid(&tmp_grid)){
                //there is at least another grid line to reach
                current_grid_goal_x_m = tmp_grid;
                current_grid_valid = true;
            }
            else{
                //no new grid line to reach
                current_grid_valid = false;
            }
        }
    }
    else{
        /* if we are not using grind lines, check if the function
         * boat_should_tack told us to tack as soon as possibile.*/
        if(make_boat_tack){
            //tack now!
            ref_act.should_tack = true;
            //change alpha to change haul
            ref_act.alpha_star = -ref_act.alpha_star;
            //set make_boat_tack to flase
            make_boat_tack = false;
            //send a message to QGC
            sprintf(txt_msg, "Tacking now.");
            send_log_info(txt_msg);
        }
    }

    //copy local reference action to the output struct
    memcpy(ref_act_p, &ref_act, sizeof(ref_act));

    //save second debug values for post-processing, other values set in guidance_module()
    #if USE_GRID_LINES == 1
    strs_p->boat_guidance_debug.next_grid_line = current_grid_goal_x_m;
    strs_p->boat_guidance_debug.x_race = local_pos.x_race_m;
    strs_p->boat_guidance_debug.y_race = local_pos.y_race_m;
    #endif
    strs_p->boat_guidance_debug.alpha_star = ref_act_p->alpha_star;
    strs_p->boat_guidance_debug.should_tack = (ref_act_p->should_tack == true) ? 1 : 0;

}

