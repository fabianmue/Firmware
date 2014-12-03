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
#include "reference_actions.c"

#ifndef NULL
    #define NULL 0
#endif

#define M_PI_F 3.14159265358979323846f

//grid lines data
static struct{
    float *x_m_p;           ///array of x coordinates [m] of grid lines, in Race frame
    int16_t size;           ///size of array x_m_p
    int16_t current_goal;   ///index of current grid line to reach
    int16_t last_goal;      ///index of the last grid line to reach
}grid_lines;

//optimal path following data
static struct{
    bool following_traj; ///true if we're already following the optimal trajector
};

static struct reference_actions_s ref_act = {.alpha_star = 0.5f, .should_tack = false};

static float current_grid_goal_x_m = 0.0f;//current x coordinate of grid line to reach [m]
static bool current_grid_valid = false;

// /** @brief distance function from an x-coordinate and a grid line x-coordinate*/
//float distance(float x1, float x2);

/** @brief read next grid line to reach*/
bool read_nex_grid(float *next_grid_p);

/** @brief advise that current goal grid line has been reached*/
void reached_current_grid(void);

/** @brief compute the y_index to acces to refernce action matrixes*/
int16_t compute_y_index(const struct local_position_race_s *local_pos_p);

// /** @brief get the number of the current grid line to reach (0 to grids_number) s*/
//int16_t get_grid_number(void);

/**
 * Initialize the grid lines struct. Delete all the old grid lines (if any).
*/
void init_grids(void){

    grid_lines.x_m_p = NULL;
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

}

/**
 * Set the x coordinate in Race frame of a grid line.
 * Grid lines are inserted with a FIFO policy.
 *
 * @param x_m       x coordinate [m] in Race frame of the new grid line
*/
void set_grid(float x_m){

    if(grid_lines.last_goal >= grid_lines.size)
        return; //not enough space to add a new grid line

    //enough space, add the new grid line
    grid_lines.last_goal++;
    grid_lines.x_m_p[grid_lines.last_goal] = x_m;

    //if this is the first grid line inserted, it's the current goal grid line
    if(grid_lines.last_goal == 0){
        grid_lines.current_goal = grid_lines.last_goal;
        current_grid_goal_x_m = x_m;
        current_grid_valid = true;
    }

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
}

/**
 * Set new value for reference alpha
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


//int16_t get_grid_number(void){
//    return grid_lines.current_goal;
//}

void read_new_ref_action(struct reference_actions_s *ref_act_p,
                         const struct structs_topics_s *strs_p,
                         const struct local_position_race_s *local_pos_p){

    float mean_wind_angle;  // mean wind angle set in QGC
    float twd;              //moving average of true wind measurements
    int8_t wind_index;
    int8_t haul_index;
    int8_t **matrix_actions;
    int16_t number_current_line;
    int16_t y_index;
    int8_t next_action;

    //get mean wind angle set by QGC in navigation.h
    mean_wind_angle = get_mean_wind_angle();

    //get mean value of true wind direction read by weather station
    twd = get_twd();

    //for now, only simple model (A) where only 2 wind directions are allowed
    /*
     * consider mean_wind_angle as a new "true" North, so we can
     * check if the mean true angle is NNW or NNE w.r.t. the mean_wind_angle,
     * that is, our new North.
    */

    //rotate twd considering mean_wind_angle as the direction of the new true north
    twd -= mean_wind_angle;

    //check if twd needs to be constrained between [-pi, pi]
    if(get_twd() < 0 && twd < -M_PI_F)
        twd = 2 * M_PI_F + twd;
    else if(get_twd() > 0 && twd > M_PI_F)
        twd = twd - 2 * M_PI_F;

    wind_index = (twd > 0) ? 2 : //wind from NNE w.r.t. our new "true" North
                             1;  //wind from NNW w.r.t. our new "true" North

    //TODO check if haul is ok as function of roll angle
    haul_index = (strs_p->att.roll > 0) ? 1 : //port haul
                                          2;  //starboard haul

    //tacke the appropriate matrix with optimal action for our current wind and haul states
    if(wind_index == 1 && haul_index == 1)
        matrix_actions = actions_w1_h1;
    else if(wind_index == 1 && haul_index == 2)
        matrix_actions = actions_w1_h2;
    else if(wind_index == 2 && haul_index == 1)
        matrix_actions = actions_w2_h1;
    else if(wind_index == 2 && haul_index == 2)
        matrix_actions = actions_w2_h2;

    //take number of current grid line to reach
    number_current_line = grid_lines.current_goal;
    //sanity check
    if(number_current_line < 0 || number_current_line >= grids_number)
        return;

    //compute y index (index of the closest discrete point in our grid line)
    y_index = compute_y_index(local_pos_p);


    //read next action to perform
    next_action = matrix_actions[number_current_line][y_index];

    /*
     * reference actions computed offline.
     * 1 = sail on starboard haul
     * 2 = sail on port haul
     * 3 = tack on the inner line
     * -1 = error in accessing matrix
    */

    //for now just use alpha_star set by QCG, TODO see which alpha has to be used
    switch(next_action){
    case 1:
        ref_act_p->alpha_star = ref_act.alpha_star;//TODO alpha_star from QGC HAS TO BE POSITIVE
        break;

    case 2:
        ref_act_p->alpha_star = -ref_act.alpha_star;//TODO alpha_star from QGC HAS TO BE POSITIVE
        break;
    case 3:
        ref_act_p->alpha_star = -ref_act_p->alpha_star; //after tack change haul
        ref_act_p->should_tack = true;
        break;
    }
}

int16_t compute_y_index(const struct local_position_race_s *local_pos_p){

    float division;
    float y_abs;
    int16_t y_index;
    bool y_race_positive = true;
    int16_t y_max_val =  y_max[grid_lines.current_goal];


    if(local_pos_p->y_race_m >= 0){
        y_abs = local_pos_p->y_race_m;
        y_race_positive = true;
    }
    else{
        y_abs = -local_pos_p->y_race_m;
        y_race_positive = false;
    }

    //see how many times d_y (defined in reference_actions.c) is in y_abs
    division = y_abs / d_y;

    //test which discrete y-position on our grid line is closer

    //if y_race is greater (smaller) than our farthest y-discrete point on the right (left), take it as y-point
    if(division > y_max_val)
        y_index = (y_race_positive) ?   2 * y_max_val :
                                        0;
    else{
        //see which y-discrete-point is closer
        if((division - ((int32_t)division) ) > 0.5f){
            if(y_race_positive)
                y_index = y_max_val + (int32_t)division + 1;//closer to the right point
            else
                y_index = y_max_val - (int32_t)division - 1;//closer to the left point
        }
        else{
            if(y_race_positive)
                y_index = y_max_val  + (int32_t)division;//closer to left point
            else
                y_index = y_max_val - (int32_t)division;//closer to the right point
        }
    }


    return y_index;
}

void path_planning(struct reference_actions_s *ref_act_p,
                   struct structs_topics_s *strs_p){

    struct local_position_race_s local_pos;
    float tmp;

    //convert geodedical coordinate into Race frame coordinate
    navigation_module(strs_p, &local_pos);

    //if the next grid line to reach is valid
    if(current_grid_valid){
        //see if we have reached or exceeded our goal
        if(local_pos.x_race_m <= current_grid_goal_x_m){

            //read optimal action computed offline
            read_new_ref_action(ref_act_p, strs_p, &local_pos);

            //Advise we have reached the current target grid line
            reached_current_grid();

            //tacke the new grid line, if any
            if(read_nex_grid(&tmp)){
                //there is at least another grid line to reach
                current_grid_goal_x_m = tmp;
                current_grid_valid = true;
            }
            else //no new grid line to reach
                current_grid_valid = false;

            //TODO see appropriate action
            //for now just tack
            //ref_act.should_tack = true;//ripristina
        }
    }

    //copy local reference action to the output struct
    memcpy(ref_act_p, &ref_act, sizeof(ref_act));

    //save second debug values for post-processing, other values set in guidance_module()
    strs_p->boat_guidance_debug.next_grid_line = current_grid_goal_x_m;
    strs_p->boat_guidance_debug.x_race = local_pos.x_race_m;
    strs_p->boat_guidance_debug.y_race = local_pos.y_race_m;
    strs_p->boat_guidance_debug.alpha_star = ref_act_p->alpha_star;
    strs_p->boat_guidance_debug.should_tack = (ref_act_p->should_tack == true) ? 1 : 0;

}
