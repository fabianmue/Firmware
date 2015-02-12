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
#include "reference_actions.h"

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

//optimal path following data
static struct{
    bool following_traj; ///true if we're already following the optimal trajector
    float abs_alpha_star;///absolute value of Velocity Made Good
}following_path_planning = {
                            .following_traj = false,
                            .abs_alpha_star = 0.5f
                            };


static struct reference_actions_s ref_act = {.alpha_star = 0.5f, .should_tack = false};

static float current_grid_goal_x_m = 0.0f;//current x coordinate of grid line to reach [m]
static bool current_grid_valid = false;

static bool make_boat_tack = false; //true if the boat should tack now

// /** @brief distance function from an x-coordinate and a grid line x-coordinate*/
//float distance(float x1, float x2);

/** @brief read next grid line to reach*/
bool read_nex_grid(float *next_grid_p);

/** @brief advise that current goal grid line has been reached*/
void reached_current_grid(void);

/** @brief compute the y_index to acces to refernce action matrixes*/
int16_t compute_y_index(const struct local_position_race_s *local_pos_p);

/** @brief acces to optimal reference actions computed offline and take the next action to perform. */
void read_new_ref_action(struct structs_topics_s *strs_p,
                         const struct local_position_race_s *local_pos_p);

/** @boref based on actual local position, see which y discrete coordinate is closest.*/
int16_t compute_y_index(const struct local_position_race_s *local_pos_p);

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
 * Grid line number can be set by QGroundControl ONLY IF the boat is not
 * following the optimal path pre_computed offline
 *
 * @param size  total number of new grid lines
*/
void set_grids_number_qgc(int16_t size){
    if(following_path_planning.following_traj == false)
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
 * A Grid line can be set by QGroundControl ONLY IF the boat is not
 * following the optimal path pre_computed offline
 *
 * @param x_m       x coordinate [m] in Race frame of the new grid line
*/
void set_grid_qgc(float x_m){
    if(following_path_planning.following_traj == false)
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

    /*send a message to QGC to tell that a new grid line has been reached,
     * only if we are not following an optimal path
    */
    if(following_path_planning.following_traj == false){
        sprintf(txt_msg, "Grid line reached.");
        send_log_info(txt_msg);
    }
}

/**
 * Set new value for reference alpha, only possibile if the boat is not following optimal path
*/
void set_alpha_star(float val){

    if(following_path_planning.following_traj == false){
        ref_act.alpha_star = val;
    }
}

/**
 * Notify the end of the tack maneuver.
 * Only guidance_module should use this function when a tack action has been completed.
*/
void notify_tack_completed(void){
    ref_act.should_tack = false;
}


/**
 * Acces to optimal reference actions computed offline and take the next action to perform.
 *
 * See which is the closest discrete position (used in optimal path planning) to our actual position.
 * Read wind and haul data, and combined these data to the current grid line index
 * to acces to an appropriate matrix of optimal actions.
 * Based on the closest y-coordinate on the actual grid line, read the next action to perform.
 *
 * @param strs_p        struct with sensors information
 * @param local_pos_p   local position coordinate in Race fram
*/
void read_new_ref_action(struct structs_topics_s *strs_p,
                         const struct local_position_race_s *local_pos_p){

    float mean_wind_angle;  // mean wind angle set in QGC
    float twd;              //moving average of true wind measurements
    int8_t wind_index;
    int8_t haul_index;
    int8_t (*matrix_actions)[actions_row_number][actions_col_number]; //pointer to next action matrix
    int16_t number_current_line;
    int16_t y_index;
    int8_t next_action;

    //get mean wind angle set by QGC in navigation.h
    mean_wind_angle = get_mean_wind_angle();

    //get mean value of true wind direction read by weather station
    twd = get_twd_sns();

    //for now, only simple model (A) where only 2 wind directions are allowed
    /*
     * consider mean_wind_angle as a new "true" North, so we can
     * check if the mean true angle (twd) is NNW or NNE w.r.t. the mean_wind_angle,
     * that is, our new North.
    */

    //rotate twd considering mean_wind_angle as the direction of the new true North
    twd -= mean_wind_angle;

    //check if twd needs to be constrained between [-pi, pi]
    if(get_twd_sns() < 0 && twd < -M_PI_F)
        twd = 2 * M_PI_F + twd;
    else if(get_twd_sns() > 0 && twd > M_PI_F)
        twd = twd - 2 * M_PI_F;

    wind_index = (twd > 0) ? 2 : //wind from NNE w.r.t. our new "true" North
                             1;  //wind from NNW w.r.t. our new "true" North

    //TODO check if haul is ok as function of roll angle
//    haul_index = (strs_p->att.roll > 0) ? 1 : //port haul
//                                          2;  //starboard haul

    /*if ref_act.alpha_star is < 0, we were sailing on port haul, otherwise
     * we were sailing on starboard haul.
    */
    haul_index = (ref_act.alpha_star < 0) ? 1 : //port haul
                                            2;  //satrboard haul

    //take the appropriate matrix with optimal action for our current wind and haul states
    if(wind_index == 1 && haul_index == 1)
        matrix_actions = &actions_w1_h1;
    else if(wind_index == 1 && haul_index == 2)
        matrix_actions = &actions_w1_h2;
    else if(wind_index == 2 && haul_index == 1)
        matrix_actions = &actions_w2_h1;
    else if(wind_index == 2 && haul_index == 2)
        matrix_actions = &actions_w2_h2;

    //take the number of the current grid line we've just reached
    number_current_line = grid_lines.current_goal;
    //sanity check
    if(number_current_line < 0 || number_current_line >= total_grids_number)
        return;

    //compute y index (index of the closest discrete point in our grid line
    y_index = compute_y_index(local_pos_p);

    #if SIMULATION_FLAG == 1
    //cancella
    //strs_p->airspeed.true_airspeed_m_s = y_index;
    #endif

    //read next action to perform
    next_action = matrix_actions[0][number_current_line][y_index];

    /*
     * reference actions computed offline.
     * 3 = sail on starboard haul
     * 2 = sail on port haul
     * 1 = tack on the inner line
     * -1 = error in accessing matrix
    */

    //for now just use one alpha for model A, set by QCG
    switch(next_action){
    case 1:
        //tack against inner layline
        ref_act.should_tack = true;
        //change haul
        ref_act.alpha_star = -ref_act.alpha_star;

        //send a message to QGC
        sprintf(txt_msg, "Ref act: tack on inner layline.");
        send_log_info(txt_msg);
        break;

    case 2:
        //sail on port haul, check if a tack has to be performed
        //if we were sailing on starboard haul, before sailing on port haul we have to tack
        if(ref_act.alpha_star > 0){
            ref_act.should_tack = true;
        }
        //after tacking, sail on port haul
        ref_act.alpha_star = -following_path_planning.abs_alpha_star;

        //send a message to QGC
        sprintf(txt_msg, "Ref act: sail on port haul.");
        send_log_info(txt_msg);
        break;

    case 3:
        //sail on starboard haul, check if a tack has to be performed
        //if we were sailing on port haul, before sailing on starboard haul we have to tack
        if(ref_act.alpha_star < 0){
            ref_act.should_tack = true;
        }
        //after tacking, sail on starboard haul
        ref_act.alpha_star = following_path_planning.abs_alpha_star;

        //send a message to QGC
        sprintf(txt_msg, "Ref act: sail on starboard haul.");
        send_log_info(txt_msg);
        break;
    }
}

/**
 * Based on actual local position, see which y discrete coordinate is closest.
*/
int16_t compute_y_index(const struct local_position_race_s *local_pos_p){

    float division;
    int16_t y_index;
    int16_t y_max_val =  y_max[grid_lines.current_goal];

    //see how many times d_y (defined in reference_actions.c) is in local_pos_p->y_race_m
    division = local_pos_p->y_race_m / d_y;

    //test which discrete y-position on our grid line is closer

    //if y_race is greater (smaller) than our farthest y-discrete point on the right (left), take it as y-point
    if(division > y_max_val || division < - y_max_val){
        y_index = (division > 0) ?   2 * y_max_val :
                                     0;
    }
    else{
        //see which y-discrete-point is closer
        if(division > 0){
            if(division - (int32_t)division > 0.5f)
                y_index = y_max_val + (int32_t)division + 1;    //closer to the right point
            else
               y_index = y_max_val + (int32_t)division;         //closer to the left point
        }

        else{
            if(division - (int32_t)division < -0.5f)
                y_index = y_max_val + (int32_t)division - 1;    //closer to the left point
            else
               y_index = y_max_val + (int32_t)division;         //closer to the right point
        }
    }

    return y_index;
}

/**
 * Based on a new global position estimate, see if there is a new reference action to perform.
 *
 * Convert the global position coordinate in a coordinate in Race frame.
 * Check if we've passed a grid line, if so, see which is the next reference action
 * to pass to guidance_module.
 *
 * @param ref_act_p     pointer to struct which will contain next reference action to perform
 * @param strs_p        pointer to struct with data
*/
void path_planning(struct reference_actions_s *ref_act_p,
                   struct structs_topics_s *strs_p){

    struct local_position_race_s local_pos;
    float tmp;

    //convert geodedical coordinate into Race frame coordinate
    navigation_module(strs_p, &local_pos);

    //check if we are using grid lines to tell the boat where to tack
    //if the next grid line to reach is valid
    if(current_grid_valid){
        //see if we have reached or exceeded our goal
        if(local_pos.x_race_m <= current_grid_goal_x_m){

            //read optimal action computed offline, if QCG told us to use it
            if(following_path_planning.following_traj == true)
                read_new_ref_action(strs_p, &local_pos);
            else{
                ref_act.should_tack = true;
                //change alpha to change haul
                ref_act.alpha_star = -ref_act.alpha_star;
            }

            //Advise we have reached the current target grid line
            reached_current_grid();

            //take the new grid line, if any
            if(read_nex_grid(&tmp)){
                //there is at least another grid line to reach
                current_grid_goal_x_m = tmp;
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
    strs_p->boat_guidance_debug.next_grid_line = current_grid_goal_x_m;
    strs_p->boat_guidance_debug.x_race = local_pos.x_race_m;
    strs_p->boat_guidance_debug.y_race = local_pos.y_race_m;
    strs_p->boat_guidance_debug.alpha_star = ref_act_p->alpha_star;
    strs_p->boat_guidance_debug.should_tack = (ref_act_p->should_tack == true) ? 1 : 0;

}

/**
 * Start/stop following a pre-computed optimal path.
 *
 * @param start             1 if you wish to start following the pre-computed optimal path, 0 otherwise
 * @param abs_alpha_star    absolute value of true wind angle (alpha angle, in Dumas' thesis)
*/
void start_following_optimal_path(int32_t start, float abs_alpha_star){

    if(start > 0){
        //check if we're starting now
        if(following_path_planning.following_traj == false){
            //set up the total number of grid lines
            set_grids_number(total_grids_number);
            //we'are starting, add grid lines for optimal path computed offline
            for(int16_t i = total_grids_number; i > 0; i--){
                //the first grid line is the farthest from the top mark, etc..
                set_grid(i * d_x);
            }
            //remember that we've just started
            following_path_planning.following_traj = true;
            //set the absolute value of alpha star
            following_path_planning.abs_alpha_star = abs_alpha_star;

            //send a message to QGC
            sprintf(txt_msg, "Starting following optimal path.");
            send_log_info(txt_msg);
        }
        //else: nothing to do, we're already following optimal trajectory
    }
    else{
        //stop following optimal path
        if(following_path_planning.following_traj == true){
            //stop following optimal trajectory, delete all the grid lines inserted
            set_grids_number(1);

            //remember that we stopped following optimal trajecotry
            following_path_planning.following_traj = false;

            //send a message to QGC
            sprintf(txt_msg, "Stopping following optimal path.");
            send_log_info(txt_msg);
        }
        //else: nothing to do, we're not following any trajectory
    }
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
