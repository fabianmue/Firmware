/*
 * pp_parameters.c
 *
 * Handle changes in QGroundControl Parameters
 *
 *  Created on: 04.03.2015
 *      Author: Jonas Wirz <wirzjo@student.ethz.ch>
 *      Author: Marco Tranzatto <marco.tranzatto@gmail.com>
 */

#include "pp_parameters.h"

#define M_PI_F 3.14159265358979323846f

static const float deg2rad = 0.0174532925199433f; // pi / 180

static struct boat_qgc_param2_s boat_qgc_param2;

/**
 * Reference angle with respect to the wind, in degrees.
 *
 * Use Dumas'convention.
 *
 * @min -180
 * @max 180
 */
PARAM_DEFINE_FLOAT(ASP_ALST_ANG_D, 45.0f);

/**
 * If ASP_ALST_SET == 1, the value of ASP_ALST_ANG_D will be used
 * as the new reference angle.
 *
 * @min 0
 * @max 1
 */
PARAM_DEFINE_INT32(ASP_ALST_SET, 1);

/**
 * Swtich this value from 0 to 1 if you want to tack/jybe.
 * Then rememeber to switch it to 0 after the maneuver is completed.
 * Attention: if ASP_DO_MANEUV switches from 0 to 1, the parameter
 * ASP_ALST_ANG_D will NOT be considered!
*/
PARAM_DEFINE_INT32(ASP_DO_MANEUV, 0);

// --- coordinates variables

/**
 * Latitude of the origin of the NED system, in degrees * E7.
 *
 * @min -900000000
 * @max 900000000
 */
PARAM_DEFINE_INT32(ASP_R_LAT0_E7, 473494820);

/**
 * Longitude of the origin of the NED system, in degrees * E7.
 *
 * @min -1800000000
 * @max 1800000000
 */
PARAM_DEFINE_INT32(ASP_R_LON0_E7, 85605120);

/**
 * Altitude of origin of NED system, in millimeters.
 *
 * @min 0
 * @max ?
 */
PARAM_DEFINE_INT32(ASP_R_ALT0_E3, 406000);

/**
 * Latitude of the top mark, in degrees * E7.
 *
 * @min -900000000
 * @max 900000000
 */
PARAM_DEFINE_INT32(ASP_T_LAT_E7, 473459370);

/**
 * Longitude of the top mark, in degrees * E7.
 *
 * @min -1800000000
 * @max 1800000000
 */
PARAM_DEFINE_INT32(ASP_T_LON_E7, 85547940);

/**
 * Altitude of the top mark, in millimeters.
 *
 * @min 0
 * @max ?
 */
PARAM_DEFINE_INT32(ASP_T_ALT_E3, 406000);

/**
 * ASP_MEAN_WIND_D, specifies the mean wind direction [deg], in [-180, 180].
 * Positive on the right (going from North to East),
 * negative on the left (going from North to West).
 *
 * @min -180
 * @max 180
 */
PARAM_DEFINE_FLOAT(ASP_MEAN_WIND_D, 0.0f);

//------------------------- grid lines parameters
#if USE_GRID_LINES == 1

/**
 * Total numbers of grid lines.
 *
 * @min 1
 * @max ?
 */
PARAM_DEFINE_INT32(ASP_P_TOT_G, 1);

/**
 * X coordinate in Race frame of grid line of index AS_P_INDEX, in meters.
 *
 */
PARAM_DEFINE_FLOAT(ASP_P_X_M, 0.0f);

/**
 * 1 if you want to add a new grid line at x = ASC_P_X_M.
 *
 * @min 0
 * @max 1
 */
PARAM_DEFINE_INT32(ASP_P_ADD, 0);

/**
 * 1 if you want to re-insert the same grid lines you used before.
 *
 * @min 0
 * @max 1
 */
PARAM_DEFINE_INT32(ASP_REIN_GRS, 0);

#endif

#if SIMULATION_FLAG == 1

//---------------------------------------- Simulation variables --------------------

#if USE_GRID_LINES == 1
/**
 * Simulated Latitude, in degrees * E7.
 *
 *
 * @min -900000000
 * @max 900000000
 */
PARAM_DEFINE_INT32(ASIM_LAT_E7, 473494820);

/**
 * Simulated Longitude, in degrees * E7.
 *
 *
 * @min -1800000000
 * @max 1800000000
 */
PARAM_DEFINE_INT32(ASIM_LON_E7, 85605120);

/**
 * Simulated Altitude, in millimeters.
 *
 *
 * @min 0
 * @max ?
 */
PARAM_DEFINE_INT32(ASIM_ALT_E3, 406000);

#endif

/**
 * Simulated Course over ground, in deg, sign opposite to Dumas convention.
 *
 *
 * @min -180
 * @max 180
 */
PARAM_DEFINE_FLOAT(ASIM_COG_D, 0.0f);

/**
 * Simulated true wind direction, in deg, sign opposite to Dumas convention.
 *
 *
 * @min -180
 * @max 180
 */
PARAM_DEFINE_FLOAT(ASIM_TWD_D, 0.0f);

/**
 * Simulated yaw, in deg, sign opposite to Dumas convention.
 *
 *
 * @min -180
 * @max 180
 */
PARAM_DEFINE_FLOAT(ASIM_YAW_D, 0.0f);

/**
 * Temporary value for debug porpuses.
 */
PARAM_DEFINE_FLOAT(ASIM_DEVA1, 0.0f);

#endif


static struct pointers_param_qgc_s{


    param_t lat0_pointer;         /**< pointer to param ASP_R_LAT0_E7*/
    param_t lon0_pointer;         /**< pointer to param ASP_R_LON0_E7*/
    param_t alt0_pointer;         /**< pointer to param ASP_R_ALT0_E3*/

    param_t mean_wind_pointer;/**< pointer to param ASP_MEAN_WIND_D*/

    param_t lat_tmark_pointer;         /**< pointer to param ASP_T_LAT_E7*/
    param_t lon_tmark_pointer;         /**< pointer to param ASP_T_LON_E7*/
    param_t alt_tmark_pointer;         /**< pointer to param ASP_T_ALT_E3*/

    // --- grid lines system parameters
    #if USE_GRID_LINES == 1
    param_t grids_number_pointer;         /**< pointer to param ASP_P_TOT_G*/
    param_t grid_x_pointer;         /**< pointer to param ASP_P_X_M*/
    param_t grid_add_pointer;         /**< pointer to param ASP_P_ADD*/
    param_t repeat_past_grids_pointer;    /**< pointer to param ASP_REIN_GRS */
    #endif

    // --- explicit tak now command from QGC
    param_t do_maneuver_now;                   /**< pointer to param ASP_DO_MANEUV */
    // --- explicit alpha reference from QGC
    param_t alpha_star_pointer;         /**< pointer to param ASP_ALST_ANG_D*/
    param_t use_alpha_star_pointer;         /**< pointer to param ASP_ALST_SET*/

    //-- simulation params

    #if SIMULATION_FLAG == 1

    #if USE_GRID_LINES == 1
    param_t lat_sim_pointer; /**< pointer to param ASIM_LAT_E7*/
    param_t lon_sim_pointer; /**< pointer to param ASIM_LON_E7*/
    param_t alt_sim_pointer; /**< pointer to param ASIM_ALt_E3*/
    #endif

    param_t twd_sim_pointer; /**< pointer to param ASIM_TWD_D*/
    param_t cog_sim_pointer; /**< pointer to param ASIM_COG_D*/


    param_t yaw_sim_pointer; /**< pointer to param ASIM_YAW_D*/
    param_t deva1_sim_pointer; /**< pointer to param ASIM_DEVA1 */
    #endif
}pointers_param_qgc;


/**
* Initialize parameters.
*
*/
void p_param_init(void){

    //initialize pointer to parameters
    pointers_param_qgc.lat0_pointer    = param_find("ASP_R_LAT0_E7");
    pointers_param_qgc.lon0_pointer    = param_find("ASP_R_LON0_E7");
    pointers_param_qgc.alt0_pointer    = param_find("ASP_R_ALT0_E3");

    pointers_param_qgc.mean_wind_pointer = param_find("ASP_MEAN_WIND_D");

    pointers_param_qgc.lat_tmark_pointer    = param_find("ASP_T_LAT_E7");
    pointers_param_qgc.lon_tmark_pointer    = param_find("ASP_T_LON_E7");
    pointers_param_qgc.alt_tmark_pointer    = param_find("ASP_T_ALT_E3");

    // --- grid lines system parameters
    #if USE_GRID_LINES == 1
    pointers_param_qgc.grids_number_pointer    = param_find("ASP_P_TOT_G");
    pointers_param_qgc.grid_x_pointer    = param_find("ASP_P_X_M");

    pointers_param_qgc.grid_add_pointer = param_find("ASP_P_ADD");
    pointers_param_qgc.repeat_past_grids_pointer = param_find("ASP_REIN_GRS");
    #endif

    //explicit tack now command from QGC
    pointers_param_qgc.do_maneuver_now = param_find("ASP_DO_MANEUV");

    //explicit alpha star from QGC
    pointers_param_qgc.alpha_star_pointer    = param_find("ASP_ALST_ANG_D");
    pointers_param_qgc.use_alpha_star_pointer    = param_find("ASP_ALST_SET");

    #if SIMULATION_FLAG == 1

    #if USE_GRID_LINES == 1
    pointers_param_qgc.lat_sim_pointer = param_find("ASIM_LAT_E7");
    pointers_param_qgc.lon_sim_pointer = param_find("ASIM_LON_E7");
    pointers_param_qgc.alt_sim_pointer = param_find("ASIM_ALT_E3");
    #endif

    pointers_param_qgc.cog_sim_pointer = param_find("ASIM_COG_D");
    pointers_param_qgc.twd_sim_pointer = param_find("ASIM_TWD_D");

    pointers_param_qgc.yaw_sim_pointer = param_find("ASIM_YAW_D");
    pointers_param_qgc.deva1_sim_pointer = param_find("ASIM_DEVA1");

    #endif

    //clean boat_qgc_param2
    memset(&boat_qgc_param2, 0, sizeof(boat_qgc_param2));

    //get parameters but do not add any grid lines at start up
    p_param_update(false);
}

/** Update local copy of parameters.
 *
*/
void p_param_update(bool update_path_param){

    //----- reference geo coordinate
    int32_t lat0;
    int32_t lon0;
    int32_t alt0;
    //lat0
    param_get(pointers_param_qgc.lat0_pointer, &lat0);

    //lon0
    param_get(pointers_param_qgc.lon0_pointer, &lon0);

    //alt0
    param_get(pointers_param_qgc.alt0_pointer, &alt0);

    //update NED origin in pp_navigation_module
    n_set_ref0(&lat0, &lon0, &alt0);

    //----- mean wind
    float mean_wind;
    param_get(pointers_param_qgc.mean_wind_pointer, &mean_wind);

    //convert mean_wind in rad
    mean_wind = mean_wind * deg2rad;

    //set mean wind angle in pp_navigation_module
    n_set_mean_wind_angle(mean_wind);

    //----- top mark geo coordinate
    int32_t lat_tmark;
    int32_t lon_tmark;
    int32_t alt_tmark;
    //lat_tmark
    param_get(pointers_param_qgc.lat_tmark_pointer, &lat_tmark);

    //lon_tmark
    param_get(pointers_param_qgc.lon_tmark_pointer, &lon_tmark);

    //alt_tmark
    param_get(pointers_param_qgc.alt_tmark_pointer, &alt_tmark);

    //set top mark position in pp_navigation_module
    n_set_pos_top_mark(&lat_tmark, &lon_tmark, &alt_tmark);

    // --- grid lines system parameters
    #if USE_GRID_LINES == 1
    //----- number of grids
    int32_t grids_number;
    float grids_x_m;
    param_get(pointers_param_qgc.grids_number_pointer, &grids_number);

    //x coordinate of current grid line
    param_get(pointers_param_qgc.grid_x_pointer, &grids_x_m);

    //check if we have to add a new grid line
    int32_t temp = 0;
    param_get(pointers_param_qgc.grid_add_pointer, &temp);
    if(temp > 0 && update_path_param){
        //set x coordinate of a new grid line
        //set_grid_qgc(grids_x_m);
    }

    //set the new number of grid lines
    //set_grids_number_qgc(grids_number);

    param_get(pointers_param_qgc.repeat_past_grids_pointer, &temp);
    //bool use_last_grids = (temp > 0) ? true : false;
    //reuse_last_grids(use_last_grids);

    #endif


    //----- explicit tack now command and alpha star value from QGC
    float alpha_tmp;
    int32_t set_alpha;
    int32_t do_maneuver_now;

    //take values
    param_get(pointers_param_qgc.do_maneuver_now, &do_maneuver_now);
    param_get(pointers_param_qgc.alpha_star_pointer, &alpha_tmp);
    param_get(pointers_param_qgc.use_alpha_star_pointer, &set_alpha);

    //convert alpha from deg to rad
    alpha_tmp = alpha_tmp * deg2rad;

    //safety check: we must have abs(alpha) <= pi
    if(alpha_tmp > M_PI_F)
        alpha_tmp = M_PI_F;
    else if(alpha_tmp < -M_PI_F)
        alpha_tmp = -M_PI_F;

    //set alpha_tmp as the new alpha star ONLY if set_alpha is not 0 AND we do not have to tack/jybe
    if(set_alpha != 0 && do_maneuver_now == 0){
        cb_set_alpha_star(alpha_tmp);
    }
    //do we have to tack or jybe?
    if(update_path_param == true && do_maneuver_now != 0){
        //do maneuver and simply change the sign of alpha_star
        cb_do_maneuver(-cb_get_alpha_star());
    }

    //save interested param in boat_qgc_param and publish this topic
    //qgc2
    boat_qgc_param2.timestamp = hrt_absolute_time();

    boat_qgc_param2.lat0 = lat0;
    boat_qgc_param2.lon0 = lon0;
    boat_qgc_param2.alt0 = alt0;
    boat_qgc_param2.latT = lat_tmark;
    boat_qgc_param2.lonT = lon_tmark;
    boat_qgc_param2.altT = alt_tmark;
    boat_qgc_param2.mean_wind_direction_r = mean_wind;

    //publish topic
    th_publish_qgc2(&boat_qgc_param2);

    #if SIMULATION_FLAG == 1

    #if USE_GRID_LINES == 1
    //----- simulation coordinates
    int32_t lat_sim;
    int32_t lon_sim;
    int32_t alt_sim;
    //lat_sim
    param_get(pointers_param_qgc.lat_sim_pointer, &lat_sim);

    //lon_sim
    param_get(pointers_param_qgc.lon_sim_pointer, &lon_sim);

    //alt_sim
    param_get(pointers_param_qgc.alt_sim_pointer, &alt_sim);

    //set lat, lon and alt to gps_filtered struct to simulate
    strs_p->gps_filtered.lat = ((double)lat_sim) / 1e7;
    strs_p->gps_filtered.lon = ((double)lon_sim) / 1e7;
    strs_p->gps_filtered.alt = alt_sim / 1e3;
    #endif

    //cog_sim
    param_get(pointers_param_qgc.cog_sim_pointer, &(params_p->cog_sim));
    //convert cog in rad
    params_p->cog_sim = params_p->cog_sim * deg2rad;

    //twd_sim
    param_get(pointers_param_qgc.twd_sim_pointer, &(params_p->twd_sim));
    //convert twd_sim in rad
    params_p->twd_sim = params_p->twd_sim * deg2rad;

    //yaw_sim
    param_get(pointers_param_qgc.yaw_sim_pointer, &(params_p->yaw_sim));
    //convert yaw_sim in rad
    params_p->yaw_sim = params_p->yaw_sim * deg2rad;

    //set them in the appropriate struct to simulate heading changing
    strs_p->att.yaw = params_p->yaw_sim;

    //deva1
    param_get(pointers_param_qgc.deva1_sim_pointer, &(params_p->deva1));

    #endif
}
