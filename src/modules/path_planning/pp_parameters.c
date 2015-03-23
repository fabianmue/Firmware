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
#include <drivers/drv_hrt.h>
#include <stdio.h>

#include "pp_navigator.h"
#include "pp_cost_method.h"

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
 * Ray of the the grid line of index AS_P_INDEX, in meters.
 *
 */
PARAM_DEFINE_FLOAT(ASP_P_R_M, 0.0f);

/**
 * 1 if you want to add a new grid line at ray = ASP_P_R_M.
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

/**
 * How fast (deg/s) alpha_star should change after the boat has reached
 * the last grid line.
 *
 * @min 0
 */
PARAM_DEFINE_FLOAT(ASP_ALPST_V_DS, 12.0f);

/**
 * Alpha star value when sailing downwind after reached last grid line.
 *
 */
PARAM_DEFINE_FLOAT(ASP_DWN_ALPST_D, 160.0f);

#endif //USE_GRID_LINES == 1

#if SIMULATION_FLAG == 1

//---------------------------------------- Simulation variables --------------------
struct vehicle_global_position_s vgp;
int32_t lat_sim;
int32_t lon_sim;
int32_t alt_sim;

#if USE_GRID_LINES == 1
/**
 * Simulated Latitude, in degrees * E7.
 *
 *
 * @min -900000000
 * @max 900000000
 */
PARAM_DEFINE_INT32(ASPS_LAT_E7, 473494820);

/**
 * Simulated Longitude, in degrees * E7.
 *
 *
 * @min -1800000000
 * @max 1800000000
 */
PARAM_DEFINE_INT32(ASPS_LON_E7, 85605120);

/**
 * Simulated Altitude, in millimeters.
 *
 *
 * @min 0
 * @max ?
 */
PARAM_DEFINE_INT32(ASPS_ALT_E3, 406000);

#endif //USE_GRID_LINES == 1
#endif //SIMULATION_FLAG == 1




/**
* pp_cost_method: Weighting factors for Cost Function Method
*
* @min 0
* @max 5
*/
PARAM_DEFINE_FLOAT(PP_CM_W_GW, 0.9f);
PARAM_DEFINE_FLOAT(PP_CM_W_GO, 0.8f);
PARAM_DEFINE_FLOAT(PP_CM_W_GM, 0.4f);
PARAM_DEFINE_FLOAT(PP_CM_W_GS, 0.05f);
PARAM_DEFINE_FLOAT(PP_CM_W_GT, 0.1f);
PARAM_DEFINE_FLOAT(PP_CM_W_GLEE, 0.15f);


/**
 * pp_cost_method: Saftey Radius around Obstacles
 *
 * @min 0
 */
PARAM_DEFINE_FLOAT(PP_CM_OBSTSAFRAD, 10.0f);

/**
 * pp_cost_method: Radius inside which obstacles are detected
 *
 * @min 0
 */
PARAM_DEFINE_FLOAT(PP_CM_OBSTHORIZN, 100.0f);

/**
 * pp_cost_method: Size of the window for smoothing the total cost
 *
 * @min 0
 */
PARAM_DEFINE_INT32(PP_CM_WINDOWSIZE, 5);


/**
 * pp_navigator: Time between two calls of Path Planning [s]
 *
 * @min 0
 */
PARAM_DEFINE_INT32(PP_NAV_PERIOD, 1);


/**
 * pp_navigator: Maximum Turnrate of the boat [°/s]
 *
 * @min 0
 */
PARAM_DEFINE_INT32(PP_NAV_TURNRATE, 10);


/**
 * pp_navigator: Target Position (GEO-Coordinate Frame)
 */
PARAM_DEFINE_INT32(PP_NAV_TAR1_LAT, HOMELAT);
PARAM_DEFINE_INT32(PP_NAV_TAR1_LON, HOMELON);

PARAM_DEFINE_INT32(PP_NAV_TAR2_LAT, HOMELAT);
PARAM_DEFINE_INT32(PP_NAV_TAR2_LON, HOMELON);

PARAM_DEFINE_INT32(PP_NAV_TAR3_LAT, HOMELAT);
PARAM_DEFINE_INT32(PP_NAV_TAR3_LON, HOMELON);

PARAM_DEFINE_INT32(PP_NAV_TAR_NUM, 1);	//Number of Targets currently set


/**
 * pp_navigator: Target Position (GEO-Coordinate Frame)
 */
PARAM_DEFINE_INT32(PP_NAV_OBST1_LAT, HOMELAT);
PARAM_DEFINE_INT32(PP_NAV_OBST1_LON, HOMELON);

PARAM_DEFINE_INT32(PP_NAV_OBST2_LAT, HOMELAT);
PARAM_DEFINE_INT32(PP_NAV_OBST2_LON, HOMELON);

PARAM_DEFINE_INT32(PP_NAV_OBST3_LAT, HOMELAT);
PARAM_DEFINE_INT32(PP_NAV_OBST3_LON, HOMELON);

PARAM_DEFINE_INT32(PP_NAV_OBST_NUM, 1);	//Number of Obstacles currently set


/**
 * pp_navigator: Start-Line defined by two buoys
 */
PARAM_DEFINE_INT32(PP_NAV_STRT1_LAT, HOMELAT);
PARAM_DEFINE_INT32(PP_NAV_STRT1_LON, HOMELON);
PARAM_DEFINE_INT32(PP_NAV_STRT2_LAT, HOMELAT);
PARAM_DEFINE_INT32(PP_NAV_STRT2_LON, HOMELON);

PARAM_DEFINE_FLOAT(PP_NAV_ALTITUDE, HOMEALT);		//Note: The altitude value is in Millimeters

/**
 * pp_navigator: Simulate the position update for the boat
 */
#if P_DEBUG == 1
PARAM_DEFINE_FLOAT(SIM_NED_NORTHX, 0);	//Current position of the boat in NED-Frame
PARAM_DEFINE_FLOAT(SIM_NED_EASTY, 0);
PARAM_DEFINE_FLOAT(SIM_HEADING,0);			//Current heading of the boat in degrees (compass-frame)
#endif


/**
 * pp_navigator: Reset all variables to the initial state.
 */
PARAM_DEFINE_INT32(PP_NAV_RESET,0);


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
    param_t grid_ray_pointer;         /**< pointer to param ASP_P_R_M*/
    param_t grid_add_pointer;         /**< pointer to param ASP_P_ADD*/
    param_t repeat_past_grids_pointer;    /**< pointer to param ASP_REIN_GRS */
    param_t alpha_star_vel_pointer; /**< pointer to param ASP_ALPST_V_DS */
    #endif //USE_GRID_LINES == 1

    // --- explicit tak now command from QGC
    param_t do_maneuver_now;                   /**< pointer to param ASP_DO_MANEUV */
    // --- explicit alpha reference from QGC
    param_t alpha_star_pointer;         /**< pointer to param ASP_ALST_ANG_D*/
    param_t use_alpha_star_pointer;         /**< pointer to param ASP_ALST_SET*/
    param_t downwind_alpha_star_pointer; /**< pointer to ASP_DWN_ALPST_D */
    //-- simulation params

    #if SIMULATION_FLAG == 1

    #if USE_GRID_LINES == 1
    param_t lat_sim_pointer; /**< pointer to param ASPS_LAT_E7*/
    param_t lon_sim_pointer; /**< pointer to param ASPS_LON_E7*/
    param_t alt_sim_pointer; /**< pointer to param ASPS_ALt_E3*/
    #endif //USE_GRID_LINES == 1

    #endif //SIMULATION_FLAG == 1


	//**COST_METHOD
	param_t cm_weight_gw_pointer;
	param_t cm_weight_go_pointer;
	param_t cm_weight_gm_pointer;
	param_t cm_weight_gs_pointer;
	param_t cm_weight_gt_pointer;
	param_t cm_weight_glee_pointer;

	param_t cm_obstsafetyradius_pointer;
	param_t cm_obsthorizon_pointer;
	param_t cm_windowsize_pointer;


	//**NAVIGATION
	param_t nav_period;
	param_t nav_turnrate;
	param_t nav_target1_lat;
	param_t nav_target1_lon;
	param_t nav_target2_lat;
	param_t nav_target2_lon;
	param_t nav_target3_lat;
	param_t nav_target3_lon;
	param_t nav_target_number;
	param_t nav_obstacle1_lat;
	param_t nav_obstacle1_lon;
	param_t nav_obstacle2_lat;
	param_t nav_obstacle2_lon;
	param_t nav_obstacle3_lat;
	param_t nav_obstacle3_lon;
	param_t nav_obstacle_number;
	param_t nav_start1_lat;
	param_t nav_start1_lon;
	param_t nav_start2_lat;
	param_t nav_start2_lon;
	param_t nav_altitude;

	param_t nav_reset;


	//**SIMULATION FOR DEBUGGING
	#if P_DEBUG == 1
	param_t sim_ned_northx;
	param_t sim_ned_easty;
	param_t sim_heading;
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
    pointers_param_qgc.grid_ray_pointer    = param_find("ASP_P_R_M");

    pointers_param_qgc.grid_add_pointer = param_find("ASP_P_ADD");
    pointers_param_qgc.repeat_past_grids_pointer = param_find("ASP_REIN_GRS");

    pointers_param_qgc.alpha_star_vel_pointer = param_find("ASP_ALPST_V_DS");
    pointers_param_qgc.downwind_alpha_star_pointer = param_find("ASP_DWN_ALPST_D");
    #endif //USE_GRID_LINES == 1

    //explicit tack now command from QGC
    pointers_param_qgc.do_maneuver_now = param_find("ASP_DO_MANEUV");

    //explicit alpha star from QGC
    pointers_param_qgc.alpha_star_pointer    = param_find("ASP_ALST_ANG_D");
    pointers_param_qgc.use_alpha_star_pointer    = param_find("ASP_ALST_SET");

    #if SIMULATION_FLAG == 1

    #if USE_GRID_LINES == 1
    pointers_param_qgc.lat_sim_pointer = param_find("ASPS_LAT_E7");
    pointers_param_qgc.lon_sim_pointer = param_find("ASPS_LON_E7");
    pointers_param_qgc.alt_sim_pointer = param_find("ASPS_ALT_E3");
    #endif //USE_GRID_LINES == 1

    #endif //SIMULATION_FLAG == 1

    //clean boat_qgc_param2
    memset(&boat_qgc_param2, 0, sizeof(boat_qgc_param2));



    //**COST_METHOD
    pointers_param_qgc.cm_weight_gw_pointer = param_find("PP_CM_W_GW");
    pointers_param_qgc.cm_weight_go_pointer = param_find("PP_CM_W_GO");
    pointers_param_qgc.cm_weight_gm_pointer = param_find("PP_CM_W_GM");
    pointers_param_qgc.cm_weight_gs_pointer = param_find("PP_CM_W_GS");
    pointers_param_qgc.cm_weight_gt_pointer = param_find("PP_CM_W_GT");
    pointers_param_qgc.cm_weight_glee_pointer = param_find("PP_CM_W_GLEE");

    pointers_param_qgc.cm_obstsafetyradius_pointer = param_find("PP_CM_OBSTSAFRAD");
    pointers_param_qgc.cm_obsthorizon_pointer = param_find("PP_CM_OBSTHORIZN");
    pointers_param_qgc.cm_windowsize_pointer = param_find("PP_CM_WINDOWSIZE");


    //**NAVIGATION
    pointers_param_qgc.nav_period = param_find("PP_NAV_PERIOD");
    pointers_param_qgc.nav_turnrate = param_find("PP_NAV_TURNRATE");
    pointers_param_qgc.nav_target1_lat = param_find("PP_NAV_TAR1_LAT");
    pointers_param_qgc.nav_target1_lon = param_find("PP_NAV_TAR1_LON");
    pointers_param_qgc.nav_target2_lat = param_find("PP_NAV_TAR2_LAT");
    pointers_param_qgc.nav_target2_lon = param_find("PP_NAV_TAR2_LON");
    pointers_param_qgc.nav_target3_lat = param_find("PP_NAV_TAR3_LAT");
    pointers_param_qgc.nav_target3_lon = param_find("PP_NAV_TAR3_LON");
    pointers_param_qgc.nav_target_number = param_find("PP_NAV_TAR_NUM");
    pointers_param_qgc.nav_obstacle1_lat = param_find("PP_NAV_TAR1_LAT");
    pointers_param_qgc.nav_obstacle1_lon = param_find("PP_NAV_TAR1_LON");
    pointers_param_qgc.nav_obstacle2_lat = param_find("PP_NAV_TAR2_LAT");
    pointers_param_qgc.nav_obstacle2_lon = param_find("PP_NAV_TAR2_LON");
    pointers_param_qgc.nav_obstacle3_lat = param_find("PP_NAV_TAR3_LAT");
    pointers_param_qgc.nav_obstacle3_lon = param_find("PP_NAV_TAR3_LON");
    pointers_param_qgc.nav_obstacle_number = param_find("PP_NAV_OBST_NUM");

    pointers_param_qgc.nav_start1_lat = param_find("PP_NAV_STRT1_LAT");
    pointers_param_qgc.nav_start1_lon = param_find("PP_NAV_STRT1_LON");
    pointers_param_qgc.nav_start2_lat = param_find("PP_NAV_STRT2_LAT");
    pointers_param_qgc.nav_start2_lon = param_find("PP_NAV_STRT2_LON");
    pointers_param_qgc.nav_altitude = param_find("PP_NAV_ALTITUDE");

    pointers_param_qgc.nav_reset = param_find("PP_NAV_RESET");


    //**SIMULATION DEBUG
    #if P_DEBUG == 1
    pointers_param_qgc.sim_ned_northx = param_find("SIM_NED_NORTHX");
    pointers_param_qgc.sim_ned_easty = param_find("SIM_NED_EASTY");
    pointers_param_qgc.sim_heading = param_find("SIM_HEADING");
    #endif



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
    float grids_ray_m;
    param_get(pointers_param_qgc.grids_number_pointer, &grids_number);

    //x coordinate of current grid line
    param_get(pointers_param_qgc.grid_ray_pointer, &grids_ray_m);

    //check if we have to add a new grid line
    int32_t temp = 0;
    param_get(pointers_param_qgc.grid_add_pointer, &temp);
    if(temp > 0 && update_path_param){
        //set x coordinate of a new grid line
        gh_set_grid_qgc(grids_ray_m);
    }

    //set the new number of grid lines
    gh_set_grids_number_qgc(grids_number);

    param_get(pointers_param_qgc.repeat_past_grids_pointer, &temp);
    bool use_last_grids = (temp > 0) ? true : false;
    gh_reuse_last_grids(use_last_grids);

    //velocity of alpha star
    float alpha_star_vel_r;

    param_get(pointers_param_qgc.alpha_star_vel_pointer, &alpha_star_vel_r);

    //convert alpha_star_vel_r from deg/s to rad/s
    alpha_star_vel_r = alpha_star_vel_r * deg2rad;
    //make sure alpha_star_vel_r is positive
    alpha_star_vel_r = (alpha_star_vel_r < 0.0f) ? -alpha_star_vel_r : alpha_star_vel_r;
    //send alpha_star_vel_r to pp_communication_buffer module
    cb_set_alpha_star_vel(alpha_star_vel_r);

    //alpha_star in downwind course
    float downwind_alpha_star;

    param_get(pointers_param_qgc.downwind_alpha_star_pointer, &downwind_alpha_star);

    downwind_alpha_star = downwind_alpha_star * deg2rad;

    cb_set_downwind_alpha_star(downwind_alpha_star);

    #endif //USE_GRID_LINES == 1


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
    //do we have to tack?
    if(update_path_param == true && do_maneuver_now != 0){
        //if we can tack, do it now and send a message to QGC
        if(cb_tack_now())
            smq_send_log_info("Tack now.");
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
    //lat_sim
    param_get(pointers_param_qgc.lat_sim_pointer, &lat_sim);

    //lon_sim
    param_get(pointers_param_qgc.lon_sim_pointer, &lon_sim);

    //alt_sim
    param_get(pointers_param_qgc.alt_sim_pointer, &alt_sim);

    //set lat, lon and alt to gps_filtered struct to simulate
    vgp.lat = ((double)lat_sim) / 1e7;
    vgp.lon = ((double)lon_sim) / 1e7;
    vgp.alt = alt_sim / 1e3;

    //compute fake boat position in race frame using pp_navigation_module
    n_navigation_module(&vgp);
    gh_gridlines_handler();

    #endif //USE_GRID_LINES == 1

    #endif //SIMULATION_FLAG == 1


    //**COST METHOD
    	float gw, go, gm, gs, gt, glee, obstsafetyradius, obsthorizon, windowsize;
    	param_get(pointers_param_qgc.cm_weight_gw_pointer, &gw);
    	param_get(pointers_param_qgc.cm_weight_go_pointer, &go);
    	param_get(pointers_param_qgc.cm_weight_gm_pointer, &gm);
    	param_get(pointers_param_qgc.cm_weight_gs_pointer, &gs);
    	param_get(pointers_param_qgc.cm_weight_gt_pointer, &gt);
    	param_get(pointers_param_qgc.cm_weight_glee_pointer, &glee);

    	param_get(pointers_param_qgc.cm_obstsafetyradius_pointer, &obstsafetyradius);
    	param_get(pointers_param_qgc.cm_obsthorizon_pointer, &obsthorizon);
    	param_get(pointers_param_qgc.cm_windowsize_pointer, &windowsize);

    	cm_set_configuration(gw, go, gm, gs, gt, glee, obstsafetyradius, obsthorizon, windowsize);


    	//**NAVIGATOR
    	uint32_t period, turnrate;
    	param_get(pointers_param_qgc.nav_period, &period);
    	param_get(pointers_param_qgc.nav_turnrate, &turnrate);
    	nav_set_configuration(period, turnrate);

    	PointE7 target[MAXTARGETNUMBER];
    	PointE7 obstacle[MAXOBSTACLENUMBER];
    	uint8_t t_num, o_num;
    	int32_t altitude;

    	param_get(pointers_param_qgc.nav_altitude, &altitude);

    	param_get(pointers_param_qgc.nav_target1_lat,&(target[0].lat));
    	param_get(pointers_param_qgc.nav_target1_lon,&(target[0].lon));
    	param_get(pointers_param_qgc.nav_target2_lat,&(target[1].lat));
    	param_get(pointers_param_qgc.nav_target2_lon,&(target[1].lon));
    	param_get(pointers_param_qgc.nav_target3_lat,&(target[2].lat));
    	param_get(pointers_param_qgc.nav_target3_lon,&(target[2].lon));
    	param_get(pointers_param_qgc.nav_target_number,&t_num);
    	param_get(pointers_param_qgc.nav_obstacle1_lat,&(obstacle[0].lat));
    	param_get(pointers_param_qgc.nav_obstacle1_lon,&(obstacle[0].lon));
    	param_get(pointers_param_qgc.nav_obstacle2_lat,&(obstacle[1].lat));
    	param_get(pointers_param_qgc.nav_obstacle2_lon,&(obstacle[1].lon));
    	param_get(pointers_param_qgc.nav_obstacle3_lat,&(obstacle[2].lat));
    	param_get(pointers_param_qgc.nav_obstacle3_lon,&(obstacle[2].lon));
    	param_get(pointers_param_qgc.nav_obstacle_number,&o_num);

    	target[0].alt = altitude;
    	target[1].alt = altitude;
    	target[2].alt = altitude;
    	obstacle[0].alt = altitude;
    	obstacle[1].alt = altitude;
    	obstacle[2].alt = altitude;

    	uint8_t t;
    	for(t = 0; t < t_num; t++) {
    		nav_set_target(t,target[t]);
    	}

    	for(t = 0; t < o_num; t++) {
    		nav_set_obstacle(t,obstacle[t]);
    	}

    	PointE7 start[2];
    	param_get(pointers_param_qgc.nav_start1_lat, &(start[0].lat));
    	param_get(pointers_param_qgc.nav_start1_lon, &(start[0].lon));
    	param_get(pointers_param_qgc.nav_start2_lat, &(start[1].lat));
    	param_get(pointers_param_qgc.nav_start2_lon, &(start[1].lon));

    	start[0].alt = altitude;
    	start[1].alt = altitude;

    	nav_set_startline(start[0],start[1]);

    	//**RESET THE NAVIGTOR PARAMETERS
    	uint8_t reset;
    	param_get(pointers_param_qgc.nav_reset, &reset);
    	if(reset == 1) {
    		//Reset the parameters for the pathplanning to the default values.
    		nav_init();
    		smq_send_log_info("NAVIGATOR RESET! switch back to 0!");
    	}


    	//**SIMULATION DEBUG
    	#if P_DEBUG == 1
    	NEDpoint p;
    	float head;
    	param_get(pointers_param_qgc.sim_ned_northx,&(p.northx));
    	param_get(pointers_param_qgc.sim_ned_easty,&(p.easty));
    	param_get(pointers_param_qgc.sim_heading,&head);

    	DEBUG_nav_set_fake_state(p, head);
    	#endif

}
