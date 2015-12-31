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
#include <string.h>

#include "pp_navigator.h"
#include "pp_cost_method.h"
#include "pp_potentialfield_method.h"

#include "kalman_tracker/kt_tracker.h"
#include "kalman_tracker/kt_track_list.h"
#include "kalman_tracker/kt_cog_list.h"

#include "parser_sensorboard/ps_sensorboard.h"

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



#if LDEBUG_FAKEPOSITION==1
/**
 * Simulated Latitude, in degrees * E7.
 *
 *
 * @min -900000000
 * @max 900000000
 */
PARAM_DEFINE_FLOAT(ASIM_NORTH, 0);

/**
 * Simulated Longitude, in degrees * E7.
 *
 *
 * @min -1800000000
 * @max 1800000000
 */
PARAM_DEFINE_FLOAT(ASIM_EAST, 0);

#endif




/**
 * pp_potentialfield_method: Weighting factors and other
 * configuration parameters for potentialfield method
 */
PARAM_DEFINE_FLOAT(PP_PM_W_GT, 0.7f);
PARAM_DEFINE_FLOAT(PP_PM_W_GO, 0.4f);
PARAM_DEFINE_FLOAT(PP_PM_W_GW, 1.0f);
PARAM_DEFINE_FLOAT(PP_PM_W_GM, 0.3f);
PARAM_DEFINE_FLOAT(PP_PM_W_SDIST, 20.0f);



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
PARAM_DEFINE_FLOAT(PP_CM_W_GSEN, 0.5f);


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
PARAM_DEFINE_FLOAT(PP_NAV_PERIOD, 1);


/**
 * pp_navigator: Maximum Turnrate of the boat [°/s]
 *
 * @min 0
 */
PARAM_DEFINE_INT32(PP_NAV_TURNRATE, 10);

/*
PARAM_DEFINE_INT32(PP_NAV_STRT1_LAT, HOMELAT);
PARAM_DEFINE_INT32(PP_NAV_STRT1_LON, HOMELON);
PARAM_DEFINE_INT32(PP_NAV_STRT2_LAT, HOMELAT);
PARAM_DEFINE_INT32(PP_NAV_STRT2_LON, HOMELON);
 */

PARAM_DEFINE_FLOAT(PP_NAV_ALTITUDE, HOMEALT);		//Note: The altitude value is in Millimeters

/**
 * pp_navigator: Simulate the position update for the boat
 */
PARAM_DEFINE_FLOAT(SIM_NED_NORTHX, 0);		//Current position of the boat in NED-Frame
PARAM_DEFINE_FLOAT(SIM_NED_EASTY, 0);
PARAM_DEFINE_FLOAT(SIM_HEADING,0);			//Current heading of the boat in degrees (compass-frame)


/**
 * pp_navigator: Reset all variables to the initial state.
 */
PARAM_DEFINE_INT32(PP_NAV_RESET,0);

/**
 * pp_apathp_on
 * Enable the use of the Pathplanner
 */
PARAM_DEFINE_INT32(PP_APATHP_ON,0);

/**
 * pp_nav_meth: Set the method used for Pathplanning
 * Note: The selectable methods are described in navigator.c (struct Config)
 */
PARAM_DEFINE_INT32(PP_NAV_METH,1);


/**
 * pp_nav_useyaw: Uses the unfiltered yaw as the heading update for the pathplanning
 */
PARAM_DEFINE_INT32(PP_NAV_USEYAW,0);


/**
 * Do no gybe maneuvers. When doing a gybe simply change the reference heading
 */
PARAM_DEFINE_INT32(PP_DBG_NOGYBE,0);



/**
 * pp_nav_nodist
 * Do not include a distance to target dependency in Target-Cost
 */
PARAM_DEFINE_INT32(PP_DBG_NODIST,0);


/* Invert alpha star before sending it to the Helsman */
PARAM_DEFINE_INT32(PP_DBG_INVALP,0);


/**
 * pp_nav_setar
 * Set the current position as the next target position. As soon as it is switched to
 * autonomous mode the pathplanner guides the boat towards this target
 */
PARAM_DEFINE_INT32(PP_NAV_SETAR,0);


/**
 * kt_enable
 * Enable the Kalman Tracker and therefore use the Tracked Obstacles in the Pathplanning
 */
PARAM_DEFINE_INT32(KT_A_ENABLE,0);

/**
 * kt_...
 * Parameters for the Kalman Tracker
 */
PARAM_DEFINE_FLOAT(KT_SIGMA,2.0f);
PARAM_DEFINE_INT32(KT_UNSEEN,2);
PARAM_DEFINE_FLOAT(KT_NNSF_THRESH,100);
PARAM_DEFINE_FLOAT(KT_CO,10);
PARAM_DEFINE_FLOAT(KT_PERIOD,2);

/**
 * mi_...
 * Parameters for the mission
 */
PARAM_DEFINE_INT32(MI_ID, 0);
PARAM_DEFINE_FLOAT(MI_TAR_LAT, 0.0f);
PARAM_DEFINE_FLOAT(MI_TAR_LON, 0.0f);
PARAM_DEFINE_INT32(MI_TAR_NUM, 0);
PARAM_DEFINE_FLOAT(MI_OBS_LAT, 0.0f);
PARAM_DEFINE_FLOAT(MI_OBS_LON, 0.0f);
PARAM_DEFINE_INT32(MI_OBS_NUM, 0);

static struct pointers_pp_param_qgc_s {


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



#if LDEBUG_FAKEPOSITION==1
    param_t north_sim_pointer; /**< pointer to param ASIM_LAT_E7*/
    param_t east_sim_pointer; /**< pointer to param ASIM_LON_E7*/
#endif


	//**COST_METHOD
	param_t cm_weight_gw_pointer;
	param_t cm_weight_go_pointer;
	param_t cm_weight_gm_pointer;
	param_t cm_weight_gs_pointer;
	param_t cm_weight_gt_pointer;
	param_t cm_weight_glee_pointer;
	param_t cm_weight_gsensor_pointer;

	param_t cm_obstsafetyradius_pointer;
	param_t cm_obsthorizon_pointer;
	param_t cm_windowsize_pointer;

	param_t cm_dbg_nodist;


	//**NAVIGATION
	param_t nav_period;
	param_t nav_turnrate;

	param_t nav_altitude;

	param_t nav_reset;
	param_t nav_pathp_on;
	param_t nav_meth;
	param_t nav_setar;
	param_t nav_useyaw;

	param_t nav_dbg_invalp;

	param_t nav_dbg_nogybe;

	// SIMULATION FOR DEBUGGING
	param_t sim_ned_northx;
	param_t sim_ned_easty;
	param_t sim_heading;


	// POTENTIALFIELD METHOD
	param_t pm_weight_gt_pointer;
	param_t pm_weight_go_pointer;
	param_t pm_weight_gw_pointer;
	param_t pm_weight_gm_pointer;
	param_t pm_weight_sdist_pointer;

	// KALMAN OBSTACLE TRACKER
	param_t kt_enable;
	param_t kt_sigma;
	param_t kt_unseen;
	param_t kt_nnsf_thresh;
	param_t kt_co;
	param_t kt_period;

	// MISSION
	param_t mi_id;
	param_t mi_tar_lat;
	param_t mi_tar_lon;
	param_t mi_tar_num;
	param_t mi_obs_lat;
	param_t mi_obs_lon;
	param_t mi_obs_num;

} pointers_pp_param_qgc;

/**
* Initialize parameters.
*
*/
void pp_param_QGC_init(void){

    // initialize pointer to parameters
	pointers_pp_param_qgc.lat0_pointer    = param_find("ASP_R_LAT0_E7");
	pointers_pp_param_qgc.lon0_pointer    = param_find("ASP_R_LON0_E7");
	pointers_pp_param_qgc.alt0_pointer    = param_find("ASP_R_ALT0_E3");

	pointers_pp_param_qgc.mean_wind_pointer = param_find("ASP_MEAN_WIND_D");

	pointers_pp_param_qgc.lat_tmark_pointer    = param_find("ASP_T_LAT_E7");
	pointers_pp_param_qgc.lon_tmark_pointer    = param_find("ASP_T_LON_E7");
	pointers_pp_param_qgc.alt_tmark_pointer    = param_find("ASP_T_ALT_E3");

    // --- grid lines system parameters
    #if USE_GRID_LINES == 1
		pointers_pp_param_qgc.grids_number_pointer    = param_find("ASP_P_TOT_G");
		pointers_pp_param_qgc.grid_ray_pointer    = param_find("ASP_P_R_M");

		pointers_pp_param_qgc.grid_add_pointer = param_find("ASP_P_ADD");
		pointers_pp_param_qgc.repeat_past_grids_pointer = param_find("ASP_REIN_GRS");

		pointers_pp_param_qgc.alpha_star_vel_pointer = param_find("ASP_ALPST_V_DS");
		pointers_pp_param_qgc.downwind_alpha_star_pointer = param_find("ASP_DWN_ALPST_D");
    #endif //USE_GRID_LINES == 1

    // explicit tack now command from QGC
    pointers_pp_param_qgc.do_maneuver_now = param_find("ASP_DO_MANEUV");

    // explicit alpha star from QGC
    pointers_pp_param_qgc.alpha_star_pointer    = param_find("ASP_ALST_ANG_D");
    pointers_pp_param_qgc.use_alpha_star_pointer    = param_find("ASP_ALST_SET");

    #if SIMULATION_FLAG == 1

    #if USE_GRID_LINES == 1
		pointers_pp_param_qgc.lat_sim_pointer = param_find("ASPS_LAT_E7");
		pointers_pp_param_qgc.lon_sim_pointer = param_find("ASPS_LON_E7");
		pointers_pp_param_qgc.alt_sim_pointer = param_find("ASPS_ALT_E3");
    #endif //USE_GRID_LINES == 1

    #endif //SIMULATION_FLAG == 1



	#if LDEBUG_FAKEPOSITION==1
		pointers_pp_param_qgc.north_sim_pointer = param_find("ASIM_NORTH");
		pointers_pp_param_qgc.east_sim_pointer = param_find("ASIM_EAST");
	#endif

    //clean boat_qgc_param2
    memset(&boat_qgc_param2, 0, sizeof(boat_qgc_param2));



    // COST_METHOD
    pointers_pp_param_qgc.cm_weight_gw_pointer = param_find("PP_CM_W_GW");
    pointers_pp_param_qgc.cm_weight_go_pointer = param_find("PP_CM_W_GO");
    pointers_pp_param_qgc.cm_weight_gm_pointer = param_find("PP_CM_W_GM");
    pointers_pp_param_qgc.cm_weight_gs_pointer = param_find("PP_CM_W_GS");
    pointers_pp_param_qgc.cm_weight_gt_pointer = param_find("PP_CM_W_GT");
    pointers_pp_param_qgc.cm_weight_glee_pointer = param_find("PP_CM_W_GLEE");
    pointers_pp_param_qgc.cm_weight_gsensor_pointer = param_find("PP_CM_W_GSEN");

    pointers_pp_param_qgc.cm_obstsafetyradius_pointer = param_find("PP_CM_OBSTSAFRAD");
    pointers_pp_param_qgc.cm_obsthorizon_pointer = param_find("PP_CM_OBSTHORIZN");
    pointers_pp_param_qgc.cm_windowsize_pointer = param_find("PP_CM_WINDOWSIZE");

    pointers_pp_param_qgc.cm_dbg_nodist = param_find("PP_DBG_NODIST");

    // NAVIGATION
    pointers_pp_param_qgc.nav_period = param_find("PP_NAV_PERIOD");
    pointers_pp_param_qgc.nav_turnrate = param_find("PP_NAV_TURNRATE");

    pointers_pp_param_qgc.nav_altitude = param_find("PP_NAV_ALTITUDE");

    pointers_pp_param_qgc.nav_reset = param_find("PP_NAV_RESET");
    pointers_pp_param_qgc.nav_useyaw = param_find("PP_NAV_USEYAW");

    pointers_pp_param_qgc.nav_dbg_invalp = param_find("PP_DBG_INVALP");
    pointers_pp_param_qgc.nav_dbg_nogybe = param_find("PP_DBG_NOGYBE");

    // SIMULATION DEBUG
    pointers_pp_param_qgc.sim_ned_northx = param_find("SIM_NED_NORTHX");
    pointers_pp_param_qgc.sim_ned_easty = param_find("SIM_NED_EASTY");
    pointers_pp_param_qgc.sim_heading = param_find("SIM_HEADING");

    // ENABLE PATHPLANNER
    pointers_pp_param_qgc.nav_pathp_on = param_find("PP_APATHP_ON");
    pointers_pp_param_qgc.nav_meth = param_find("PP_NAV_METH");
    pointers_pp_param_qgc.nav_setar = param_find("PP_NAV_SETAR");


    // POTENTIALFIELD METHOD
    pointers_pp_param_qgc.pm_weight_gt_pointer = param_find("PP_PM_W_GT");
    pointers_pp_param_qgc.pm_weight_go_pointer = param_find("PP_PM_W_GO");
    pointers_pp_param_qgc.pm_weight_gw_pointer = param_find("PP_PM_W_GW");
    pointers_pp_param_qgc.pm_weight_gm_pointer = param_find("PP_PM_W_GM");
    pointers_pp_param_qgc.pm_weight_sdist_pointer = param_find("PP_PM_W_SDIST");


	// KALMAN OBSTACLE TRACKER
    pointers_pp_param_qgc.kt_enable = param_find("KT_A_ENABLE");
    pointers_pp_param_qgc.kt_sigma = param_find("KT_SIGMA");
    pointers_pp_param_qgc.kt_unseen = param_find("KT_UNSEEN");
    pointers_pp_param_qgc.kt_nnsf_thresh = param_find("KT_NNSF_THRESH");
    pointers_pp_param_qgc.kt_co = param_find("KT_CO");
    pointers_pp_param_qgc.kt_period = param_find("KT_PERIOD");

	// MISSION
    pointers_pp_param_qgc.mi_id = param_find("MI_ID");
    pointers_pp_param_qgc.mi_tar_lat = param_find("MI_TAR_LAT");
    pointers_pp_param_qgc.mi_tar_lon = param_find("MI_TAR_LON");
    pointers_pp_param_qgc.mi_tar_num = param_find("MI_TAR_NUM");
    pointers_pp_param_qgc.mi_obs_lat = param_find("MI_OBS_LAT");
    pointers_pp_param_qgc.mi_obs_lon = param_find("MI_OBS_LON");
    pointers_pp_param_qgc.mi_obs_num = param_find("MI_OBS_NUM");

    //get parameters but do not add any grid lines at start up
    pp_param_QGC_get(false);
}

/** Update local copy of parameters.
 *
*/
void pp_param_QGC_get(bool update_path_param){

    //----- reference geo coordinate
    int32_t lat0;
    int32_t lon0;
    int32_t alt0;
    //lat0
    param_get(pointers_pp_param_qgc.lat0_pointer, &lat0);

    //lon0
    param_get(pointers_pp_param_qgc.lon0_pointer, &lon0);

    //alt0
    param_get(pointers_pp_param_qgc.alt0_pointer, &alt0);

    //update NED origin in pp_navigation_module
    n_set_ref0(&lat0, &lon0, &alt0);

    //----- mean wind
    float mean_wind;
    param_get(pointers_pp_param_qgc.mean_wind_pointer, &mean_wind);

    //convert mean_wind in rad
    mean_wind = mean_wind * deg2rad;

    //set mean wind angle in pp_navigation_module
    n_set_mean_wind_angle(mean_wind);

    //----- top mark geo coordinate
    int32_t lat_tmark;
    int32_t lon_tmark;
    int32_t alt_tmark;
    //lat_tmark
    param_get(pointers_pp_param_qgc.lat_tmark_pointer, &lat_tmark);

    //lon_tmark
    param_get(pointers_pp_param_qgc.lon_tmark_pointer, &lon_tmark);

    //alt_tmark
    param_get(pointers_pp_param_qgc.alt_tmark_pointer, &alt_tmark);

    //set top mark position in pp_navigation_module
    n_set_pos_top_mark(&lat_tmark, &lon_tmark, &alt_tmark);

    // --- grid lines system parameters
    #if USE_GRID_LINES == 1
    //----- number of grids
    int32_t grids_number;
    float grids_ray_m;
    param_get(pointers_pp_param_qgc.grids_number_pointer, &grids_number);

    //x coordinate of current grid line
    param_get(pointers_pp_param_qgc.grid_ray_pointer, &grids_ray_m);

    //check if we have to add a new grid line
    int32_t temp = 0;
    param_get(pointers_pp_param_qgc.grid_add_pointer, &temp);
    if(temp > 0 && update_path_param){
        //set x coordinate of a new grid line
        gh_set_grid_qgc(grids_ray_m);
    }

    //set the new number of grid lines
    gh_set_grids_number_qgc(grids_number);

    param_get(pointers_pp_param_qgc.repeat_past_grids_pointer, &temp);
    bool use_last_grids = (temp > 0) ? true : false;
    gh_reuse_last_grids(use_last_grids);

    //velocity of alpha star
    float alpha_star_vel_r;

    param_get(pointers_pp_param_qgc.alpha_star_vel_pointer, &alpha_star_vel_r);

    //convert alpha_star_vel_r from deg/s to rad/s
    alpha_star_vel_r = alpha_star_vel_r * deg2rad;
    //make sure alpha_star_vel_r is positive
    alpha_star_vel_r = (alpha_star_vel_r < 0.0f) ? -alpha_star_vel_r : alpha_star_vel_r;
    //send alpha_star_vel_r to pp_communication_buffer module
    cb_set_alpha_star_vel(alpha_star_vel_r);

    //alpha_star in downwind course
    float downwind_alpha_star;

    param_get(pointers_pp_param_qgc.downwind_alpha_star_pointer, &downwind_alpha_star);

    downwind_alpha_star = downwind_alpha_star * deg2rad;

    cb_set_downwind_alpha_star(downwind_alpha_star);

    #endif //USE_GRID_LINES == 1


    //----- explicit tack now command and alpha star value from QGC
    float alpha_tmp;
    int32_t set_alpha;
    int32_t do_maneuver_now;

    //take values
    param_get(pointers_pp_param_qgc.do_maneuver_now, &do_maneuver_now);
    param_get(pointers_pp_param_qgc.alpha_star_pointer, &alpha_tmp);
    param_get(pointers_pp_param_qgc.use_alpha_star_pointer, &set_alpha);

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
    pp_th_publish_qgc2(&boat_qgc_param2);

    #if SIMULATION_FLAG == 1

    #if USE_GRID_LINES == 1
    //----- simulation coordinates
    //lat_sim
    param_get(pointers_pp_param_qgc.lat_sim_pointer, &lat_sim);

    //lon_sim
    param_get(pointers_pp_param_qgc.lon_sim_pointer, &lon_sim);

    //alt_sim
    param_get(pointers_pp_param_qgc.alt_sim_pointer, &alt_sim);

    //set lat, lon and alt to gps_filtered struct to simulate
    vgp.lat = ((double)lat_sim) / 1e7;
    vgp.lon = ((double)lon_sim) / 1e7;
    vgp.alt = alt_sim / 1e3;

    //compute fake boat position in race frame using pp_navigation_module
    n_navigation_module(&vgp);
    gh_gridlines_handler();

    #endif //USE_GRID_LINES == 1

    #endif //SIMULATION_FLAG == 1


    //**SIMULATE GPS-Position


#if LDEBUG_FAKEPOSITION==1
    float north_sim;
    float east_sim;

    param_get(pointers_pp_param_qgc.north_sim_pointer, &north_sim);
    param_get(pointers_pp_param_qgc.east_sim_pointer, &east_sim);

    DEBUG_fakened(north_sim, east_sim);
#endif




    //**COST METHOD
    float gw, go, gm, gs, gt, glee, gsensor, obstsafetyradius, obsthorizon, windowsize;
    param_get(pointers_pp_param_qgc.cm_weight_gw_pointer, &gw);
    param_get(pointers_pp_param_qgc.cm_weight_go_pointer, &go);
    param_get(pointers_pp_param_qgc.cm_weight_gm_pointer, &gm);
    param_get(pointers_pp_param_qgc.cm_weight_gs_pointer, &gs);
    param_get(pointers_pp_param_qgc.cm_weight_gt_pointer, &gt);
    param_get(pointers_pp_param_qgc.cm_weight_glee_pointer, &glee);
    param_get(pointers_pp_param_qgc.cm_weight_gsensor_pointer, &gsensor);

    param_get(pointers_pp_param_qgc.cm_obstsafetyradius_pointer, &obstsafetyradius);
    param_get(pointers_pp_param_qgc.cm_obsthorizon_pointer, &obsthorizon);
    param_get(pointers_pp_param_qgc.cm_windowsize_pointer, &windowsize);

    cm_set_configuration(gw, go, gm, gs, gt, glee, gsensor, obstsafetyradius, obsthorizon, windowsize);

    uint8_t dbg_nodist = 0;
    param_get(pointers_pp_param_qgc.cm_dbg_nodist, &dbg_nodist);
    DEBUG_set_minus(dbg_nodist);


    //**NAVIGATOR
    float period, turnrate;
    param_get(pointers_pp_param_qgc.nav_period, &period);
    param_get(pointers_pp_param_qgc.nav_turnrate, &turnrate);
    nav_set_configuration(period, turnrate);

   	//**RESET THE NAVIGTOR PARAMETERS
   	uint8_t reset = 0;
   	param_get(pointers_pp_param_qgc.nav_reset, &reset);
   	if(reset == 1) {
   		//Reset the parameters for the pathplanning to the default values.
   		nav_reset();
   		smq_send_log_info("NAVIGATOR RESET! switch back to 0!");
   	}

   	//**INVERT ALPHA BEFORE SENDING TO AUTONOMOUS SAILING APP
   	uint8_t dbg_invalp = 0;
   	param_get(pointers_pp_param_qgc.nav_dbg_invalp, &dbg_invalp);
   	DEBUG_nav_alpha_minus(dbg_invalp);


   	//**Do not command gybes on downwind courses => simply change reference
   	uint8_t dbg_nogybe = 0;
   	param_get(pointers_pp_param_qgc.nav_dbg_nogybe, &dbg_nogybe);
   	nav_set_nogybe(dbg_nogybe);



   	//**ENABLE THE USE OF THE NAVIGATOR
   	uint8_t pathp_on = 0;
   	param_get(pointers_pp_param_qgc.nav_pathp_on, &pathp_on);
   	nav_enable_navigator(pathp_on);

   	//**SET METHOD USED FOR PATHPLANNING
   	uint8_t pathp_meth = 1;
   	param_get(pointers_pp_param_qgc.nav_meth, &pathp_meth);
   	nav_set_method(pathp_meth);

   	//**USE THE YAW ONLY FOR HEADING
   	uint8_t pathp_useyaw = 0;
   	param_get(pointers_pp_param_qgc.nav_useyaw, &pathp_useyaw);
   	nav_set_use_yaw(pathp_useyaw);


   	//**SET THE CURRENT POSITION OF THE BOAT AS THE NEXT TARGET POSITION
   	uint8_t pathp_settar = 0;
   	param_get(pointers_pp_param_qgc.nav_setar, &pathp_settar);
   	if(pathp_settar == 1) {
		#if LDEBUG_USEMISSION == 0
   		nav_set_quick_target();
		#endif
   		smq_send_log_info("Set quick Target Position! switch back to 0!");
   	}


   	//**SIMULATION DEBUG
   	NEDpoint p;
   	float head;
   	param_get(pointers_pp_param_qgc.sim_ned_northx,&(p.northx));
   	param_get(pointers_pp_param_qgc.sim_ned_easty,&(p.easty));
   	param_get(pointers_pp_param_qgc.sim_heading,&head);

   	DEBUG_nav_set_fake_state(p, head);


   	//**SET CONFIGURATION FOR POTENTIALFIELD METHOD
   	float Gt, Go, Gm, Gw, SearchDist;
   	param_get(pointers_pp_param_qgc.pm_weight_gt_pointer, &Gt);
   	param_get(pointers_pp_param_qgc.pm_weight_go_pointer, &Go);
   	param_get(pointers_pp_param_qgc.pm_weight_gw_pointer, &Gm);
   	param_get(pointers_pp_param_qgc.pm_weight_gm_pointer, &Gw);
   	param_get(pointers_pp_param_qgc.pm_weight_sdist_pointer, &SearchDist);
   	pm_set_configuration(Gt, Go, Gm, Gw, SearchDist);


   	//**KALMAN OBSTACLE TRACKER
   	uint8_t kt_status = 0;
   	param_get(pointers_pp_param_qgc.kt_enable, &kt_status);
   	kt_enable(kt_status);

   	uint8_t kt_unseen = 0;
   	float kt_sigma = 0;
   	param_get(pointers_pp_param_qgc.kt_sigma, &kt_sigma);
   	param_get(pointers_pp_param_qgc.kt_unseen, &kt_unseen);
   	tl_set_configuration(kt_sigma, kt_unseen);

   	float kt_nnsf_thresh = 0;
   	param_get(pointers_pp_param_qgc.kt_nnsf_thresh, &kt_nnsf_thresh);
   	cl_set_configuration(kt_nnsf_thresh);

   	float kt_co = 0;
   	param_get(pointers_pp_param_qgc.kt_co, &kt_co);
   	kt_set_configuration(kt_co);

   	float kt_period = 0;
   	param_get(pointers_pp_param_qgc.kt_period, &kt_period);
   	sb_set_configuration(kt_period);
}
