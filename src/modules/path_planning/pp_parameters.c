/*
 * pp_parameters.c
 *
 * Handle changes in QGroundControl Parameters
 *
 *  Created on: 04.03.2015
 *      Author: Jonas Wirz <wirzjo@student.ethz.ch>
 */

#include "pp_parameters.h"

#include "pp_cost_method.h"

#include "pp_navigator.h"



/**
 * pp_cost_method: Weighting factors for Cost Function Method
 *
 * @min 0
 * @max 5
 */
PARAM_DEFINE_FLOAT(PP_CM_WEIGHT_GW, 0.9f);
PARAM_DEFINE_FLOAT(PP_CM_WEIGHT_GO, 0.8f);
PARAM_DEFINE_FLOAT(PP_CM_WEIGHT_GM, 0.4f);
PARAM_DEFINE_FLOAT(PP_CM_WEIGHT_GS, 0.05f);
PARAM_DEFINE_FLOAT(PP_CM_WEIGHT_GT, 0.1f);
PARAM_DEFINE_FLOAT(PP_CM_WEIGHT_GLEE, 0.15f);

/**
 * pp_cost_method: Saftey Radius around Obstacles
 *
 * @min 0
 */
PARAM_DEFINE_FLOAT(PP_CM_OBSTSAFETYRADIUS, 10.0f);

/**
 * pp_cost_method: Radius inside which obstacles are detected
 *
 * @min 0
 */
PARAM_DEFINE_FLOAT(PP_CM_OBSTHORIZON, 100.0f);

/**
 * pp_cost_method: Size of the window for smoothing the total cost
 *
 * @min 0
 */
PARAM_DEFINE_INT32(PP_CM_WINDOWSIZE, 5);


/**
 * pp_navigator: Time between two calls of Path Planning [us]
 *
 * @min 0
 */
PARAM_DEFINE_INT32(PP_NAV_PERIOD, 1000);





/** Struct holding the pointer to the QGround Control Variables */
static struct pointers_param_qgc_s{

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

} pointers_param_qgc;



/**
 * Init the Parameter Pointer at Startup
 *
 * @param: Struct containing the pointers
 */
void p_param_init(void) {

	//**COST_METHOD
	pointers_param_qgc.cm_weight_gw_pointer = param_find("PP_CM_WEIGHT_GW");
	pointers_param_qgc.cm_weight_go_pointer = param_find("PP_CM_WEIGHT_GO");
	pointers_param_qgc.cm_weight_gm_pointer = param_find("PP_CM_WEIGHT_GM");
	pointers_param_qgc.cm_weight_gs_pointer = param_find("PP_CM_WEIGHT_GS");
	pointers_param_qgc.cm_weight_gt_pointer = param_find("PP_CM_WEIGHT_GT");
	pointers_param_qgc.cm_weight_glee_pointer = param_find("PP_CM_WEIGHT_GLEE");

	pointers_param_qgc.cm_obstsafetyradius_pointer = param_find("PP_CM_OBSTACLESAFETYRADIUS");
	pointers_param_qgc.cm_obsthorizon_pointer = param_find("PP_CM_OBSTHORIZONT");
	pointers_param_qgc.cm_windowsize_pointer = param_find("PP_CM_WINDOWSIZE");


	//**NAVIGATION
	pointers_param_qgc.nav_period = param_find("PP_NAV_PERIOD");


}


/**
 * Update the Parameters
 *
 * @param: Struct containing the pointers
 */
void p_param_update(void) {

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
	uint32_t period;
	param_get(pointers_param_qgc.nav_period, &period);

	nav_set_configuration(period);

}


