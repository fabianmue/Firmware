/**
 * @file pp_potentialfield_method.c
 *
 * Implementation of the potential field method
 *
 * @author Jonas Wirz <wirzjo@student.ethz.ch>
 */

#include <math.h>
#include "pp_config.h"
#include "pp_potentialfield_method.h"
#include "pp_navigation_helper.h"
#include "pp_polardiagram.h"


struct {
	float G_target;       //Weighting factor for getting against target 0.7
	float G_obstacle;     //Weighting factor for avoiding obstacles 0.4
	float G_wind;         //Weighting factor for avoiding the non-sailing-sector 1
	float G_tack;         //Weighting factor for avoiding tacks 0.3

	float search_dist;    //Distance wrt. the boat, where the probe-charge is placed [m]
	float upwind_dir; 	  //Maximum angle the boat can go upwind
} Config = {
		.G_target = 0.7f,
		.G_obstacle = 0.4f,
		.G_wind = 1.0f,
		.G_tack = 0.3f,

		.search_dist = 20.0f,
		.upwind_dir = 0.785398163397f //<=> 45°
};


#define HEADRESOLUTION 0.0872664625997f 	//Resolution for simulating the headings in [rad] (here 5°)
#define HEADRANGE	   1.74532925199f   	//Range for simulating the headings in [rad] (here [-100°...100°] wrt. boat-heading)


/** @brief Calculate the total potential at a given probe-placement */
float total_potential(NEDpoint probe, float seg, struct nav_field_s *field, struct nav_state_s *state);

/** @brief Calcualte the Target Potential */
float target_potential(NEDpoint probe, struct nav_field_s *field, struct nav_state_s *state);

/** @brief Calcuate the Wind Potential */
float wind_potential(float seg, struct nav_state_s *state);

/** @brief Calcualte the Obstacle Potential */
float obstacle_potential(NEDpoint probe, struct nav_field_s *field);

/** @brief Calculate the gaussian distance between the current position and an obstacle*/
float gaussian_ned_distance(NEDpoint obst_pos, NEDpoint pos);


/**
 * Main Potentialfield Pathplanning Function
 * This Function returns the optimal heading according to the potentialfield method and the
 * according parameters.
 *
 * @param Pointer to the State-Struct
 * @param Pointer to the Field-Struct
 */
float pm_NewHeadingReference(struct nav_state_s *state, struct nav_field_s *field) {

	/*Iterate over the possible "probe" headings
	 * Note: In order to reduce computational costs only a range specified by the variables
	 * HEADRANGE and HEADRESOLUTION is used as possible headings. */
	float seg_start = (state->heading_cur-HEADRANGE);
	float seg_end = (state->heading_cur+HEADRANGE);

	uint8_t probeNum = 2*HEADRANGE/HEADRESOLUTION+1;	//Number of Probe-Headings
	float potMat[(int)(probeNum)];
	float headMat[(int)(probeNum)];

	uint8_t ind = 0;	//Index for addressing costMat-Elements

	float seg;
	for(seg = seg_start; seg <= seg_end; seg += HEADRESOLUTION) {
		float seg_compass = fmod(seg,(2*PI));

		//Get the cost and save it in the matrix
		NEDpoint probe;
		probe.northx = cosf(seg_compass) * Config.search_dist + state->position.northx;
	    probe.easty = sinf(seg_compass) * Config.search_dist + state->position.easty;

	    //Calcualte the total Potential at the given probe-Point
	    potMat[ind] = total_potential(probe, seg_compass, field, state);
	    headMat[ind] = seg_compass;

		//Update Index
		ind++;
	}

	//Find minimum Index
	uint8_t minIndex = nh_findMin(potMat,probeNum);

	//Get corresponding minimum Heading
	float optHeading = headMat[minIndex];

	return optHeading;
}


/**
 * Calcualte the total potential at a given Test-Point
 */
float total_potential(NEDpoint probe, float seg, struct nav_field_s *field, struct nav_state_s *state) {

	//** Target Potential
	float target_p = target_potential(probe, field, state);


	//** Wind Potential
	float wind_p = wind_potential(seg, state);


	//** Obstacle Potential
	float obst_p = obstacle_potential(probe, field);


	return target_p + wind_p + obst_p;
}


/**
 * Calculate the target Potential
 */
float target_potential(NEDpoint probe, struct nav_field_s *field, struct nav_state_s *state) {
	return Config.G_target * nh_ned_dist(probe, field->targets[state->targetNum]);
}



/**
 * Calculate the wind Potential
 */
float wind_potential(float seg, struct nav_state_s *state) {

	float alpha_app = nh_appWindDir(seg,state->wind_dir);

	float p_wind = Config.G_wind/pol_polardiagram(alpha_app,state->wind_speed);


	//Punish Tacks
	float p_maneuver = 0;
	if(fabsf(seg-state->heading_cur) >= (2.0f*Config.upwind_dir)) {
		p_maneuver = Config.G_tack;
	}

	return (p_maneuver + Config.G_wind * p_wind);
}


/**
 * Calculate the obstacle Potential
 */
float obstacle_potential(NEDpoint probe, struct nav_field_s *field) {

	float p_obst_tot = 0;

	//Loop over all Obstacles
	uint8_t i;
	for (i = 0; i < field->NumberOfObstacles; i++) {

	    //Take Gaussian distance
	    float obst_dist = gaussian_ned_distance(field->obstacles[i],probe);
	    p_obst_tot = p_obst_tot + Config.G_obstacle * obst_dist;
	}

	 return p_obst_tot;
}


/**
 * Modulate the distance to obstacles as a Gaussian
 * The distance from a Point to an obstacle is modulated as a Gaussian.
 * Therefore the size of the obstacle can be modulated.
 */
float gaussian_ned_distance(NEDpoint obst_pos, NEDpoint pos) {

	//Set Parameters
	float A = 150;     //Height of Peak
	float dx = 30;     //Width of the Peak in x-Direction
	float dy = 30;     //Width of the Peak in x-Direction


	//SET POSITIONS
	float x = pos.northx;
	float y = pos.easty;

	float x00 = obst_pos.northx;
	float y00 = obst_pos.easty;

	//CALCUALTE DISTANCE
	return A*expf(-((((x-x00)*(x-x00))/(2*dx*dx))+(((y-y00)*(y-y00))/(2*dy*dy))));
}



/**
 * Set the configuration parameters of the potentialfield method by QGround Control
 * Note: This function is called by QGroundControl, when changing parameters!
 *
 * @param: Gt: Weight for distance to target minimization
 * @param: Go: Weight for distance to obstacles maximization
 * @param: Gw: Weight for Speed/Wind optimization
 * @param: Gm: Weight for avoiding maneuvers (tacks/gybes)
 * @param: search_dist: Simulated distance from the boat's bow for placing the probe-charges
 */
void pm_set_configuration(float Gt, float Go, float Gm, float Gw, float SearchDist) {

	Config.G_target = Gt;
	Config.G_obstacle = Go;
	Config.G_wind = Gw;
	Config.G_tack = Gm;

	Config.search_dist = SearchDist;
}




