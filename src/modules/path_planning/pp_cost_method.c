/*
 * pathplanning_costfunction.c
 *
 * This file contains all functions/variables for calculating an optimal path to a given target position.
 *
 * Notes:
 * 	- The whole pathplanning-module uses radians for angles <=> GPS-Positions have to be stored in Radians and not in degrees
 * 	- all angles are wrt. true north => bearings/headings are element of [0...2pi].
 *
 *  Created on: 02.03.2015
 *      Author: Jonas Wirz (wirzjo@student.ethz.ch)
 */


#include <stdint.h>
#include "pp_cost_method.h"

#include "pp_polardiagram.h"

//Calculate the minimum of two values
#define min(a, b) (((a) < (b)) ? (a) : (b))


/**********************************************************************************************/
/****************************  VARIABLES  *****************************************************/
/**********************************************************************************************/

#define HEADRESOLUTION 0.0872664625997f 	//Resolution for simulating the headings in [rad] (here 5°)
#define HEADRANGE	   1.74532925199f   	//Range for simulating the headings in [rad] (here [-100°...100°] wrt. boat-heading)


//Weights and other configuration parameters for the Cost-Function-Method
static struct {
	float Gw;          		//Weighting factor for sailing against the target while maximizing speed (was 0.5)
    float Go;          		//Weighting factor for avoiding obstacles (was 0.8)
	float Gm;          		//Weighting factor for avoiding maneuovres (was 0.5) (higher value <=> less maneuovres are allowed)
	float Gs;         		//Weighting factor for prefering courses that need no change in course
	float Gt;          		//Weighting factor for tactical considerations
	float GLee;       		//Weighting factor for passing Obstacles in Lee. (higher value <=> force boat to pass in Lee)
	float ObstSafetyRadius; //Safety Radius around an obstacle [m]
	float ObstHorizon; 		//Obstacle Horizon <=> inside this horizon obstacles are recognized [m]
	uint8_t WindowSize; 	//Size of the window for smoothing the Costfunction
} Config = {
		.Gw = 0.9f, //0.9
		.Go = 0.8f, //0.8
		.Gm = 0.4f, //0.4
		.Gs = 0.05f,//0.05
		.Gt = 0.1f, //0.1
		.GLee = 0.15f,//0.15
		.ObstSafetyRadius = 10.0f, //10
		.ObstHorizon = 100.0f, //100
		.WindowSize = 5 //5
};


/* @brief Calculate the total cost for a given simulated heading */
float total_cost(float seg, struct nav_state_s *state, struct nav_field_s *field);

/* @brief Get the cost for the wind and the target */
float cost_target_wind(float seg, struct nav_state_s *state, struct nav_field_s *field);

/* @brief Get the cost for avoiding maneuvers */
float cost_maneuver(float seg, struct nav_state_s *state);

/* @brief Get the cost for optimazing tactical considerations */
float cost_tactical(float seg,struct nav_state_s *state, struct nav_field_s *field);

/* @brief Get the cost for preventing the boat from selecting different headings too often */
float cost_heading_change(float seg,struct nav_state_s *state);

/* @brief Get the cost for avoiding obstacles and pass obstacles in lee */
float cost_ostacle(float seg, struct nav_state_s *state, struct nav_field_s *field);

/* @brief Smooth an Array of variable size and with a variable windowSize */
void smooth(const float signal[], int SignalLen, int KernelLen,float result[]);

#if C_DEBUG == 1
	void print_array(float array[], size_t length);
#endif






/**********************************************************************************************/
/****************************  PUBLIC FUNCTIONS  **********************************************/
/**********************************************************************************************/

/**
 * Main Pathplanning Function
 * This Function returns the optimal heading according to the cost-function and
 * the corresponding Parameters.
 *
 * @param Pointer to the State-Struct
 * @param Pointer to the Field-Struct
 */
float cm_NewHeadingReference(struct nav_state_s *state, struct nav_field_s *field) {

	/*Iterate over the possible "probe" headings
	 * Note: In order to reduce computational costs only a range specified by the variables
	 * HEADRANGE and HEADRESOLUTION is used as possible headings. */
	float seg_start = (state->heading_cur-HEADRANGE);
	float seg_end = (state->heading_cur+HEADRANGE);

	uint8_t probeNum = 2*HEADRANGE/HEADRESOLUTION+1;	//Number of Probe-Headings
	float costMat[(int)(probeNum)];
	float headMat[(int)(probeNum)];

	uint8_t ind = 0;	//Index for addressing costMat-Elements

	float seg;
	for(seg = seg_start; seg <= seg_end; seg += HEADRESOLUTION) {
		float seg_compass = fmod(seg,(2*PI));

		//Get the cost and save it in the matrix
		costMat[ind] = total_cost(seg_compass,state,field);
		headMat[ind] = seg_compass;

		#if C_DEBUG == 1
			//printf("Total Cost: %f, %f\n",seg_compass*RAD2DEG,costMat[ind]);
		#endif

		//Update Index
		ind++;
	}

	//Smooth Cost-Matrix
	float mat[probeNum];
	smooth(costMat,probeNum,Config.WindowSize,mat);

	//Print the two arrays
	#if C_DEBUG == 1
	print_array(costMat,probeNum);
	print_array(mat,probeNum);
	#endif


	//Find minimum Index
	uint8_t minIndex = nh_findMin(costMat,probeNum);

	//Get corresponding minimum Heading
	float optHeading = headMat[minIndex];

	return optHeading;
}



/**
 * Set the configuration parameters for the costfunction method
 * This function is called by QGroundControl
 *
 * @param Gw
 * @param Go
 * @param Gm
 * @param Gs
 * @param Gt
 * @param GLee
 * @param ObstSafetyRadius
 * @param ObstHorizon
*/
void cm_set_configuration(float Gw, float Go, float Gm, float Gs, float Gt, float GLee, float ObstSafetyRadius, float ObstHorizon, float WindowSize) {

	Config.Gw = Gw;
	Config.Go = Go;
	Config.Gm = Gm;
	Config.Gs = Gs;
	Config.Gt = Gt;
	Config.GLee = GLee;
	Config.ObstSafetyRadius = ObstSafetyRadius;
	Config.ObstHorizon = ObstHorizon;
	Config.WindowSize = WindowSize;
}




/**********************************************************************************************/
/****************************  PRIVATE FUNCTIONS  *********************************************/
/**********************************************************************************************/

/**
 * Calculate the cost for a given (simulated) heading.
 *
 * @param	seg: the angle (true heading), the cost is calculated for [rad]
 * @param   *state: Pointer to the State-Struct
 * @param   *field: Pointer to the Field-Struct
 * @return  the total cost for a given simulated heading
*/
float total_cost(float seg, struct nav_state_s *state, struct nav_field_s *field) {

	/*** TARGET/WIND COST ***/
	float Cw = cost_target_wind(seg,state,field);

	#if C_DEBUG == 1
	//printf("  Target/Wind-Cost: %f, %f\n",seg*RAD2DEG,Cw);
	#endif

	/*** MANEUVER COST ***/
	float Cm = cost_maneuver(seg,state);

	#if C_DEBUG == 1
	//printf("  Maneuver Cost: %f, %f\n",seg*RAD2DEG,Cm);
	#endif

	/*** TACTICAL COST ***/
	float Ct = cost_tactical(seg,state,field);

	#if C_DEBUG == 1
	//printf("  Tactical Cost: %f, %f\n",seg*RAD2DEG,Ct);
	#endif


	/*** SMALL HEADING CHANGE COST ***/
	float Cs = cost_heading_change(seg,state);

	#if C_DEBUG == 1
	//printf("  Small Heading Cost: %f, %f\n",seg*RAD2DEG,Cs);
	#endif

	/*** OBSTACLE COST ***/
	float Co = cost_ostacle(seg,state,field);

	#if C_DEBUG == 1
	//printf("  Obstacle Cost: %f, %f\n",seg*RAD2DEG,Co);
	#endif

	/*** TOTAL COST ***/
	return (Cw + Cm + Ct + Cs + Co);
}



/**
 * Target/Wind Cost
 *
 * This cost takes the winddirection and the boats expected velocities at different relative angles towards
 * the wind into account. Meanwhile it minimizes the distance towards the target.
 *
 *  @param seg: Simulated heading [rad]
 *  @param *state: Pointer to the State-Struct
 *  @param *field: Pointer to the Field-Struct
 *  @return Cost for the given simulated Heading
 */
float cost_target_wind(float seg, struct nav_state_s *state, struct nav_field_s *field) {

	//Apparent Wind Direction
	float appWind = nh_appWindDir(seg,state->wind_dir);

	//Calcualte x and y Differences
	float dx = state->position.northx-field->targets[state->targetNum].northx;
	float dy = state->position.easty-field->targets[state->targetNum].easty;

	//Distance to target
	float distToTarget = nh_ned_dist(state->position,field->targets[state->targetNum]);

	float tgx = dx/distToTarget;			//Vector pointing towards the target
	float tgy = dy/distToTarget;

	//Get the Boatspeed from the Polardiagram
	float boatspeed = pol_polardiagram(appWind,state->wind_speed);

	//Calcualte Direction and Speed of the Boat
	float vhx = cosf(seg)*boatspeed;
	float vhy = sinf(seg)*boatspeed;

	//Calculate Dot-Product of Boatspeed and TargetVector
	float vg = vhx * tgx + vhy * tgy;


	//Return the weighted cost
	return (vg*Config.Gw);

} //end of cost_target_wind




/**
 * Maneuver Cost
 *
 * This cost prevents the boat from tacking or gybing too often. Each tack or gybe slows down the boat.
 *
 *  @param seg: Simulated heading [rad]
 *  @param *state: Pointer to the State-Struct
 *  @return Cost for the given simulated Heading
 *
 *  Debug State: Tested with PC-Environment => working
 */
float cost_maneuver(float seg, struct nav_state_s *state) {

	float appWind= nh_appWindDir(seg,state->wind_dir);		//Current apparent Wind Direction

	uint8_t oldhull = 0;									//Current hull
	if(nh_appWindDir(state->heading_cur,state->wind_dir) < 0) {
		//Wind from Starboard
		oldhull = -1;
	} else {
		//Wind from Portside
		oldhull = 1;
	}

	uint8_t newhull = 0;									//New hull
	if(appWind < 0) {
		//Wind from Starboard
		newhull = -1;
	} else {
		//Wind from Portside
		newhull = 1;
	}

	//Calculate the Maneuver Cost
	if(oldhull == newhull) {
		//Boat sails on the same hull <=> assign low cost
		return 0;
	} else {
		//Boat changes hull <=> tack or gybe <=> assign high cost
		return Config.Gm * 1;
	}

} //end of cost_maneuver



/**
 * Tactical Cost
 *
 * This cost account for tactical considerations.
 * 	1) The boat should prefer sailing with wind from starboard. This way it does NOT have to give way to
 * 	   boats sailing with wind from portside (COLREGS). This is explicitly important when sailing upwind.
 * 	2) The boat should not get too close to the laylines. Because sailing at the laylines is risky, because
 * 	   of possible windshifts. Therefore, the boat should prefere sailing close to the centerline.
 *
 *  @param seg: Simulated heading [rad]
 *  @param *state: Pointer to the State-Struct
 *  @param *field: Pointer to the Field-Struct
 *  @return Cost for the given simulated Heading
 */
float cost_tactical(float seg,struct nav_state_s *state, struct nav_field_s *field) {

	float appWind= nh_appWindDir(seg,state->wind_dir);		//Current apparent Wind Direction


	/* Case 1) */
	float Ct_colreg = 0;									//Tactical Cost for COLREGS

    if(appWind < 0) {
        Ct_colreg = 0;
    } else {
        Ct_colreg = Config.Gt * 0.5 * 1;
    }

    //DEBUG:



    /* Case 2) */
	float Ct = 0;											//Tactical Cost for Centerline

    if(Config.Gt > 0) {

		float target_bearing = nh_ned_bearing(state->position,field->targets[state->targetNum]);

		if((state->heading_cur >= target_bearing*0.9f) && (state->heading_ref <= 1.1f*target_bearing)) {
			//The boat is on the last leg.

			Ct = 0; 	//Assign a low cost, since we are on the last leg it's up to the other costs to
						//guide the boat towards the target.

		} else {
			//The boat is not on the last leg. => push the boat away from the laylines <=> stay close to the center-line

			//Calculate the Centerline
			float dx = cosf(state->wind_dir);
			float dy = sinf(state->wind_dir);
			float norm = sqrtf(dx*dx+dy*dy);

			float clx = dx/norm;
			float cly = dy/norm;


			//Calculate the Heading-Vector
			float hx = cosf(seg);
			float hy = sinf(seg);
			norm = sqrt(hx * hx + hy * hy);
			hx = hx/norm;
			hy = hy/norm;


			//The meeting-point of the center-line and the boat-heading
			float tcl = (clx*(state->position.easty-field->targets[state->targetNum].easty)-cly*(state->position.northx-field->targets[state->targetNum].northx))/(hx*cly-hy*clx);
			NEDpoint mcl;
			mcl.northx = state->position.northx + tcl * hx;
			mcl.easty = state->position.easty + tcl * hy;

			if(tcl < 0) {
				//the meeting point lays behind the boat's heading => no meeting point exists => assign very high cost
				mcl.northx = 10000000.0f;
				mcl.easty = 10000000.0f;
			}

			float dist_target = nh_ned_dist(state->position,field->targets[state->targetNum]);  //The closer the boat gets to the target, the smaller is the cone-opening
			float dist_mcl = nh_ned_dist(state->position,mcl);			//Distance to the Center-Line

			Ct = min(dist_mcl,dist_target)/dist_target;

			if(Ct < 0.99999f) {  //Give a rectangular shape to the cost
					Ct = 0;
			}

		} //if we are on the last leg

    } //end of "is the tactical cost weight bigger than 0?"

	return (Config.Gt * Ct + Ct_colreg);           //Weight and save the cost

} //end of cost_tactical



/**
 * Small heading change cost
 *
 * Every change in heading causes movements with the rudder. Each rudder movement slows down the boat.
 * Therefore, whenever possible choose the same heading as the new heading reference as the boat is
 * currently sailing.
 *
 *  @param seg: Simulated heading [rad]
 *  @param *state: Pointer to the State-Struct
 *  @param *field: Pointer to the Field-Struct
 *  @return Cost for the given simulated Heading
 *
 *  Debug State: Validated on PC-Test => working
 */
float cost_heading_change(float seg,struct nav_state_s *state) {

	return Config.Gs * fabsf(seg-state->heading_cur)/12.57f;

} //end of cost_heading_change



/**
 * Obstacle Cost
 *
 * Cost for maximising the distance to obstacles and avoid them.
 * 	1) Avoid Obstacles by avoiding selecting headings that guide the boat too close
 * 	   to the obstacles.
 * 	2) Whenever possible pass obstacles in lee, since this leaves more space for maneuvers.
 * 	   Especially on upwind courses, passing an obstacle in windward side could lead to an
 * 	   additional maneuver that slows down the boat drastically.
 */
float cost_ostacle(float seg, struct nav_state_s *state, struct nav_field_s *field) {

	float Co = 0;
	float CLee = 0;

	//Loop over all Obstacles
	uint8_t i;
	for (i = 0; i < field->NumberOfObstacles; i++) {

		float C = 0;	//Cost for this Obstacle

		/* Only obstacles within a certain Horizon are taken into account. This means that only obstacles in our range
		 * affect the Pathplanning. We do not care about far away obstacles. */
		float distance = nh_ned_dist(state->position,field->obstacles[i]);
		/*Note: the distance to the Obstacle is never zero, since this would lead to a division by zero and therefore
		 *      an unpredictable behaviour of the Pixhawk. */
		if(distance > Config.ObstHorizon) {
			//Obstacle is too far away => continue with the next obstacle
			continue;
		}


		/* Add the safety radius to the obstacle. An obstacle is modelled as a point on the map. To avoid the obstacle
		 * and compensate for uncertainties (e.g. sudden changes in wind) the obstacle is made larger virtually */
		float obst_bear = nh_ned_bearing(state->position,field->obstacles[i]);

		float ang_correction = atanf((Config.ObstSafetyRadius*1.5f)/distance);

		float max_obst_bear = fmod(ang_correction+obst_bear,2*PI);
		float min_obst_bear = fmod(obst_bear-ang_correction,2*PI);


		/* Check, if the boat is on collision course with an obstacle. It is on collision course,
		 * min_obst_bear < seg < max_obst_bear    <=> the course to check directly guides us towards
		 * an obstacle. */
		if((min_obst_bear < seg) && (seg < max_obst_bear )) {
			//Boat is on collision course
			C = Config.Go;
		} else {
			//Boat is NOT on collision course
			C = 0;
		}

		/* Update the TOTAL obstacle cost */
		Co = Co + C;



		/************************************/
		/************ PASS IN LEE ***********/
		/************************************/
		/* Whenever possible pass an obstacle in Lee. This way there is more room left for accounting for windshifts
		 * and compensating other uncertainties. This is especially important when sailing upwind, because windshifts could
		 * force the boat to do another tack, what slows down the boat and therefore has to be avoided.*/

		float appWind= nh_appWindDir(obst_bear,state->wind_dir);

		if(appWind < 0) { //Prefere Sailing on min_obst_head
			if(seg < min_obst_bear) { //The Boat will pass in Lee of the Obstacle => GOOD
				CLee = 0;
			} else { //The Boat will pass in Luv of the Obstacle => BAD
		        CLee = Config.GLee * 1;
			}
		} else {          //Prefere Sailing on max_obst_head
			if(seg > max_obst_bear) { //The Boat will pass in Lee of the Obstacle => GOOD
				CLee = 0;
		    } else {  //The Boat will pass in Luv of the Obstacle => BAD
		        CLee = Config.GLee * 1;
		    }
		}

	} //for

	return (Co + CLee);

} //end of cost_ostacle




/**
 * Smooths an array of variable size using a moving averaging window of variable size
 *
 * @param signal: Pointer to an array. Note this array is directly modified!
 * @param windowSize: Size of the moving averaging window
 */
void smooth(const float signal[], int SignalLen, int KernelLen,
            float result[]) {

	uint8_t kernSize = KernelLen/2;	//Look half of the kernel to the left and to the right

	int n;
	for(n = 0; n < SignalLen; n++) {
		//Iterate over all elements in the signal

		//Let the endpoints of the Array untouched
		if(n<kernSize) {
			//Endpoints at the beginning of the Array
			result[n] = signal[n];
		} else if(n>(SignalLen-kernSize-1)) {
			//Endpoints at the end of the Array
			result[n] = signal[n];
		} else {
			float sum = 0;
			int i;
			for(i = (n-kernSize); i <= n+kernSize; i++) {
				sum += signal[i];
				//printf("%d ",i);
			}

			result[n] = sum/KernelLen;
		}

	}

} //end of smooth


#if C_DEBUG == 1
	void print_array(float array[], size_t length) {
		FILE *f = fopen("output.txt", "a");
		if (f == NULL)
		{
		    printf("Error opening file!\n");
		}


		uint8_t i;
		for(i=0; i<length-1; i++) {
			fprintf(f,"%f, ",array[i]);
		}
		fprintf(f,"%f, //\n",array[length-1]);
		fclose(f);
	}
#endif









