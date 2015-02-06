/*
mpc_boatTack : A fast customized optimization solver.

Copyright (C) 2014 EMBOTECH GMBH [info@embotech.com]


This software is intended for simulation and testing purposes only. 
Use of this software for any commercial purpose is prohibited.

This program is distributed in the hope that it will be useful.
EMBOTECH makes NO WARRANTIES with respect to the use of the software 
without even the implied warranty of MERCHANTABILITY or FITNESS FOR A 
PARTICULAR PURPOSE. 

EMBOTECH shall not have any liability for any damage arising from the use
of the software.

This Agreement shall exclusively be governed by and interpreted in 
accordance with the laws of Switzerland, excluding its principles
of conflict of laws. The Courts of Zurich-City shall have exclusive 
jurisdiction in case of any dispute.

*/

#ifndef __mpc_boatTack_H__
#define __mpc_boatTack_H__

/* DATA TYPE ------------------------------------------------------------*/
typedef float mpc_boatTack_FLOAT;

typedef float INTERFACE_FLOAT;

/* SOLVER SETTINGS ------------------------------------------------------*/
/* print level */
#ifndef mpc_boatTack_SET_PRINTLEVEL
#define mpc_boatTack_SET_PRINTLEVEL    (2)
#endif

/* timing */
#ifndef mpc_boatTack_SET_TIMING
#define mpc_boatTack_SET_TIMING    (0)
#endif

/* maximum number of iterations  */
#define mpc_boatTack_SET_MAXIT         (30)

/* desired relative duality gap */
#define mpc_boatTack_SET_ACC_DGAP     (0.001)

/* desired maximum residual on consensus constraint */
#define mpc_boatTack_SET_ACC_CONSENSUS     (0.001)


/* RETURN CODES----------------------------------------------------------*/
/* solver has converged within desired accuracy */
#define mpc_boatTack_OPTIMAL      (1)

/* maximum number of iterations has been reached */
#define mpc_boatTack_MAXITREACHED (0)



/* PARAMETERS -----------------------------------------------------------*/
/* fill this with data before calling the solver! */
typedef struct mpc_boatTack_params
{
    /* vector of size 3 */
    float minusAExt_times_x0[3];

    /* diagonal matrix of size [4 x 4] (only the diagonal is stored) */
    float Hessians[4];

    /* matrix of size [4 x 4] (column major format) */
    float HessiansFinal[16];

    /* vector of size 2 */
    float lowerBound[2];

    /* vector of size 2 */
    float upperBound[2];

} mpc_boatTack_params;


/* OUTPUTS --------------------------------------------------------------*/
/* the desired variables are put here by the solver */
typedef struct mpc_boatTack_output
{
    /* vector of size 1 */
    float u0[1];

} mpc_boatTack_output;


/* SOLVER INFO ----------------------------------------------------------*/
/* diagnostic data from last step */
typedef struct mpc_boatTack_info
{ 
	/* iteration number */
    int it;

	/* solvertime */
	float solvetime; 

	/* inf-norm of equality constraint residuals */
    float res_eq;

    /* primal objective */
    float pobj;	
	
    /* dual objective */
    float dobj;	

    /* duality gap := pobj - dobj */
    float dgap;		
	
    /* relative duality gap := |dgap / pobj | */
    float rdgap;	

} mpc_boatTack_info;





/* SOLVER FUNCTION DEFINITION -------------------------------------------*/
int mpc_boatTack_solve(mpc_boatTack_params* params, mpc_boatTack_output* output, mpc_boatTack_info* info);

/*int mpc_boatTack_control_interface(mpc_boatTack_controlparams* params, mpc_boatTack_output* output, mpc_boatTack_info* info); */



/* DATA SIZES */
#define TOTAL_SIZE 44
#define EQU_SIZE 33
#define INEQU_SIZE 44
#define Z_SIZE 44

/* FIXED NUMBER OF ITERATIONS */
//#define I_MAX 30

/* FIXED NUMBER OF PRIMAL STEPS PER ITERATION */
#define PRIMAL_STEPS 1



#endif
