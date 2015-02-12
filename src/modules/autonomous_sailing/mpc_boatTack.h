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
#define mpc_boatTack_SET_PRINTLEVEL    (0)
#endif

/* timing */
#ifndef mpc_boatTack_SET_TIMING
#define mpc_boatTack_SET_TIMING    (0)
#endif

/* Numeric Warnings */
/* #define PRINTNUMERICALWARNINGS */

/* maximum number of iterations  */
#define mpc_boatTack_SET_MAXIT         (30)	

/* scaling factor of line search (affine direction) */
#define mpc_boatTack_SET_LS_SCALE_AFF  (0.9f)

/* scaling factor of line search (combined direction) */
#define mpc_boatTack_SET_LS_SCALE      (0.95f)

/* minimum required step size in each iteration */
#define mpc_boatTack_SET_LS_MINSTEP    ((float)1E-08)

/* maximum step size (combined direction) */
#define mpc_boatTack_SET_LS_MAXSTEP    (0.995f)

/* desired relative duality gap */
#define mpc_boatTack_SET_ACC_RDGAP     (0.0001f)

/* desired maximum residual on equality constraints */
#define mpc_boatTack_SET_ACC_RESEQ     ((float)1E-06)

/* desired maximum residual on inequality constraints */
#define mpc_boatTack_SET_ACC_RESINEQ   ((float)1E-06)

/* desired maximum violation of complementarity */
#define mpc_boatTack_SET_ACC_KKTCOMPL  ((float)1E-06)


/* RETURN CODES----------------------------------------------------------*/
/* solver has converged within desired accuracy */
#define mpc_boatTack_OPTIMAL      (1)

/* maximum number of iterations has been reached */
#define mpc_boatTack_MAXITREACHED (0)

/* no progress in line search possible */
#define mpc_boatTack_NOPROGRESS   (-7)




/* PARAMETERS -----------------------------------------------------------*/
/* fill this with data before calling the solver! */
typedef struct mpc_boatTack_params
{
    /* vector of size 3 */
    mpc_boatTack_FLOAT minusAExt_times_x0[3];

    /* diagonal matrix of size [4 x 4] (only the diagonal is stored) */
    mpc_boatTack_FLOAT Hessians[4];

    /* matrix of size [4 x 4] (column major format) */
    mpc_boatTack_FLOAT HessiansFinal[16];

    /* vector of size 2 */
    mpc_boatTack_FLOAT lowerBound[2];

    /* vector of size 2 */
    mpc_boatTack_FLOAT upperBound[2];

} mpc_boatTack_params;


/* OUTPUTS --------------------------------------------------------------*/
/* the desired variables are put here by the solver */
typedef struct mpc_boatTack_output
{
    /* vector of size 1 */
    mpc_boatTack_FLOAT u0[1];

} mpc_boatTack_output;


/* SOLVER INFO ----------------------------------------------------------*/
/* diagnostic data from last interior point step */
typedef struct mpc_boatTack_info
{
    /* iteration number */
    int it;
	
    /* inf-norm of equality constraint residuals */
    mpc_boatTack_FLOAT res_eq;
	
    /* inf-norm of inequality constraint residuals */
    mpc_boatTack_FLOAT res_ineq;

    /* primal objective */
    mpc_boatTack_FLOAT pobj;	
	
    /* dual objective */
    mpc_boatTack_FLOAT dobj;	

    /* duality gap := pobj - dobj */
    mpc_boatTack_FLOAT dgap;		
	
    /* relative duality gap := |dgap / pobj | */
    mpc_boatTack_FLOAT rdgap;		

    /* duality measure */
    mpc_boatTack_FLOAT mu;

	/* duality measure (after affine step) */
    mpc_boatTack_FLOAT mu_aff;
	
    /* centering parameter */
    mpc_boatTack_FLOAT sigma;
	
    /* number of backtracking line search steps (affine direction) */
    int lsit_aff;
    
    /* number of backtracking line search steps (combined direction) */
    int lsit_cc;
    
    /* step size (affine direction) */
    mpc_boatTack_FLOAT step_aff;
    
    /* step size (combined direction) */
    mpc_boatTack_FLOAT step_cc;    

	/* solvertime */
	mpc_boatTack_FLOAT solvetime;   

} mpc_boatTack_info;



/* SOLVER FUNCTION DEFINITION -------------------------------------------*/
/* examine exitflag before using the result! */
int mpc_boatTack_solve(mpc_boatTack_params* params, mpc_boatTack_output* output, mpc_boatTack_info* info);



#endif
