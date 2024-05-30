#include "mex.h"
#include <math.h>
#include <stdio.h>
#include "types_TMATS.h"
#include "constants_TMATS.h"


/* Input Arguments */
#define	ENV_IN	prhs[0] /*--- 3 Inputs: Alt, MN, dTamb ---*/

/* Output Arguments */
#define	AMB_OUT	plhs[0] /*--- 4 Outputs: Tt, Pt, Ps, Ts ---*/

#if !defined(MAX)
#define	MAX(A, B)	((A) > (B) ? (A) : (B))
#endif

#if !defined(MIN)
#define	MIN(A, B)	((A) < (B) ? (A) : (B))
#endif

extern void Ambient_TMATS_body(double *y, const double *u, const AmbientStruct* prm);

void mexFunction(int nlhs, mxArray *plhs[],
                 int nrhs, const mxArray *prhs[])
{
    unsigned int m,n; 
    /*--------Define Inputs----------*/
    double *env;
    double AltIn, MNIn, dTambIn;

    /*--- Define Block Inputs/Outputs ---*/
    /*--- Ambient ---*/
    double amb_u[3]; /*--- Inputs:  Alt, dTamb, MN ---*/
    double amb_y[4]; /*--- Outputs: Tt, Pt, Ps, Ts ---*/

    /*--- Define Output Vector Pointers ---*/
    double *AMB; /*--- Outputs ---*/

    /*===================================================================*/
    /*--- Define parameters ---*/
    /*===================================================================*/
    /*--- Ambient outputs ---*/
    double ht0, Tt0, Pt0, FAR0, Ps0, Ts0, Veng;

    /*===================================================================*/
    /* DEFINE All necessary structures and populate with data            */
    /*===================================================================*/
    /*--- Define GTF ambient structure ---*/
    double GTF_ambient_AFARc = 0;
    double GTF_ambient_X_A_AltVec[15] = {-5000, 0, 5000, 10000, 15000, 20000, 25000, 30000, 35000, 40000, 45000, 50000, 60000, 70000, 80000};
    double GTF_ambient_T_A_TsVec[15] = {536.51, 518.67, 500.84, 483.03, 465.22, 447.41, 429.62, 411.84, 394.06, 389.97, 389.97, 389.97, 389.97, 392.25, 397.69};
    double GTF_ambient_T_A_PsVec[15] = {17.554, 14.696, 12.228, 10.108, 8.297, 6.759, 5.461, 4.373, 3.468, 2.73, 2.149, 1.692, 1.049, 0.651, 0.406};
    double GTF_ambient_X_A_FARVec[7] = {0, 0.0050, 0.0100, 0.0150, 0.0200, 0.0250, 0.0300};
    double GTF_ambient_T_A_RtArray[7] = {0.0686, 0.0686, 0.0686, 0.0686, 0.0686, 0.0686, 0.0686};
    double GTF_ambient_Y_A_TVec[2] = {300, 10000};
    double GTF_ambient_T_A_gammaArray[14] = {1.4, 1.4, 1.4, 1.4, 1.4, 1.4, 1.4, 1.4, 1.4, 1.4, 1.4, 1.4, 1.4, 1.4};
    char GTF_ambient_BlkNm[13] = "GTF_ambient\0";
    int GTF_ambient_IWork[5] = {0, 0, 0, 0, 0};
    int GTF_ambient_A = 15;
    int GTF_ambient_B = 7;
    int GTF_ambient_C = 2;
    
    struct AmbientStruct GTF_ambient = {
        GTF_ambient_AFARc,
        &GTF_ambient_X_A_AltVec[0],
        &GTF_ambient_T_A_TsVec[0],
        &GTF_ambient_T_A_PsVec[0],
        &GTF_ambient_X_A_FARVec[0],
        &GTF_ambient_T_A_RtArray[0],
        &GTF_ambient_Y_A_TVec[0],
        &GTF_ambient_T_A_gammaArray[0],
        &GTF_ambient_BlkNm[0],
        &GTF_ambient_IWork[0],
        GTF_ambient_A,
        GTF_ambient_B,
        GTF_ambient_C,
    };

    /*===================================================================*/
    /*===================================================================*/
    /* Check for proper number of arguments. */
    if (nrhs != 1) {
    mexErrMsgTxt("1 inputs to Ambient_C required");
    } else if (nlhs != 1) {
    mexErrMsgTxt("1 output arguments to Ambient_C required");
    }
    
    m = mxGetM(ENV_IN); 
    n = mxGetN(ENV_IN);
    if (!mxIsDouble(ENV_IN) || mxIsComplex(ENV_IN) || 
	(MAX(m,n) != 3) || (MIN(m,n) != 1)) { 
	mexErrMsgTxt("Requires that ENV_IN be a 3 x 1 vector."); 
    } 

    /* Create a matrix for the return argument */ 
    AMB_OUT = mxCreateDoubleMatrix(4, 1, mxREAL); 

    /* Assign pointers to the various I/O parameters */ 
    AMB = mxGetPr(AMB_OUT);

    /*--- Environmental inputs ---*/
    env = mxGetPr(ENV_IN);
    AltIn    = env[0]; /* Altitude */
    MNIn     = env[1]; /* Mach */
    dTambIn  = env[2]; /* dTamb */

    /*--- Ambient ---*/
    amb_u[0] = AltIn;
    amb_u[1] = dTambIn;
    amb_u[2] = MNIn;

    Ambient_TMATS_body(&amb_y[0], &amb_u[0], &GTF_ambient);
    ht0 = amb_y[0];
    Tt0 = amb_y[1];
    Pt0 = amb_y[2];
    FAR0 = amb_y[3];
    Ps0 = amb_y[4];
    Ts0 = amb_y[5];
    Veng = amb_y[6];

    /* ================================================================= */
    /*--- Assign Outputs ---*/
    /* ================================================================= */
    /*--- 4 AMB Outputs: Tt0, Pt0, Ts0, Ps0 ---*/
    AMB[0] = Tt0;
    AMB[1] = Pt0;
    AMB[2] = Ts0;
    AMB[3] = Ps0;
}    