#include "types_TMATS_additions.h"
#include <math.h>
#include "types_TMATS.h"

#ifdef MATLAB_MEX_FILE
#include "simstruc.h"
#endif

extern void StaticCalc_TMATS_body(double *y1, const double *u1, const StaticCalcStruct* prm, const double enable_debug);

void Duct_TMATS_body(double *y, const double *u, const DuctStruct* prm, const double enable_debug)
{
    
    double WIn       = u[0];     /* Mass flow */
    double htIn      = u[1];     /* enthalpy 	*/
    double TtIn      = u[2];     /* Total Temperature [degF] 	*/
    double PtIn      = u[3];     /* Total Pressure [psia] 	*/
    double FARIn     = u[4];     /* Fuel-Air-Ratio (frac) 	*/

    /*--- Define StaticCalc I/O ---*/
    double staticcalc_y[5];
    
    /*--------Define Constants-------*/
    double MN, PtOut;

    /*--- Define StaticCalc structure ---*/
    double AthroatIn = prm->Ath;
    double MNIn = 0.45;
    int SolveType = 0;
    double X_FARVec[7] = {0, 0.0050, 0.0100, 0.0150, 0.0200, 0.0250, 0.0300};
    double T_RtArray[7] = {0.0686, 0.0686, 0.0686, 0.0686, 0.0686, 0.0686, 0.0686};
    double Y_TtVec[7] = {300, 10000};
    double T_gammaArray[14] = {1.4, 1.4, 1.4, 1.4, 1.4, 1.4, 1.4, 1.4, 1.4, 1.4, 1.4, 1.4, 1.4, 1.4};
    int IWork[1] = {0};
    int  A = 7;
    int  B = 2;
    struct StaticCalcStruct duct = {
        AthroatIn,
        MNIn,
        SolveType,
        &X_FARVec[0],
        &T_RtArray[0],
        &Y_TtVec[0],
        &T_gammaArray[0],
        &prm->BlkNm[0],
        &IWork[0],
        A,
        B,
    };

    /*---- call StaticCalc to retrieve Mach number --- */
    StaticCalc_TMATS_body(&staticcalc_y[0], &u[0], &duct, enable_debug);
    MN = staticcalc_y[3];

    /*--- calculate PtOut ---*/
    PtOut = (1 - (MN/prm->MNdes) * (MN/prm->MNdes) * prm->dP_M) * PtIn;
    
    /*------Assign output values------------*/
    y[0] = WIn;       /* Mass flow */
    y[1] = htIn;      /* Total enthalpy */
    y[2] = TtIn;      /* Total Temperature [degR] */
    y[3] = PtOut;      /* Total Pressure [psia] */
    y[4] = FARIn;     /* Fuel to Air Ratio */    
}
