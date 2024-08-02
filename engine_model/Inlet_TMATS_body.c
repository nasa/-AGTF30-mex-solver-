#include "types_TMATS.h"
#include "types_TMATS_additions.h"
#include "functions_TMATS.h"
#include <math.h>

#ifdef MATLAB_MEX_FILE
#include "simstruc.h"
#endif

void Inlet_TMATS_body(double *y, const double *u, const InletStruct* prm, const double enable_debug)
{
    
    double WIn       = u[0];     /* Mass flow */
    double htIn      = u[1];     /* enthalpy 	*/
    double TtIn      = u[2];     /* Total Temperature [degF] 	*/
    double PtIn      = u[3];     /* Total Pressure [psia] 	*/
    double FARIn     = u[4];     /* Fuel-Air-Ratio (frac) 	*/
    double PambIn    = u[5];     /* Pamb */
    
    /*--------Define Constants-------*/
    double PtOut;
    double eRam_sf;
    
    int interpErr = 0;

    eRam_sf = interp1Ac(prm->X_eRamVec_M,prm->T_eRamtbl_M,PtIn/PambIn,prm->A,&interpErr);
    if (interpErr == 1 && *(prm->IWork+Er1) == 0){
        #ifdef MATLAB_MEX_FILE
        if (enable_debug) {
        printf("Warning in %s, Error calculating eRam_sf. Vector definitions may need to be expanded.\n", prm->BlkNm);
        }
        #endif
        *(prm->IWork+Er1) = 1;
    }
    
    PtOut = PtIn * prm->eRambase_M * eRam_sf;

    /*------Assign output values------------*/
    y[0] = WIn;       /* Mass flow */
    y[1] = htIn;      /* Total enthalpy */
    y[2] = TtIn;      /* Total Temperature [degR] */
    y[3] = PtOut;      /* Total Pressure [psia] */
    y[4] = FARIn;     /* Fuel to Air Ratio */    
}
