#include "types_TMATS.h"
#include "types_TMATS_additions.h"
#include "functions_TMATS.h"
#include "constants_TMATS.h"
#include <math.h>

#ifdef MATLAB_MEX_FILE
#include "simstruc.h"
#endif

void Shaft_TMATS_body(double *y, const double *u, const ShaftStruct* prm)
{
    
    double TrqIn     = u[0];     /* Torque */
    double PwrIn     = u[1];     /* Shaft Power Extraction */
    double NmechIn   = u[2];     /* Nmech [rpm] 	*/
    
    /*--------Define Parameters -------*/
    double PwrInTrq;
    double NdotOut;
    double NmechOut;
    
    PwrInTrq = C_HP_PER_RPMtoFT_LBF*PwrIn*divby(NmechIn);

    NdotOut = (TrqIn + PwrInTrq) * 60 * divby(2 * 3.14159265358979 * prm->Inertia_M);

    NmechOut = NmechIn;

    /*------Assign output values------------*/
    y[0] = NmechOut;      /* NmechOut */
    y[1] = NdotOut;       /* Ndot */
}
