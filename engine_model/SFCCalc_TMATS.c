#include "functions_TMATS.h"
#include <math.h>

#ifdef MATLAB_MEX_FILE
#include "simstruc.h"
#endif

void SFCCalc_TMATS(double *y, const double *u)
{
    
    double WfIn      = u[0];     /* Fuel flow */
    double FgIn      = u[1];     /* Gross thrust 	*/
    double FdragIn   = u[2];     /* Drag 	*/
    
    /*--------Define Constants-------*/
    double secs2hrs = 3600.0;

    /*--------Define Parameters -------*/
    double SFC;
    double Fnet;
    
    Fnet = FgIn - FdragIn;
    SFC = WfIn*secs2hrs*divby(Fnet);

    /*------Assign output values------------*/
    y[0] = SFC;       /* Specific Fuel Consumption */
    y[1] = Fnet;      /* Net Thrust */
}
