#include "functions_TMATS.h"
#include <math.h>

void Splitter_TMATS(double *y, double *y1, const double *u, const double *u1)
{
    
    double WIn    = u[0];     /* Mass flow */
    double htIn   = u[1];     /* Enthalpy 	*/
    double TtIn   = u[2];     /* Temperature */
    double PtIn   = u[3];     /* Pressure 	*/
    double FARIn  = u[4];     /* Fuel-Air-Ratio */

    double BPR    = u1[0];
    
    /*--------Define Parameters -------*/
    double BPR2;
    double Wbp, Wcore;
    
    if(BPR > 0)
        BPR2 = BPR;
    else
        BPR2 = 0;

    Wbp = WIn * BPR2 * (1/(BPR2+1));

    Wcore = WIn * (1/(BPR2+1));

    /*------Assign output values------------*/
    y[0] = Wbp;      /* Mass flow */
    y[1] = htIn;      /* Enthalpy */
    y[2] = TtIn;     /* Temperatue */
    y[3] = PtIn;     /* Pressure */
    y[4] = FARIn;    /* Fuel-Air-Ratio */

    y1[0] = Wcore;      /* Mass flow */
    y1[1] = htIn;      /* Enthalpy */
    y1[2] = TtIn;     /* Temperatue */
    y1[3] = PtIn;     /* Pressure */
    y1[4] = FARIn;    /* Fuel-Air-Ratio */
}
