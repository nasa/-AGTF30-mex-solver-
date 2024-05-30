#include "mex.h"
#include <math.h>
#include <stdio.h>
#include "types_TMATS.h"
#include "types_TMATS_additions.h"
#include "constants_TMATS.h"


/* Input Arguments */
#define	ENV_IN	prhs[0]
#define	CMD_IN	prhs[1]
#define TAR_OUT  prhs[2]
#define HEALTH_PARAMS_IN prhs[3]

/* Output Arguments */
#define	DEP_OUT	plhs[0]
#define	X_OUT	plhs[1]
#define	U_OUT	plhs[2]
#define	Y_OUT	plhs[3]
#define	E_OUT	plhs[4]

#if !defined(MAX)
#define	MAX(A, B)	((A) > (B) ? (A) : (B))
#endif

#if !defined(MIN)
#define	MIN(A, B)	((A) < (B) ? (A) : (B))
#endif

extern void Ambient_TMATS_body(double *y, const double *u, const AmbientStruct* prm);
extern void Inlet_TMATS_body(double *y, const double *u, const InletStruct* prm);
extern void Compressor_TMATS_body(double* y, double* y1, double* y2, const double* u, const double* Wcust, const double* FracWbld, const CompressorStruct* prm);
extern void Splitter_TMATS(double* y, double* y1, const double* u, const double* u1);
extern void Duct_TMATS_body(double *y, const double *u, const DuctStruct* prm);
extern void Valve_TMATS_body(double* y, const double* u, const ValveStruct* prm);
extern void Nozzle_TMATS_body(double* y, const double* u, const NozzleStruct* prm);
extern void StaticCalc_TMATS_body(double *y1, const double *u1, const StaticCalcStruct* prm);
extern void Burner_TMATS_body(double* y, const double* u, const BurnStruct* prm);
extern void Turbine_TMATS_body(double* y, const double* u, const double* CoolFlow, const TurbineStruct* prm);
extern void Shaft_TMATS_body(double *y, const double *u, const ShaftStruct* prm);
extern void SFCCalc_TMATS(double* y, const double* u);

void mexFunction(int nlhs, mxArray *plhs[],
                 int nrhs, const mxArray *prhs[])
{
    unsigned int m,n; 
    /*--------Define Inputs----------*/
    double *env;
    double AltIn, MNIn, dTambIn;
    
    double *cmd;
    double WIn, FAN_RLIn, LPC_RLIn, HPC_RLIn, BPRIn, HPT_PRIn, LPT_PRIn;
    double WfIn, VAFNIn, VBVIn, N2In, N3In, HPpwrIn, LPpwrIn;

    double *tar;
    double LPC_SM_target, Fnet_target, T45_target;

    double *health_params;
    double GTF_fan_WcMod, GTF_fan_PRMod, GTF_fan_EffMod, GTF_fan_hp_En;
    double GTF_lpc_WcMod, GTF_lpc_PRMod, GTF_lpc_EffMod, GTF_lpc_hp_En;
    double GTF_hpc_WcMod, GTF_hpc_PRMod, GTF_hpc_EffMod, GTF_hpc_hp_En;
    double GTF_hpt_WcMod, GTF_hpt_EffMod, GTF_hpt_hp_En;
    double GTF_lpt_WcMod, GTF_lpt_EffMod, GTF_lpt_hp_En;

    /*--- Define Block Inputs/Outputs ---*/
    /*--- Ambient ---*/
    double amb_u[3]; /*--- Inputs:  Alt, dTamb, MN ---*/
    double amb_y[8]; /*--- Outputs: W, ht, Tt, Pt, FAR, Ps, Ts, Veng ---*/

    /*--- Inlet ---*/
    double inlet_u[6]; /*--- Inputs:  W, ht, Tt, Pt, FAR, Ps ---*/
    double inlet_y[5]; /*--- Outputs: W, ht, Tt, Pt, FAR ---*/

    /*--- Compressor Blocks (Fan, LPC, and HPC) ---*/
    double compressor_u[12]; /*--- Inputs:  W, ht, Tt, Pt, FAR, Nmech, Rline, Alpha, s_C_Nc, s_C_Wc, s_C_PR, s_C_Eff ---*/
    double compressor_y[27]; /*--- Outputs: W, ht, Tt, Pt, FAR, Trq, Werr, SMavail, C_Nc, C_Wc, C_PR, C_Eff, Wcin, Nc, PR, NcMap, WcMap, PRMap, EffMap, SPR, Wbleeds, Pwrb4bleed, PwrBld, Pwrout, SMMap, SPRMap, Test ---*/
    double compressor_y1[1]; /*--- Outputs: Customer bleeds. Not used in STARC-ABL so set to dimension of 1. Setting GTF_fan_CustBldEn = 0 will disable customer bleeds ---*/
    double compressor_y2[15]; /*--- Outputs: Fractional bleeds. In AGTF30 fractional bleeds used in HPC to extract 3 bleed flows (LPT exit, HPT exit, HPT in) but no bleeds off FAN or LPC ---*/

    /*--- Splitter ---*/
    double splitter_u[5];  /*--- Inputs:  W, ht, Tt, Pt, FAR ---*/
    double splitter_u1[1]; /*--- Inputs:  BPR ---*/
    double splitter_y[5];  /*--- Outputs: W, ht, Tt, Pt, FAR (Bypass) ---*/
    double splitter_y1[5]; /*--- Outputs: W, ht, Tt, Pt, FAR (Core) ---*/

    /*--- Ducts --- */
    double duct_u[5]; /*--- Inputs:  W, ht, Tt, Pt, FAR ---*/
    double duct_y[5]; /*--- Outputs: W, ht, Tt, Pt, FAR ---*/

    /*--- VBV Valve ---*/
    double vbv_u[5]; /*--- Inputs:  Pt, VBVIn, WIn, TtIn, PtIn ---*/
    double vbv_y[2]; /*--- Outputs: WthOut (valve throat flow), Test ---*/

    /*--- Nozzles ---*/
    double nozzle_u[8]; /*---  Inputs:  W, ht, Tt, Pt, FAR, Pamb, AthroatIn, AexitIn ---*/
    double nozzle_y[17]; /*--- Outputs: W, Fg, Werr, Ath, Ax, Psth, Tsth, MNth, Vth, Psx, Tsx, MNx, Vx, Woutcalc, choked, V_s, Test ---*/

    /*--- StaticCalc ---*/
    double static_u[5]; /*--- Inputs:  W, ht, Tt, Pt, FAR ---*/
    double static_y[5]; /*--- Outputs: Ts, Ps, rhos, MN, Ath ---*/

    /*--- Burner ---*/
    double burner_u[6]; /*--- Inputs:  Wf, W, ht, Tt, Pt, FAR ---*/
    double burner_y[6]; /*--- Outputs: W, ht, Tt, Pt, FAR, Test ---*/

    /*--- Turbines (HPT and LPT) ---*/
    double turbine_u[12];    /*--- Inputs:  Wf, W, ht, Tt, Pt, FAR, Nmech, PR, s_T_Nc, s_T_Wc, s_T_PR, s_T_Eff, cfWidth (cooling Flow vector length) ---*/
    double turbinecool_u[10]; /*--- Inputs:  2 incoming bleed streams x (Wcool, htcool, Ttcool, Ptcool, FARcool) ---*/
    double turbine_y[20];    /*--- Outputs: W, ht, Tt, Pt, FAR, Trq, Werr, C_Nc, C_Wc, C_PR, C_Eff, Wcin, Wcs1in, Nc, NcMap, WcMap, PRmapRead, EffMap, Pwrout, Test ---*/

    /*--- Shafts ---*/
    double shaft_u[3]; /*--- Inputs:  Trq, Pwr (extraction), Nmech ---*/
    double shaft_y[2]; /*--- Outputs: Nmech, Ndot ---*/

    /*--- TSFC Calcuation ---*/
    double SFCCalc_u[3]; /*--- Inputs:  Wf, Fg, Fdrag ---*/
    double SFCCalc_y[2]; /*--- Outputs: SFC, Fnet ---*/

    /*--- Define Output Vector Pointers ---*/
    double *DEP; /*--- Dependents ---*/
    double *X;   /*--- States, x ---*/
    double *U;   /*--- Inputs, u ---*/
    double *Y;   /*--- Outputs, y ---*/
    double *E;   /*--- Miscellaneous outputs, e ---*/

    /*===================================================================*/
    /*--- Define parameters ---*/
    /*===================================================================*/
    /*--- Ambient outputs ---*/
    double W0, ht0, Tt0, Pt0, FAR0, Ps0, Ts0, Veng, Fdrag;

    /*--- Inlet outputs ---*/
    double W2, ht2, Tt2, Pt2, FAR2;

    /*--- Fan outputs ---*/
    double W21, ht21, Tt21, Pt21, FAR21;
    double Trq21, Nerr21, SMavail21, C_Nc21, C_Wc21, C_PR21, C_Eff21, Wcin21, Nc21, PR21, NcMap21, WcMap21, PRMap21, EffMap21, SPR21, Wbleeds21, Pwrb4bleed21, PwrBld21, Pwrout21, SMMap21, SPRMap21;

    /*--- Splitter outputs ---*/
    double W13, ht13, Tt13, Pt13, FAR13; /*--- bypass ---*/
    double W22, ht22, Tt22, Pt22, FAR22; /*--- core ---*/

    /*--- Duct2 outputs ---*/
    double W23, ht23, Tt23, Pt23, FAR23;

    /*--- LPC outputs ---*/
    double W24a, ht24a, Tt24a, Pt24a, FAR24a;
    double Trq24a, Nerr24a, SMavail24a, C_Nc24a, C_Wc24a, C_PR24a, C_Eff24a, Wcin24a, Nc24a, PR24a, NcMap24a, WcMap24a, PRMap24a, EffMap24a, SPR24a, Wbleeds24a, Pwrb4bleed24a, PwrBld24a, Pwrout24a, SMMap24a, SPRMap24a;

    /*--- VBV outputs ---*/
    double W24, ht24, Tt24, Pt24, FAR24; /*--- core ---*/
    double W15, ht15, Tt15, Pt15, FAR15; /*--- bypass ---*/
    double vbv_Wth, vbv_Test;

    /*--- Duct25 outputs ---*/
    double W25, ht25, Tt25, Pt25, FAR25;

    /*--- Duct17 outputs ---*/
    double W17, ht17, Tt17, Pt17, FAR17;

    /*--- Bypass Nozzle outputs ---*/
    double W18, Fg18, NErr18, Ath18, Ax18, Psth18, Tsth18, MNth18, Vth18, Psx18, Tsx18, MNx18, Vx18, Woutcalc18, choked18, Vs18, Nozzle_Test18;

    /*--- HPC outputs ---*/
    double W36, ht36, Tt36, Pt36, FAR36;
    double Trq36, Nerr36, SMavail36, C_Nc36, C_Wc36, C_PR36, C_Eff36, Wcin36, Nc36, PR36, NcMap36, WcMap36, PRMap36, EffMap36, SPR36, Wbleeds36, Pwrb4bleed36, PwrBld36, Pwrout36, SMMap36, SPRMap36;

    /*--- HPC StaticCalc outputs ---*/
    double Ps36, Ts36;

    /*--- Burner outputs ---*/
    double W4, ht4, Tt4, Pt4, FAR4, Test4;

    /*--- HPT outputs ---*/
    double W45, ht45, Tt45, Pt45, FAR45;
    double Trq45, Nerr45, s_T_Nc45, s_T_Wc45, s_T_PR45, s_T_Eff45, Wcin45, Wcs1in45, Nc45, NcMap45, WcMap45, PRMap45, EffMap45, Pwrout45, Test45;

    /*--- Duct45 outputs ---*/
    double W48, ht48, Tt48, Pt48, FAR48;

    /*--- LPT outputs ---*/
    double W5, ht5, Tt5, Pt5, FAR5;
    double Trq5, Nerr5, s_T_Nc5, s_T_Wc5, s_T_PR5, s_T_Eff5, Wcin5, Wcs1in5, Nc5, NcMap5, WcMap5, PRMap5, EffMap5, Pwrout5, Test5;

    /*--- Duct7 outputs ---*/
    double W7, ht7, Tt7, Pt7, FAR7;

    /*--- Core Nozzle outputs ---*/
    double W8, Fg8, NErr8, Ath8, Ax8, Psth8, Tsth8, MNth8, Vth8, Psx8, Tsx8, MNx8, Vx8, Woutcalc8, choked8, Vs8, Nozzle_Test8;

    /*--- LP Shaft outputs ---*/
    double N2dot, N2mechOut;

    /*--- HP Shaft outputs ---*/
    double N3dot, N3mechOut;

    /*--- SFCCalc outputs ---*/
    double Fnet, TSFC;

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

    /*--- Define GTF inlet structure ---*/
    double GTF_inlet_eRambase = 1.0;
    double GTF_inlet_X_eRamVec_M[9] = {0.9, 1, 1.007, 1.028, 1.065, 1.117, 1.276, 1.525, 1.692};
    double GTF_inlet_T_eRamtbl_M[9] = {0.995, 0.995, 0.996, 0.997, 0.997, 0.998, 0.998, 0.998, 0.998};
    char GTF_inlet_BlkNm[11] = "GTF_inlet\0";
    int GTF_inlet_IWork[1] = {0};
    int GTF_inlet_A = 9;
    
    struct InletStruct GTF_inlet = {
        GTF_inlet_eRambase,
        &GTF_inlet_X_eRamVec_M[0],
        &GTF_inlet_T_eRamtbl_M[0],
        &GTF_inlet_BlkNm[0],
        &GTF_inlet_IWork[0],
        GTF_inlet_A,
    };

    /*--- Define GTF fan structure ---*/
    double GTF_fan_NcDes = 1;
    double GTF_fan_PRDes = 1.3;
    double GTF_fan_EffDes = 0.9689;
    double GTF_fan_RlineDes = 2;
    double GTF_fan_IDes = 2;
    double GTF_fan_SMNEn = 0;
    double GTF_fan_CustBldEn = 0;
    double GTF_fan_FBldEn = 0;
    double GTF_fan_CustBldNm = 0;
    double GTF_fan_FracBldNm = 0;
    double GTF_fan_Y_C_Map_NcVec[11] = {0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 0.95, 1, 1.05, 1.1};
    double GTF_fan_X_C_RlineVec[12] = {1, 1.25, 1.5, 1.75, 2, 2.25, 2.5, 2.75, 3, 3.2, 3.25, 3.5};
    double GTF_fan_Z_C_AlphaVec[1] = {0};

    /* IMPORTANT! : The ordering of maps is counter intuitive. The array below 12 Rline Rows x 11 Nc Columns. But further below A Rw = 11 and B Col = 12 */
    double GTF_fan_T_C_Map_WcArray[132] = { 574.9,      839.6,   1104.338,  1369.0782,  1644.4929,  1919.7177,  2191.8401,  2324.3926,  2446.8806,  2564.0947,  2675.1184,
                                             702.9,      975.1,  1247.2229,  1519.3645,  1791.9211,  2054.9785,  2304.7476,  2422.7034,   2530.636,   2632.072,  2726.3958,
                                             830.6,     1107.7,   1384.728,  1661.7891,  1929.5121,  2179.5896,  2407.9941,  2512.5652,  2607.3428,  2694.6523,  2774.0383,
                                             956.5,     1235.6,  1514.7731,  1793.9161,  2054.8865,   2291.583,  2500.2908,  2593.0525,  2676.3726,  2751.4587,  2817.8599,
                                            1079.2,     1357.3,  1635.3656,  1913.4485,  2165.8337,  2389.1497,  2580.4556,  2663.3154,   2737.145,  2802.1399,  2857.6853,
                                            1200.2,     1471.1,  1742.0557,  2012.9872,  2252.1497,  2460.2048,  2636.5833,  2712.5415,  2780.9336,  2840.2256,  2889.2947,
                                            1321.1,     1575.5,  1829.9553,  2084.3818,  2302.7083,  2491.7344,  2655.9902,  2729.2681,  2798.5239,  2858.9136,  2908.2825,
                                            1349.6,     1608.1,  1866.5565,  2125.0225,  2315.6926,  2493.1619,  2655.9902,  2729.2681,  2798.8225,  2860.8069,  2914.2979,
                                            1346.7,     1606.6,  1866.5565,  2126.4814,  2315.6926,  2493.1619,  2655.9902,  2729.2681,  2798.8225,  2860.8069,  2914.2979,
                                            1346.7,     1606.6,  1866.5565,  2126.4814,  2315.6926,  2493.1619,  2655.9902,  2729.2681,  2798.8225,  2860.8069,  2914.2979,
                                            1346.7,     1606.6,  1866.5565,  2126.4814,  2315.6926,  2493.1619,  2655.9902,  2729.2681,  2798.8225,  2860.8069,  2914.2979,
                                            1346.7,     1606.6,  1866.5565,  2126.4814,  2315.6926,  2493.1619,  2655.9902,  2729.2681,  2798.8225,  2860.8069,  2914.2979};
    
    double GTF_fan_T_C_Map_PRArray[132] = {1.0000,  1.0349,  1.0760,  1.1171,  1.1703,  1.2381,  1.3234,  1.3734,  1.4288,  1.4898,  1.5553,
                                            1.0005,  1.0408,  1.0811,  1.1214,  1.1730,  1.2386,  1.3215,  1.3703,  1.4250,  1.4854,  1.5507,
                                            1.0025,  1.0412,  1.0799,  1.1186,  1.1682,  1.2320,  1.3138,  1.3624,  1.4172,  1.4780,  1.5441,
                                            1.0002,  1.0363,  1.0724,  1.1085,  1.1559,  1.2185,  1.3004,  1.3497,  1.4055,  1.4677,  1.5356,
                                            1.0000,  1.0262,  1.0588,  1.0914,  1.1363,  1.1981,  1.2814,  1.3323,  1.3900,  1.4545,  1.5251,
                                            1.0000,  1.0112,  1.0394,  1.0676,  1.1100,  1.1715,  1.2572,  1.3104,  1.3709,  1.4386,  1.5128,
                                            1.0000,  1.0000,  1.0146,  1.0379,  1.0776,  1.1390,  1.2282,  1.2845,  1.3484,  1.4201,  1.4988,
                                            1.0000,  1.0000,  1.0000,  1.0024,  1.0414,  1.1094,  1.2039,  1.2628,  1.3284,  1.4022,  1.4831,
                                            1.0000,  1.0000,  1.0000,  1.0000,  1.0133,  1.0814,  1.1789,  1.2404,  1.3090,  1.3862,  1.4708,
                                            1.0000,  1.0000,  1.0000,  1.0000,  1.0003,  1.0582,  1.1584,  1.2221,  1.2931,  1.3731,  1.4608,
                                            1.0000,  1.0000,  1.0000,  1.0000,  1.0003,  1.0523,  1.1531,  1.2174,  1.2891,  1.3698,  1.4583,
                                            1.0000,  1.0000,  1.0000,  1.0000,  1.0003,  1.0222,  1.1265,  1.1937,  1.2686,  1.3530,  1.4455};
    
    
    double GTF_fan_T_C_Map_EffArray[132] = {   0.5465,  0.6025,  0.6585,  0.7145,  0.7692,  0.8178,  0.8588,  0.8749,  0.8857,  0.8926,  0.8945,
                                                0.7064,  0.7410,  0.7756,  0.8102,  0.8439,  0.8734,  0.8965,  0.9042,  0.9081,  0.9086,  0.9049,
                                                0.8162,  0.8357,  0.8552,  0.8747,  0.8942,  0.9113,  0.9227,  0.9250,  0.9242,  0.9204,  0.9128,
                                                0.8523,  0.8672,  0.8821,  0.8970,  0.9128,  0.9273,  0.9356,  0.9359,  0.9334,  0.9278,  0.9182,
                                                0.7742,  0.8029,  0.8316,  0.8603,  0.8896,  0.9159,  0.9327,  0.9357,  0.9350,  0.9302,  0.9207,
                                                0.5126,  0.5884,  0.6642,  0.7400,  0.8113,  0.8710,  0.9118,  0.9232,  0.9283,  0.9275,  0.9204,
                                                0.0005,  0.1140,  0.3039,  0.4938,  0.6571,  0.7838,  0.8700,  0.8968,  0.9127,  0.9195,  0.9172,
                                                0.0005,  0.0005,  0.0010,  0.0386,  0.4108,  0.6910,  0.8384,  0.8791,  0.9016,  0.9118,  0.9109,
                                                0.0020,  0.0015,  0.0010,  0.0005,  0.1588,  0.5830,  0.7979,  0.8568,  0.8903,  0.9072,  0.9099,
                                                0.0020,  0.0015,  0.0010,  0.0005,  0.0046,  0.4661,  0.7568,  0.8345,  0.8791,  0.9026,  0.9088,
                                                0.0020,  0.0015,  0.0010,  0.0005,  0.0046,  0.4314,  0.7451,  0.8282,  0.8759,  0.9012,  0.9085,
                                                0.0020,  0.0015,  0.0010,  0.0005,  0.0046,  0.2151,  0.6766,  0.7920,  0.8579,  0.8936,  0.9063};
    
    double GTF_fan_FracCusBldht[1] = {0.5};
    double GTF_fan_FracCusBldPt[1] = {0.5};
    double GTF_fan_FracBldht[1] = {0.5};
    double GTF_fan_FracBldPt[1] = {0.5};
    double GTF_fan_X_C_Map_WcSurgeVec[12] = {574.9, 839.6, 1104.338, 1369.0782, 1644.4929, 1919.7177, 2191.8401, 2324.3926, 2446.8806, 2564.0947, 2675.1184, 2914.2979};
    double GTF_fan_T_C_Map_PRSurgeVec[12] = {1, 1.0349, 1.076, 1.1171, 1.1703, 1.2381, 1.3234, 1.3734, 1.4288, 1.4898, 1.5553, 1.69640732438209};
    char   GTF_fan_BlkNm[9] = "GTF_fan\0";
    int    GTF_fan_IWork[5] = {0, 0, 0, 0, 0};
    int    GTF_fan_A = 11;
    int    GTF_fan_B = 12;
    int    GTF_fan_C = 1;
    int    GTF_fan_D = 12;
    int    GTF_fan_WcMapCol = 12;
    int    GTF_fan_PRMapCol = 12;
    int    GTF_fan_EffMapCol = 12;
    int    GTF_fan_WcMapRw = 11;
    int    GTF_fan_PRMapRw = 11;
    int    GTF_fan_EffMapRw = 11;
    int    GTF_fan_WcMapLay = 1;
    int    GTF_fan_PRMapLay = 1;
    int    GTF_fan_EffMapLay = 1;

    /*--- These aren't in fan structure, but you need to initialize them ---*/
    double GTF_fan_Alpha = 0;
    double GTF_fan_s_C_Nc = 2359.983186;
    double GTF_fan_s_C_Wc = 0.768426;
    double GTF_fan_s_C_PR = 0.769231;
    double GTF_fan_s_C_Eff = 1.036257;
    double GTF_fan_Wcust[1] = {0}; 
    double GTF_fan_FracWbld[1] = {0};
    
    struct CompressorStruct GTF_fan = {
        GTF_fan_NcDes,
        GTF_fan_PRDes,
        GTF_fan_EffDes,
        GTF_fan_RlineDes,
        GTF_fan_IDes,
        GTF_fan_SMNEn,
        GTF_fan_CustBldEn,
        GTF_fan_FBldEn,
        GTF_fan_CustBldNm,
        GTF_fan_FracBldNm,
        
        /* Vector & array data */
        &GTF_fan_Y_C_Map_NcVec[0],
        &GTF_fan_X_C_RlineVec[0],
        &GTF_fan_Z_C_AlphaVec[0],
        &GTF_fan_T_C_Map_WcArray[0],
        &GTF_fan_T_C_Map_PRArray[0],
        &GTF_fan_T_C_Map_EffArray[0],
        &GTF_fan_FracCusBldht[0],
        &GTF_fan_FracCusBldPt[0],
        &GTF_fan_FracBldht[0],
        &GTF_fan_FracBldPt[0],
        &GTF_fan_X_C_Map_WcSurgeVec[0],
        &GTF_fan_T_C_Map_PRSurgeVec[0],
        &GTF_fan_BlkNm[0],
        &GTF_fan_IWork[0],
        
        /* Dimensions of parameter arrays */
        GTF_fan_A,
        GTF_fan_B,
        GTF_fan_C,
        GTF_fan_D,
        GTF_fan_WcMapCol,
        GTF_fan_PRMapCol,
        GTF_fan_EffMapCol,
        GTF_fan_WcMapRw,
        GTF_fan_PRMapRw,
        GTF_fan_EffMapRw,
        GTF_fan_WcMapLay,
        GTF_fan_PRMapLay,
        GTF_fan_EffMapLay,
    };

    /*--- Define GTF Duct 2 structure ---*/
    double GTF_duct2_dP_M = 0.01;
    double GTF_duct2_MNdes = 0.45;
    double GTF_duct2_Ath = 286.9;
    char GTF_duct2_BlkNm[10] = "GTF_duct2\0";

    struct DuctStruct GTF_duct2 = {
        GTF_duct2_dP_M,
        GTF_duct2_MNdes,
        GTF_duct2_Ath,
        &GTF_duct2_BlkNm[0],
    };

    /*--- Define GTF lpc structure ---*/
    double GTF_lpc_NcDes = 1.1;
    double GTF_lpc_PRDes = 3;
    double GTF_lpc_EffDes = 0.8894;
    double GTF_lpc_RlineDes = 2.2;
    double GTF_lpc_IDes = 2;
    double GTF_lpc_SMNEn = 0;
    double GTF_lpc_CustBldEn = 0;
    double GTF_lpc_FBldEn = 0;
    double GTF_lpc_CustBldNm = 0;
    double GTF_lpc_FracBldNm = 0;
    double GTF_lpc_Y_C_Map_NcVec[11] = {0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1, 1.1, 1.2, 1.25};
    double GTF_lpc_X_C_RlineVec[12] = {1, 1.2, 1.4, 1.6, 1.8, 2, 2.2, 2.4, 2.6, 2.8, 3, 3.2};
    double GTF_lpc_Z_C_AlphaVec[1] = {0};

    /* IMPORTANT! : The ordering of maps is counter intuitive. The array below 12 Rline Rows x 11 Nc Columns. But further below A Rw = 11 and B Col = 12 */
    double GTF_lpc_T_C_Map_WcArray[132] = { 38.0744, 54.0383, 70.3200,  87.486, 105.8588, 125.1164, 144.4910, 165.9141, 188.5677, 214.1402, 227.8569,
                                            42.9399, 60.0388, 77.5153, 95.6896, 114.8071, 134.6062, 154.5703, 176.2228, 198.3532, 222.1943, 234.5820,
                                            47.7510, 65.9233, 84.4949, 103.5393, 123.2285, 143.3572, 163.6243, 185.1849, 206.6834, 228.9021, 240.1193,
                                            52.5016, 71.6816, 91.2421, 111.0128, 131.0978, 151.3454, 171.6346, 192.7986, 213.5745, 234.2963, 244.5040,
                                            57.1863, 77.3038, 97.7419, 118.0907, 138.3948, 158.5548, 178.5959, 199.0806, 219.0613, 238.4220, 247.7802,
                                            61.7994, 82.7808, 103.9805, 124.7566, 145.1045, 164.9773, 184.5149, 204.0644, 223.1942, 241.3359, 250.0000,
                                            66.3359, 88.1038, 109.9459, 130.9971, 151.2169, 170.6127, 189.4099, 207.7979, 226.0370, 243.1030, 251.2213,
                                            70.7905, 93.2648, 115.6273, 136.8019, 156.7268, 175.4677, 193.3090, 210.3410, 227.6647, 243.7959, 251.5216,
                                            75.1584, 98.2565, 121.0156, 142.1633, 161.6340, 179.5554, 196.2491, 211.7638, 228.1611, 243.8124, 251.5216,
                                            76.5663, 101.0545, 124.6409, 146.2312, 165.7319, 182.8951, 198.2745, 212.1506, 228.1611, 243.8124, 251.5216,
                                            76.5663, 101.0545, 124.6409, 146.2312, 165.7319, 183.0717, 198.4155, 212.1506, 228.1611, 243.8124, 251.5216,
                                            76.5663, 101.0545, 124.6409, 146.2312, 165.7319, 183.0717, 198.4155, 212.1506, 228.1611, 243.8124, 251.5216};
    
    double GTF_lpc_T_C_Map_PRArray[132] = { 1.0423, 1.0760, 1.1215, 1.1789, 1.2494, 1.3353, 1.4411, 1.5724, 1.7323, 1.9360, 2.0507,
                                            1.0412, 1.0738, 1.1180, 1.1738, 1.2422, 1.3253, 1.4282, 1.5561, 1.7101, 1.9056, 2.0158,
                                            1.0393, 1.0704, 1.1127, 1.1660, 1.2312, 1.3105, 1.4088, 1.5313, 1.6785, 1.8662, 1.9729,
                                            1.0367, 1.0658, 1.1055, 1.1555, 1.2167, 1.2910, 1.3830, 1.4982, 1.6379, 1.8184, 1.9223,
                                            1.0333, 1.0600, 1.0965, 1.1423, 1.1986, 1.2669, 1.3512, 1.4572, 1.5888, 1.7625, 1.8645,
                                            1.0292, 1.0530, 1.0856, 1.1266, 1.1771, 1.2384, 1.3136, 1.4088, 1.5318, 1.6991, 1.8000,
                                            1.0234, 1.0434, 1.0707, 1.1052, 1.1481, 1.2002, 1.2632, 1.3440, 1.4572, 1.6190, 1.7201,
                                            1.0151, 1.0297, 1.0497, 1.0753, 1.1078, 1.1476, 1.1942, 1.2556, 1.3572, 1.5142, 1.6176,
                                            1.0043, 1.0122, 1.0228, 1.0374, 1.0572, 1.0822, 1.1088, 1.1472, 1.2358, 1.3887, 1.4958,
                                            1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0056, 1.0101, 1.0233, 1.0982, 1.2471, 1.3584,
                                            1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0944, 1.2098,
                                            1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0546};
    
    double GTF_lpc_T_C_Map_EffArray[132] = {0.7256,  0.7474,  0.7610,  0.7744,  0.7872,  0.7965,  0.7997,  0.8034,  0.8214,  0.8425,  0.8540,
                                            0.7656,  0.7848,  0.7984,  0.8117,  0.8240,  0.8329,  0.8368,  0.8405,  0.8533,  0.8663,  0.8731,
                                            0.7978,  0.8147,  0.8286,  0.8421,  0.8542,  0.8627,  0.8673,  0.8712,  0.8793,  0.8853,  0.8880,
                                            0.8195,  0.8351,  0.8496,  0.8637,  0.8759,  0.8843,  0.8896,  0.8937,  0.8981,  0.8985,  0.8981,
                                            0.8274,  0.8430,  0.8586,  0.8738,  0.8866,  0.8953,  0.9013,  0.9058,  0.9079,  0.9047,  0.9024,
                                            0.8164,  0.8339,  0.8516,  0.8685,  0.8827,  0.8924,  0.8991,  0.9042,  0.9062,  0.9025,  0.9000,
                                            0.7494,  0.7757,  0.7977,  0.8183,  0.8360,  0.8485,  0.8561,  0.8628,  0.8724,  0.8778,  0.8800,
                                            0.5651,  0.6161,  0.6479,  0.6765,  0.7028,  0.7222,  0.7310,  0.7420,  0.7766,  0.8117,  0.8286,
                                            0.1931,  0.3003,  0.3526,  0.3970,  0.4407,  0.4748,  0.4858,  0.5068,  0.5961,  0.6929,  0.7386,
                                            0.0000,  0.0000,  0.0000,  0.0000,  0.0000,  0.0391,  0.0551,  0.0979,  0.2955,  0.5052,  0.6003,
                                            0.0000,  0.0000,  0.0000,  0.0000,  0.0000,  0.0000,  0.0000,  0.0000,  0.0000,  0.2255,  0.4004,
                                            0.0000,  0.0000,  0.0000,  0.0000,  0.0000,  0.0000,  0.0000,  0.0000,  0.0000,  0.0000,  0.1206};
    
    double GTF_lpc_FracCusBldht[1] = {0.5};
    double GTF_lpc_FracCusBldPt[1] = {0.5};
    double GTF_lpc_FracBldht[1] = {0.5};
    double GTF_lpc_FracBldPt[1] = {0.5};
    double GTF_lpc_X_C_Map_WcSurgeVec[12] = {38.0744, 54.0383, 70.32, 87.486, 105.8588, 125.1164, 144.491, 165.9141, 188.5677, 214.1402, 227.8569, 251.5216};
    double GTF_lpc_T_C_Map_PRSurgeVec[12] = {1.0423, 1.076, 1.1215, 1.1789, 1.2494, 1.3353, 1.4411, 1.5724, 1.7323, 1.936, 2.0507, 2.2485858683211};
    char   GTF_lpc_BlkNm[9] = "GTF_lpc\0";
    int    GTF_lpc_IWork[5] = {0, 0, 0, 0, 0};
    int    GTF_lpc_A = 11;
    int    GTF_lpc_B = 12;
    int    GTF_lpc_C = 1;
    int    GTF_lpc_D = 12;
    int    GTF_lpc_WcMapCol = 12;
    int    GTF_lpc_PRMapCol = 12;
    int    GTF_lpc_EffMapCol = 12;
    int    GTF_lpc_WcMapRw = 11;
    int    GTF_lpc_PRMapRw = 11;
    int    GTF_lpc_EffMapRw = 11;
    int    GTF_lpc_WcMapLay = 1;
    int    GTF_lpc_PRMapLay = 1;
    int    GTF_lpc_EffMapLay = 1;

    /*--- These aren't in lpc structure, but you need to initialize them ---*/
    double GTF_lpc_Alpha = 0;
    double GTF_lpc_s_C_Nc = 6398.399938;
    double GTF_lpc_s_C_Wc = 0.300704;
    double GTF_lpc_s_C_PR = 4.374453;
    double GTF_lpc_s_C_Eff = 1.019486;
    double GTF_lpc_Wcust[1] = {0}; 
    double GTF_lpc_FracWbld[1] = {0};
    
    struct CompressorStruct GTF_lpc = {
        GTF_lpc_NcDes,
        GTF_lpc_PRDes,
        GTF_lpc_EffDes,
        GTF_lpc_RlineDes,
        GTF_lpc_IDes,
        GTF_lpc_SMNEn,
        GTF_lpc_CustBldEn,
        GTF_lpc_FBldEn,
        GTF_lpc_CustBldNm,
        GTF_lpc_FracBldNm,
        
        /* Vector & array data */
        &GTF_lpc_Y_C_Map_NcVec[0],
        &GTF_lpc_X_C_RlineVec[0],
        &GTF_lpc_Z_C_AlphaVec[0],
        &GTF_lpc_T_C_Map_WcArray[0],
        &GTF_lpc_T_C_Map_PRArray[0],
        &GTF_lpc_T_C_Map_EffArray[0],
        &GTF_lpc_FracCusBldht[0],
        &GTF_lpc_FracCusBldPt[0],
        &GTF_lpc_FracBldht[0],
        &GTF_lpc_FracBldPt[0],
        &GTF_lpc_X_C_Map_WcSurgeVec[0],
        &GTF_lpc_T_C_Map_PRSurgeVec[0],
        &GTF_lpc_BlkNm[0],
        &GTF_lpc_IWork[0],
        
        /* Dimensions of parameter arrays */
        GTF_lpc_A,
        GTF_lpc_B,
        GTF_lpc_C,
        GTF_lpc_D,
        GTF_lpc_WcMapCol,
        GTF_lpc_PRMapCol,
        GTF_lpc_EffMapCol,
        GTF_lpc_WcMapRw,
        GTF_lpc_PRMapRw,
        GTF_lpc_EffMapRw,
        GTF_lpc_WcMapLay,
        GTF_lpc_PRMapLay,
        GTF_lpc_EffMapLay,
    };

    /*--- Define VBV structure ---*/
    double vbv_VlvfullyOpen = 1.0;
    double vbv_VlvdeadZone = 0.0;
    double vbv_Valve_Ae = 4.0;
    double vbv_X_V_PRVec[4] = {1.0, 1.1, 2.0, 5.0};
    double vbv_T_V_WcVec[4] = {0.0, 3.0, 5.0, 9.9};
    char   vbv_BlkNm[8] = "GTF_VBV\0";
    int    vbv_IWork[1] = {0};
    int    vbv_A = 4;

    struct ValveStruct GTF_vbv = {
        vbv_VlvfullyOpen,
        vbv_VlvdeadZone,
        vbv_Valve_Ae,
        &vbv_X_V_PRVec[0],
        &vbv_T_V_WcVec[0],
        &vbv_BlkNm[0],
        &vbv_IWork[0],
        vbv_A,
    };

    /*--- Define GTF Duct 25 structure ---*/
    double GTF_duct25_dP_M = 0.015;
    double GTF_duct25_MNdes = 0.45;
    double GTF_duct25_Ath = 115.6;
    char GTF_duct25_BlkNm[11] = "GTF_duct25\0";

    struct DuctStruct GTF_duct25 = {
        GTF_duct25_dP_M,
        GTF_duct25_MNdes,
        GTF_duct25_Ath,
        &GTF_duct25_BlkNm[0],
    };

    /*--- Define GTF Duct 17 structure ---*/
    double GTF_duct17_dP_M = 0.015;
    double GTF_duct17_MNdes = 0.45;
    double GTF_duct17_Ath = 6917.7;
    char GTF_duct17_BlkNm[11] = "GTF_duct17\0";

    struct DuctStruct GTF_duct17 = {
        GTF_duct17_dP_M,
        GTF_duct17_MNdes,
        GTF_duct17_Ath,
        &GTF_duct17_BlkNm[0],
    };

    /*--- Define Bypass Nozzle structure ---*/
	double GTF_NozByp_SwitchType = 1; /*  SWType_M - Nozzle Type (1 - Convergent, 2 - Convergent-Divergent) */
	double GTF_NozByp_flowLoss = 0.0;
	double GTF_NozByp_IDes = 2;
	double GTF_NozByp_WDes = 780.95;
	double GTF_NozByp_CfgEn = 1;

	/* Vector & array data */
	double GTF_NozByp_Y_N_FARVec[7] = {0, 0.0050, 0.0100, 0.0150, 0.0200, 0.0250, 0.0300};
	double GTF_NozByp_T_N_RtArray[7] = {0.0686,    0.0686,    0.0686,    0.0686,    0.0686,    0.0686,    0.0686};
	double GTF_NozByp_X_N_TtVec[2] = {300, 10000};
	double GTF_NozByp_T_N_MAP_gammaArray[14] = {1.4, 1.4, 1.4, 1.4, 1.4, 1.4, 1.4, 1.4, 1.4, 1.4, 1.4, 1.4, 1.4, 1.4};
	double GTF_NozByp_X_N_PEQPaVec[2] = {0, 10000};
	double GTF_NozByp_T_N_CdThArray[2] = {1, 1};
	double GTF_NozByp_T_N_CvArray[2] = {1, 1};
	double GTF_NozByp_T_N_CfgArray[2] = {0.9975, 0.9975};
	double GTF_NozByp_T_N_TGArray[2] = {1, 1};
	double GTF_NozByp_X_N_TtVecTG[2] = {300, 10000};
    char   GTF_NozByp_BlkNm[11] = "GTF_NozByp\0";
    int    GTF_NozByp_IWork[16] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

	/* Dimensions of parameter arrays */
    int GTF_NozByp_A = 7;
    int GTF_NozByp_B = 2;
    int GTF_NozByp_B1 = 2;
    int GTF_NozByp_C = 2;

    /*--- These aren't in the nozzle structure, they are in *.bin and will be ignored if iDes = 2, but you need to give them an initial value ---*/
    double GTF_NozByp_N_TArea_M = 1458.15449;
    double GTF_NozByp_N_EArea_M = 1458.15449;

    struct NozzleStruct GTF_NozByp = {
	    GTF_NozByp_SwitchType,
	    GTF_NozByp_flowLoss,
	    GTF_NozByp_IDes,
	    GTF_NozByp_WDes,
	    GTF_NozByp_CfgEn,
    
	    /* Vector & array data */
	    &GTF_NozByp_Y_N_FARVec[0],
	    &GTF_NozByp_T_N_RtArray[0],
	    &GTF_NozByp_X_N_TtVec[0],
	    &GTF_NozByp_T_N_MAP_gammaArray[0],
	    &GTF_NozByp_X_N_PEQPaVec[0],
	    &GTF_NozByp_T_N_CdThArray[0],
	    &GTF_NozByp_T_N_CvArray[0],
	    &GTF_NozByp_T_N_CfgArray[0],
	    &GTF_NozByp_T_N_TGArray[0],
	    &GTF_NozByp_X_N_TtVecTG[0],
        &GTF_NozByp_BlkNm[0],
        &GTF_NozByp_IWork[0],
    
	    /* Dimensions of parameter arrays */
        GTF_NozByp_A,
        GTF_NozByp_B,
        GTF_NozByp_B1,
        GTF_NozByp_C,
    };

    /*--- Define GTF hpc structure ---*/
    double GTF_hpc_NcDes = 1;
    double GTF_hpc_PRDes = 14.103;
    double GTF_hpc_EffDes = 0.8469;
    double GTF_hpc_RlineDes = 2;
    double GTF_hpc_IDes = 2;
    double GTF_hpc_SMNEn = 0;
    double GTF_hpc_CustBldEn = 0;
    double GTF_hpc_FBldEn = 1;
    double GTF_hpc_CustBldNm = 0;
    double GTF_hpc_FracBldNm = 3;
    double GTF_hpc_Y_C_Map_NcVec[13] = {0.5, 0.6, 0.7, 0.75, 0.8, 0.85, 0.9, 0.925, 0.95, 0.975, 1, 1.025, 1.05};
    double GTF_hpc_X_C_RlineVec[11] = {1, 1.2, 1.4, 1.6, 1.8, 2, 2.2, 2.4, 2.6, 2.8, 3};
    double GTF_hpc_Z_C_AlphaVec[1] = {0};

    /* IMPORTANT! : The ordering of maps is counter intuitive. The array below 11 Rline Rows x 13 Nc Columns. But further below A Rw = 13 and B Col = 11 */
    double GTF_hpc_T_C_Map_WcArray[143] = { 22.7411, 31.7548, 46.1066, 56.7268, 70.1448, 89.3764, 118.0620, 138.5093, 160.6243, 181.7993, 202.6315, 209.9986, 216.6847,
                                            24.0487, 33.1181, 47.4088, 58.0480, 71.5163, 90.9746, 120.1207, 140.8966, 162.5676, 183.4993, 203.5858, 210.5917, 217.0279,
                                            25.1548, 34.2670, 48.5066, 59.1608, 72.6688, 92.3098, 121.8253, 142.8639, 164.1805, 184.9150, 204.3958, 211.1029, 217.3287,
                                            26.0615, 35.2054, 49.4046, 60.0704, 73.6088, 93.3900, 123.1867, 144.4238, 165.4722, 186.0545, 205.0661, 211.5321, 217.5860,
                                            26.7738, 35.9397, 50.1096, 60.7837, 74.3429, 94.2232, 124.2166, 145.5916, 166.4536, 186.9260, 205.5998, 211.8825, 217.8015,
                                            27.2992, 36.4783, 50.6291, 61.3084, 74.8795, 94.8199, 124.9292, 146.3836, 167.1370, 187.5389, 206.0000, 212.1554, 217.9767,
                                            27.6470, 36.8308, 50.9717, 61.6527, 75.2269, 95.1897, 125.3385, 146.8174, 167.5334, 187.9029, 206.2702, 212.3516, 218.1106,
                                            27.8286, 37.0085, 51.1469, 61.8260, 75.3943, 95.3442, 125.4609, 146.9192, 167.6563, 188.0273, 206.4145, 212.4735, 218.2041,
                                            27.8634, 37.0362, 51.1757, 61.8517, 75.4134, 95.3504, 125.4609, 146.9192, 167.6563, 188.0271, 206.4418, 212.5220, 218.2586,
                                            27.8634, 37.0362, 51.1757, 61.8517, 75.4134, 95.3504, 125.4609, 146.9192, 167.6563, 188.0271, 206.4418, 212.5227, 218.2739,
                                            27.8634, 37.0362, 51.1757, 61.8517, 75.4134, 95.3504, 125.4609, 146.9192, 167.6563, 188.0271, 206.4418, 212.5227, 218.2739};
    
    double GTF_hpc_T_C_Map_PRArray[143] = { 2.4769, 3.4633, 5.0821, 6.3490, 8.0021, 10.4899, 14.4564, 17.4426, 20.7403, 23.8298, 26.6962, 27.6439, 28.4663,
                                            2.4288, 3.3778, 4.9375, 6.1658, 7.7686, 10.1976, 14.0970, 17.0500, 20.2486, 23.2601, 26.0933, 27.0687, 27.9667,
                                            2.3620, 3.2643, 4.7554, 5.9371, 7.4792, 9.8249, 13.6074, 16.4870, 19.6093, 22.5536, 25.4175, 26.4522, 27.4460,
                                            2.2778, 3.1248, 4.5391, 5.6667, 7.1388, 9.3786, 12.9977, 15.7661, 18.8329, 21.7200, 24.6733, 25.7969, 26.9054,
                                            2.1774, 2.9619, 4.2923, 5.3594, 6.7532, 8.8669, 12.2808, 14.9034, 17.9324, 20.7705, 23.8656, 25.1052, 26.3460,
                                            2.0627, 2.7787, 4.0194, 5.0204, 6.3287, 8.2989, 11.4715, 13.9183, 16.9227, 19.7178, 22.9999, 24.3798, 25.7690,
                                            1.9284, 2.5679, 3.7106, 4.6377, 5.8504, 7.6539, 10.5377, 12.7692, 15.7626, 18.5212, 22.0495, 23.6033, 25.1640,
                                            1.7711, 2.3253, 3.3602, 4.2042, 5.3097, 6.9201, 9.4621, 11.4347, 14.4263, 17.1524, 20.9930, 22.7614, 24.5225,
                                            1.5958, 2.0595, 2.9800, 3.7342, 4.7237, 6.1229, 8.2878, 9.9718, 12.9562, 15.6466, 19.8436, 21.8600, 23.8472,
                                            1.4083, 1.7802, 2.5826, 3.2431, 4.1114, 5.2899, 7.0614, 8.4425, 11.3983, 14.0424, 18.6163, 20.9058, 23.1399,
                                            1.2146, 1.4973, 2.1813, 2.7467, 3.4919, 4.4495, 5.8306, 6.9104, 9.8011, 12.3810, 17.3267, 19.9057, 22.4038};
    
    double GTF_hpc_T_C_Map_EffArray[143] = {0.6753, 0.6953, 0.7248, 0.7427, 0.7634, 0.7891, 0.8139, 0.8206, 0.8403, 0.8408, 0.8470, 0.8350, 0.8202,
                                            0.6913, 0.7094, 0.7359, 0.7533, 0.7736, 0.8008, 0.8280, 0.8356, 0.8512, 0.8492, 0.8505, 0.8364, 0.8203,
                                            0.7016, 0.7184, 0.7429, 0.7600, 0.7804, 0.8090, 0.8385, 0.8469, 0.8593, 0.8552, 0.8529, 0.8371, 0.8201,
                                            0.7050, 0.7214, 0.7452, 0.7627, 0.7834, 0.8134, 0.8449, 0.8541, 0.8643, 0.8588, 0.8539, 0.8370, 0.8195,
                                            0.7004, 0.7176, 0.7424, 0.7606, 0.7822, 0.8136, 0.8469, 0.8567, 0.8660, 0.8597, 0.8536, 0.8362, 0.8185,
                                            0.6864, 0.7058, 0.7335, 0.7533, 0.7762, 0.8092, 0.8439, 0.8544, 0.8641, 0.8578, 0.8520, 0.8346, 0.8171,
                                            0.6570, 0.6812, 0.7154, 0.7379, 0.7630, 0.7974, 0.8333, 0.8442, 0.8566, 0.8516, 0.8483, 0.8318, 0.8152,
                                            0.6044, 0.6378, 0.6838, 0.7108, 0.7394, 0.7754, 0.8117, 0.8229, 0.8415, 0.8394, 0.8418, 0.8275, 0.8124,
                                            0.5236, 0.5717, 0.6366, 0.6703, 0.7041, 0.7417, 0.7779, 0.7892, 0.8179, 0.8209, 0.8324, 0.8217, 0.8088,
                                            0.4075, 0.4783, 0.5713, 0.6147, 0.6556, 0.6950, 0.7303, 0.7416, 0.7852, 0.7954, 0.8200, 0.8141, 0.8045,
                                            0.2467, 0.3512, 0.4848, 0.5414, 0.5920, 0.6335, 0.6671, 0.6783, 0.7423, 0.7624, 0.8043, 0.8049, 0.7992};
    
    double GTF_hpc_FracCusBldht[1] = {0.5};
    double GTF_hpc_FracCusBldPt[1] = {0.5};
    double GTF_hpc_FracBldht[3] = {0.4997, 1, 1};
    double GTF_hpc_FracBldPt[3] = {0.146498, 1, 1};
    double GTF_hpc_X_C_Map_WcSurgeVec[14] = {22.7411, 31.7548, 46.1066, 56.7268, 70.1448, 89.3764, 118.062, 138.5093, 160.6243, 181.7993, 202.6315, 209.9986, 216.6847, 218.2739};
    double GTF_hpc_T_C_Map_PRSurgeVec[14] = {2.4769, 3.4633, 5.0821, 6.349, 8.0021, 10.4899, 14.4564, 17.4426, 20.7403, 23.8298, 26.6962, 27.6439, 28.4663, 28.6617739055653};
    char   GTF_hpc_BlkNm[8] = "GTF_hpc\0";
    int    GTF_hpc_IWork[5] = {0, 0, 0, 0, 0};
    int    GTF_hpc_A = 13;
    int    GTF_hpc_B = 11;
    int    GTF_hpc_C = 1;
    int    GTF_hpc_D = 14;
    int    GTF_hpc_WcMapCol = 11;
    int    GTF_hpc_PRMapCol = 11;
    int    GTF_hpc_EffMapCol = 11;
    int    GTF_hpc_WcMapRw = 13;
    int    GTF_hpc_PRMapRw = 13;
    int    GTF_hpc_EffMapRw = 13;
    int    GTF_hpc_WcMapLay = 1;
    int    GTF_hpc_PRMapLay = 1;
    int    GTF_hpc_EffMapLay = 1;

    /*--- These aren't in hpc structure, but you need to initialize them ---*/
    double GTF_hpc_Alpha = 0;
    double GTF_hpc_s_C_Nc = 18242.834381;
    double GTF_hpc_s_C_Wc = 0.1328;
    double GTF_hpc_s_C_PR = 0.595594;
    double GTF_hpc_s_C_Eff = 0.994014;
    double GTF_hpc_Wcust[1] = {0}; 
    double GTF_hpc_FracWbld[3] = {0.02, 0.0693, 0.0625};
    
    struct CompressorStruct GTF_hpc = {
        GTF_hpc_NcDes,
        GTF_hpc_PRDes,
        GTF_hpc_EffDes,
        GTF_hpc_RlineDes,
        GTF_hpc_IDes,
        GTF_hpc_SMNEn,
        GTF_hpc_CustBldEn,
        GTF_hpc_FBldEn,
        GTF_hpc_CustBldNm,
        GTF_hpc_FracBldNm,
        
        /* Vector & array data */
        &GTF_hpc_Y_C_Map_NcVec[0],
        &GTF_hpc_X_C_RlineVec[0],
        &GTF_hpc_Z_C_AlphaVec[0],
        &GTF_hpc_T_C_Map_WcArray[0],
        &GTF_hpc_T_C_Map_PRArray[0],
        &GTF_hpc_T_C_Map_EffArray[0],
        &GTF_hpc_FracCusBldht[0],
        &GTF_hpc_FracCusBldPt[0],
        &GTF_hpc_FracBldht[0],
        &GTF_hpc_FracBldPt[0],
        &GTF_hpc_X_C_Map_WcSurgeVec[0],
        &GTF_hpc_T_C_Map_PRSurgeVec[0],
        &GTF_hpc_BlkNm[0],
        &GTF_hpc_IWork[0],
        
        /* Dimensions of parameter arrays */
        GTF_hpc_A,
        GTF_hpc_B,
        GTF_hpc_C,
        GTF_hpc_D,
        GTF_hpc_WcMapCol,
        GTF_hpc_PRMapCol,
        GTF_hpc_EffMapCol,
        GTF_hpc_WcMapRw,
        GTF_hpc_PRMapRw,
        GTF_hpc_EffMapRw,
        GTF_hpc_WcMapLay,
        GTF_hpc_PRMapLay,
        GTF_hpc_EffMapLay,
    };

    /*--- Define GTF HPC StaticCalc structure ---*/
    double GTF_hpcstatic_AthroatIn = 17.2;
    double GTF_hpcstatic_MNIn = 0.3;
    int GTF_hpcstatic_SolveType = 0;
    double GTF_hpcstatic_X_FARVec[7] = {0, 0.0050, 0.0100, 0.0150, 0.0200, 0.0250, 0.0300};
    double GTF_hpcstatic_T_RtArray[7] = {0.0686, 0.0686, 0.0686, 0.0686, 0.0686, 0.0686, 0.0686};
    double GTF_hpcstatic_Y_TtVec[7] = {300, 10000};
    double GTF_hpcstatic_T_gammaArray[14] = {1.4, 1.4, 1.4, 1.4, 1.4, 1.4, 1.4, 1.4, 1.4, 1.4, 1.4, 1.4, 1.4, 1.4};
    int GTF_hpcstatic_IWork[5] = {0, 0, 0, 0, 0};
    int  GTF_hpcstatic_A = 7;
    int  GTF_hpcstatic_B = 2;
    struct StaticCalcStruct GTF_hpcstatic = {
        GTF_hpcstatic_AthroatIn,
        GTF_hpcstatic_MNIn,
        GTF_hpcstatic_SolveType,
        &GTF_hpcstatic_X_FARVec[0],
        &GTF_hpcstatic_T_RtArray[0],
        &GTF_hpcstatic_Y_TtVec[0],
        &GTF_hpcstatic_T_gammaArray[0],
        &GTF_hpc_BlkNm[0],
        &GTF_hpcstatic_IWork[0],
        GTF_hpcstatic_A,
        GTF_hpcstatic_B,
    };

    /*--- Define GTF Burner structure ---*/
    double GTF_burner_LHV = 18400.0;
    double GTF_burner_dPqP = 0.04;
    double GTF_burner_Eff = 0.999;
    double GTF_LHVEn = 1.0;
    double GTF_hFuel = -1200.0;

    struct BurnStruct GTF_burner = {
        GTF_burner_LHV,
        GTF_burner_dPqP,
        GTF_burner_Eff,
        GTF_LHVEn,
        GTF_hFuel,
    };

    /*--- Define GTF HPT structure ---*/
    /* Turbine block parameters structure */
    double GTF_hpt_s_T_Nc = 3.708724;
    double GTF_hpt_s_T_Wc = 0.1951;
    double GTF_hpt_s_T_PR = 0.773898;
    double GTF_hpt_s_T_Eff = 0.998392;
    double GTF_hpt_NcDes = 100;
    double GTF_hpt_PRmapDes = 5;
    double GTF_hpt_EffDes = 0.9313;
    double GTF_hpt_NDes = 20871;
    double GTF_hpt_IDes = 2.0;
    int    GTF_hpt_BldPosLeng = 2;
    int    GTF_hpt_CoolFlwEn = 1;
    int    GTF_hpt_ConfigNPSS = 1;
    double GTF_hpt_Y_T_NcVec[8] = {60, 70, 80, 90, 100, 110, 120, 130};
    double GTF_hpt_X_T_PRVec[20] = {3.0, 3.25, 3.5, 3.75, 4.0, 4.25, 4.5, 4.75, 5.0, 5.25, 5.5, 5.75, 6.0, 6.25, 6.5, 6.75, 7.0, 7.25, 7.5, 8.0};
    
    double GTF_hpt_T_T_Map_WcArray[160] = { 30.446, 30.299, 30.120, 30.014, 29.856, 29.799, 29.742, 29.685,
                                            30.533, 30.413, 30.239, 30.124, 29.948, 29.870, 29.792, 29.714,
                                            30.568, 30.480, 30.317, 30.201, 30.013, 29.920, 29.827, 29.734,
                                            30.572, 30.516, 30.368, 30.253, 30.059, 29.955, 29.851, 29.747,
                                            30.572, 30.529, 30.398, 30.288, 30.091, 29.979, 29.867, 29.755,
                                            30.572, 30.530, 30.415, 30.311, 30.113, 29.997, 29.881, 29.765,
                                            30.572, 30.530, 30.421, 30.325, 30.128, 30.009, 29.890, 29.771,
                                            30.572, 30.530, 30.421, 30.333, 30.139, 30.017, 29.895, 29.773,
                                            30.572, 30.530, 30.421, 30.337, 30.145, 30.023, 29.901, 29.779,
                                            30.572, 30.530, 30.421, 30.337, 30.149, 30.026, 29.903, 29.780,
                                            30.572, 30.530, 30.421, 30.337, 30.150, 30.028, 29.906, 29.784,
                                            30.572, 30.530, 30.421, 30.337, 30.150, 30.029, 29.908, 29.787,
                                            30.572, 30.530, 30.421, 30.337, 30.150, 30.029, 29.908, 29.787,
                                            30.572, 30.530, 30.421, 30.337, 30.150, 30.029, 29.908, 29.787,
                                            30.572, 30.530, 30.421, 30.337, 30.150, 30.029, 29.908, 29.787,
                                            30.572, 30.530, 30.421, 30.337, 30.150, 30.029, 29.908, 29.787,
                                            30.572, 30.530, 30.421, 30.337, 30.150, 30.029, 29.908, 29.787,
                                            30.572, 30.530, 30.421, 30.337, 30.150, 30.029, 29.908, 29.787,
                                            30.572, 30.530, 30.421, 30.337, 30.150, 30.029, 29.908, 29.787,
                                            30.572, 30.530, 30.421, 30.337, 30.150, 30.029, 29.908, 29.787};

    double GTF_hpt_T_T_Map_EffArray[160] = {0.8460, 0.8879, 0.9125, 0.9228, 0.9215, 0.9106, 0.8997, 0.8888,
                                            0.8405, 0.8842, 0.9111, 0.9242, 0.9258, 0.9176, 0.9094, 0.9012,
                                            0.8349, 0.8804, 0.9096, 0.9250, 0.9289, 0.9232, 0.9175, 0.9118,
                                            0.8296, 0.8769, 0.9078, 0.9247, 0.9304, 0.9267, 0.9230, 0.9193,
                                            0.8249, 0.8735, 0.9055, 0.9240, 0.9313, 0.9292, 0.9271, 0.9250,
                                            0.8206, 0.8701, 0.9034, 0.9232, 0.9319, 0.9312, 0.9305, 0.9298,
                                            0.8165, 0.8670, 0.9014, 0.9224, 0.9323, 0.9327, 0.9331, 0.9335,
                                            0.8127, 0.8640, 0.8995, 0.9217, 0.9326, 0.9340, 0.9354, 0.9368,
                                            0.8092, 0.8614, 0.8979, 0.9210, 0.9328, 0.9351, 0.9374, 0.9397,
                                            0.8060, 0.8590, 0.8964, 0.9203, 0.9329, 0.9361, 0.9393, 0.9425,
                                            0.8030, 0.8568, 0.8950, 0.9197, 0.9330, 0.9369, 0.9408, 0.9447,
                                            0.8002, 0.8548, 0.8936, 0.9188, 0.9311, 0.9349, 0.9387, 0.9425,
                                            0.7976, 0.8529, 0.8924, 0.9162, 0.9288, 0.9329, 0.9370, 0.9411,
                                            0.7953, 0.8511, 0.8903, 0.9137, 0.9266, 0.9311, 0.9356, 0.9401,
                                            0.7931, 0.8495, 0.8877, 0.9113, 0.9245, 0.9293, 0.9341, 0.9389,
                                            0.7911, 0.8479, 0.8853, 0.9091, 0.9225, 0.9276, 0.9327, 0.9378,
                                            0.7892, 0.8460, 0.8830, 0.9070, 0.9206, 0.9259, 0.9312, 0.9365,
                                            0.7875, 0.8436, 0.8808, 0.9050, 0.9188, 0.9239, 0.9290, 0.9341,
                                            0.7858, 0.8414, 0.8787, 0.9031, 0.9161, 0.9212, 0.9263, 0.9314,
                                            0.7826, 0.8373, 0.8749, 0.8980, 0.9107, 0.9161, 0.9215, 0.9269};
    
    double GTF_hpt_T_BldPos[2] = {1.0, 0.0};
    char   GTF_hpt_BlkNm[8] = {"GTF_hpt\0"};
    int    GTF_hpt_IWork[5] = {0, 0, 0, 0, 0};
    int    GTF_hpt_A = 8;
    int    GTF_hpt_B = 20;
    int    GTF_hpt_WcMapCol = 20;
    int    GTF_hpt_EffMapCol = 20;
    int    GTF_hpt_WcMapRw = 8;
    int    GTF_hpt_EffMapRw = 8;

    /*--- These aren't in hpt structure, but you need to initialize them ---*/
    double GTF_hpt_cfWidth = 10; /*--- this will be #bleeds x 5 ---*/

    struct TurbineStruct GTF_hpt = {
        GTF_hpt_s_T_Nc, 
        GTF_hpt_s_T_PR,
        GTF_hpt_s_T_Wc,
        GTF_hpt_s_T_Eff,
        GTF_hpt_NcDes,
        GTF_hpt_PRmapDes,
        GTF_hpt_EffDes,
        GTF_hpt_NDes,
        GTF_hpt_IDes,
        GTF_hpt_BldPosLeng,
        GTF_hpt_CoolFlwEn,
        GTF_hpt_ConfigNPSS,
        &GTF_hpt_Y_T_NcVec[0],
        &GTF_hpt_X_T_PRVec[0],
        &GTF_hpt_T_T_Map_WcArray[0],
        &GTF_hpt_T_T_Map_EffArray[0],
        &GTF_hpt_T_BldPos[0],
        &GTF_hpt_BlkNm[0],
        &GTF_hpt_IWork[0],
        GTF_hpt_A,
        GTF_hpt_B,
        GTF_hpt_WcMapCol,
        GTF_hpt_EffMapCol,
        GTF_hpt_WcMapRw,
        GTF_hpt_EffMapRw,
    };

    /*--- Define GTF Duct 45 structure ---*/
    double GTF_duct45_dP_M = 0.005;
    double GTF_duct45_MNdes = 0.3;
    double GTF_duct45_Ath = 66.3;
    char GTF_duct45_BlkNm[11] = "GTF_duct45\0";

    struct DuctStruct GTF_duct45 = {
        GTF_duct45_dP_M,
        GTF_duct45_MNdes,
        GTF_duct45_Ath,
        &GTF_duct45_BlkNm[0],
    };

    /*--- Define GTF LPT structure ---*/
    /* Turbine block parameters structure */
    double GTF_lpt_s_T_Nc = 1.430948;
    double GTF_lpt_s_T_Wc = 0.1573;
    double GTF_lpt_s_T_PR = 1.540753;
    double GTF_lpt_s_T_Eff = 1.028756;
    double GTF_lpt_NcDes = 100;
    double GTF_lpt_PRmapDes = 7.5;
    double GTF_lpt_EffDes = 0.9409;
    double GTF_lpt_NDes = 6772;
    double GTF_lpt_IDes = 2.0;
    int    GTF_lpt_BldPosLeng = 1;
    int    GTF_lpt_CoolFlwEn = 1;
    int    GTF_lpt_ConfigNPSS = 1;
    double GTF_lpt_Y_T_NcVec[11] = {20, 30, 40, 50, 60, 70, 80, 90, 100, 110, 120};
    double GTF_lpt_X_T_PRVec[28] = {1.0, 1.25, 1.5, 1.75, 2.0, 2.25, 2.5, 2.75, 3.0, 3.25, 3.5, 3.75, 4.0, 4.25, 4.5, 4.75, 5.0, 5.25, 5.5, 5.75, 6.0, 6.25, 6.5, 6.75, 7.0, 7.25, 7.5, 8.0};
    
    double GTF_lpt_T_T_Map_WcArray[308] = { 155.016, 154.715, 154.414, 154.113, 153.812, 153.511, 151.335, 148.427, 145.903, 142.728, 138.719,
                                            155.016, 154.715, 154.414, 154.113, 153.812, 153.511, 151.518, 148.748, 146.259, 143.056, 138.987,
                                            155.016, 154.715, 154.414, 154.113, 153.812, 153.511, 151.701, 149.069, 146.615, 143.384, 139.255,
                                            155.016, 154.715, 154.414, 154.113, 153.812, 153.511, 151.884, 149.390, 146.971, 143.712, 139.523,
                                            155.016, 154.715, 154.414, 154.113, 153.812, 153.511, 152.067, 149.711, 147.327, 144.040, 139.791,
                                            155.016, 154.715, 154.414, 154.113, 153.812, 153.511,  152.25, 150.032, 147.683, 144.368, 140.059,
                                            155.016, 154.715, 154.414, 154.113, 153.812, 153.511, 152.433, 150.353, 148.039, 144.696, 140.327,
                                            155.016, 154.715, 154.414, 154.113, 153.812, 153.511, 152.616, 150.674, 148.395, 145.024, 140.595,
                                            155.016, 154.715, 154.414, 154.113, 153.812, 153.511, 152.799, 150.995, 148.751, 145.352, 140.863,
                                            155.016, 154.715, 154.414, 154.113, 153.812, 153.511, 152.982, 151.316, 149.107, 145.680, 141.131,
                                            155.016, 154.715, 154.414, 154.113, 153.812, 153.511, 153.052, 151.518, 149.349, 145.905, 141.310,
                                            155.016, 154.715, 154.414, 154.113, 153.812, 153.511, 153.061, 151.647, 149.517, 146.061, 141.428,
                                            155.016, 154.715, 154.414, 154.113, 153.812, 153.511, 153.061, 151.729, 149.635, 146.169, 141.503,
                                            155.016, 154.715, 154.414, 154.113, 153.812, 153.511, 153.061, 151.781, 149.719, 146.244, 141.547,
                                            155.016, 154.715, 154.414, 154.113, 153.812, 153.511, 153.061, 151.814, 149.779, 146.293, 141.567,
                                            155.016, 154.715, 154.414, 154.113, 153.812, 153.511, 153.061, 151.834, 149.822, 146.324, 141.569,
                                            155.016, 154.715, 154.414, 154.113, 153.812, 153.511, 153.061, 151.846, 149.852, 146.339, 141.569,
                                            155.016, 154.715, 154.414, 154.113, 153.812, 153.511, 153.061, 151.852, 149.872, 146.344, 141.569,
                                            155.016, 154.715, 154.414, 154.113, 153.812, 153.511, 153.061, 151.856, 149.885, 146.344, 141.569,
                                            155.016, 154.715, 154.414, 154.113, 153.812, 153.511, 153.061, 151.858, 149.894, 146.344, 141.569,
                                            155.016, 154.715, 154.414, 154.113, 153.812, 153.511, 153.061, 151.859, 149.898, 146.344, 141.569,
                                            155.016, 154.715, 154.414, 154.113, 153.812, 153.511, 153.061, 151.859, 149.899, 146.344, 141.569,
                                            155.016, 154.715, 154.414, 154.113, 153.812, 153.511, 153.061, 151.859, 149.899, 146.344, 141.569,
                                            155.016, 154.715, 154.414, 154.113, 153.812, 153.511, 153.061, 151.859, 149.899, 146.344, 141.569,
                                            155.016, 154.715, 154.414, 154.113, 153.812, 153.511, 153.061, 151.859, 149.899, 146.344, 141.569,
                                            155.016, 154.715, 154.414, 154.113, 153.812, 153.511, 153.061, 151.859, 149.899, 146.344, 141.569,
                                            155.016, 154.715, 154.414, 154.113, 153.812, 153.511, 153.061, 151.859, 149.899, 146.344, 141.569,
                                            155.016, 154.715, 154.414, 154.113, 153.812, 153.511, 153.061, 151.859, 149.899, 146.344, 141.569};
    
    double GTF_lpt_T_T_Map_EffArray[308] = {0.7508, 0.7886, 0.8264, 0.8642, 0.9020, 0.9398, 0.9593, 0.9549, 0.9383, 0.9103, 0.8727,
                                            0.7373, 0.7765, 0.8157, 0.8549, 0.8941, 0.9333, 0.9544, 0.9528, 0.9391, 0.9142, 0.8798,
                                            0.7238, 0.7644, 0.8050, 0.8456, 0.8862, 0.9268, 0.9495, 0.9507, 0.9399, 0.9181, 0.8869,
                                            0.7103, 0.7523, 0.7943, 0.8363, 0.8783, 0.9203, 0.9446, 0.9486, 0.9407, 0.9220, 0.8940,
                                            0.6968, 0.7402, 0.7836, 0.8270, 0.8704, 0.9138, 0.9397, 0.9465, 0.9415, 0.9259, 0.9011,
                                            0.6833, 0.7281, 0.7729, 0.8177, 0.8625, 0.9073, 0.9348, 0.9444, 0.9423, 0.9298, 0.9082,
                                            0.6698, 0.7160, 0.7622, 0.8084, 0.8546, 0.9008, 0.9299, 0.9423, 0.9431, 0.9337, 0.9153,
                                            0.6563, 0.7039, 0.7515, 0.7991, 0.8467, 0.8943, 0.9250, 0.9402, 0.9439, 0.9376, 0.9224,
                                            0.6428, 0.6918, 0.7408, 0.7898, 0.8388, 0.8878, 0.9201, 0.9381, 0.9447, 0.9415, 0.9295,
                                            0.6293, 0.6797, 0.7301, 0.7805, 0.8309, 0.8813, 0.9152, 0.9360, 0.9455, 0.9454, 0.9366,
                                            0.6190, 0.6701, 0.7212, 0.7723, 0.8234, 0.8745, 0.9105, 0.9336, 0.9456, 0.9479, 0.9419,
                                            0.6055, 0.6581, 0.7107, 0.7633, 0.8159, 0.8685, 0.9061, 0.9310, 0.9450, 0.9495, 0.9458,
                                            0.5939, 0.6477, 0.7015, 0.7553, 0.8091, 0.8629, 0.9018, 0.9283, 0.9440, 0.9504, 0.9487,
                                            0.5842, 0.6389, 0.6936, 0.7483, 0.8030, 0.8577, 0.8978, 0.9257, 0.9429, 0.9510, 0.9509,
                                            0.5763, 0.6316, 0.6869, 0.7422, 0.7975, 0.8528, 0.8940, 0.9231, 0.9417, 0.9512, 0.9526,
                                            0.5684, 0.6244, 0.6804, 0.7364, 0.7924, 0.8484, 0.8905, 0.9206, 0.9404, 0.9511, 0.9538,
                                            0.5608, 0.6175, 0.6742, 0.7309, 0.7876, 0.8443, 0.8872, 0.9182, 0.9383, 0.9492, 0.9528,
                                            0.5544, 0.6116, 0.6688, 0.7260, 0.7832, 0.8404, 0.8840, 0.9153, 0.9355, 0.9472, 0.9517,
                                            0.5483, 0.6060, 0.6637, 0.7214, 0.7791, 0.8368, 0.8810, 0.9119, 0.9327, 0.9452, 0.9505,
                                            0.5429, 0.6010, 0.6591, 0.7172, 0.7753, 0.8334, 0.8776, 0.9087, 0.9301, 0.9433, 0.9493,
                                            0.5377, 0.5962, 0.6547, 0.7132, 0.7717, 0.8302, 0.8741, 0.9056, 0.9276, 0.9414, 0.9481,
                                            0.5332, 0.5920, 0.6508, 0.7096, 0.7684, 0.8272, 0.8707, 0.9027, 0.9252, 0.9396, 0.9468,
                                            0.5292, 0.5882, 0.6472, 0.7062, 0.7652, 0.8242, 0.8676, 0.8999, 0.9229, 0.9378, 0.9456,
                                            0.5275, 0.5862, 0.6449, 0.7036, 0.7623, 0.8210, 0.8646, 0.8973, 0.9207, 0.9361, 0.9444,
                                            0.5259, 0.5843, 0.6427, 0.7011, 0.7595, 0.8179, 0.8618, 0.8948, 0.9186, 0.9344, 0.9432,
                                            0.5240, 0.5822, 0.6404, 0.6986, 0.7568, 0.8150, 0.8590, 0.8924, 0.9165, 0.9326, 0.9413,
                                            0.5222, 0.5802, 0.6382, 0.6962, 0.7542, 0.8122, 0.8565, 0.8901, 0.9146, 0.9304, 0.9395,
                                            0.5191, 0.5767, 0.6343, 0.6919, 0.7495, 0.8071, 0.8516, 0.8858, 0.9099, 0.9262, 0.9360};
    
    double GTF_lpt_T_BldPos[1] = {1.0};
    char   GTF_lpt_BlkNm[8] = {"GTF_lpt\0"};
    int    GTF_lpt_IWork[5] = {0, 0, 0, 0, 0};
    int    GTF_lpt_A = 11;
    int    GTF_lpt_B = 28;
    int    GTF_lpt_WcMapCol = 28;
    int    GTF_lpt_EffMapCol = 28;
    int    GTF_lpt_WcMapRw = 11;
    int    GTF_lpt_EffMapRw = 11;

    /*--- These aren't in hpt structure, but you need to initialize them ---*/
    double GTF_lpt_cfWidth = 5; /*--- this will be #bleeds x 5 ---*/

    struct TurbineStruct GTF_lpt = {
        GTF_lpt_s_T_Nc, 
        GTF_lpt_s_T_PR,
        GTF_lpt_s_T_Wc,
        GTF_lpt_s_T_Eff,
        GTF_lpt_NcDes,
        GTF_lpt_PRmapDes,
        GTF_lpt_EffDes,
        GTF_lpt_NDes,
        GTF_lpt_IDes,
        GTF_lpt_BldPosLeng,
        GTF_lpt_CoolFlwEn,
        GTF_lpt_ConfigNPSS,
        &GTF_lpt_Y_T_NcVec[0],
        &GTF_lpt_X_T_PRVec[0],
        &GTF_lpt_T_T_Map_WcArray[0],
        &GTF_lpt_T_T_Map_EffArray[0],
        &GTF_lpt_T_BldPos[0],
        &GTF_lpt_BlkNm[0],
        &GTF_lpt_IWork[0],
        GTF_lpt_A,
        GTF_lpt_B,
        GTF_lpt_WcMapCol,
        GTF_lpt_EffMapCol,
        GTF_lpt_WcMapRw,
        GTF_lpt_EffMapRw,
    };    

    /*--- Define GTF Duct 5 structure ---*/
    double GTF_duct5_dP_M = 0.01;
    double GTF_duct5_MNdes = 0.35;
    double GTF_duct5_Ath = 945;
    char GTF_duct5_BlkNm[10] = "GTF_duct5\0";

    struct DuctStruct GTF_duct5 = {
        GTF_duct5_dP_M,
        GTF_duct5_MNdes,
        GTF_duct5_Ath,
        &GTF_duct5_BlkNm[0],
    };

    /*--- Define Core Nozzle structure ---*/
	double GTF_NozCor_SwitchType = 1; /*  SWType_M - Nozzle Type (1 - Convergent, 2 - Convergent-Divergent) */
	double GTF_NozCor_flowLoss = 0.0;
	double GTF_NozCor_IDes = 2;
	double GTF_NozCor_WDes = 33.34;
	double GTF_NozCor_CfgEn = 1;

	/* Vector & array data */
	double GTF_NozCor_Y_N_FARVec[7] = {0, 0.0050, 0.0100, 0.0150, 0.0200, 0.0250, 0.0300};
	double GTF_NozCor_T_N_RtArray[7] = {0.0686,    0.0686,    0.0686,    0.0686,    0.0686,    0.0686,    0.0686};
	double GTF_NozCor_X_N_TtVec[2] = {300, 10000};
	double GTF_NozCor_T_N_MAP_gammaArray[14] = {1.4, 1.4, 1.4, 1.4, 1.4, 1.4, 1.4, 1.4, 1.4, 1.4, 1.4, 1.4, 1.4, 1.4};
	double GTF_NozCor_X_N_PEQPaVec[2] = {0, 10000};
	double GTF_NozCor_T_N_CdThArray[2] = {1, 1};
	double GTF_NozCor_T_N_CvArray[2] = {1, 1};
	double GTF_NozCor_T_N_CfgArray[2] = {0.9999, 0.9999};
	double GTF_NozCor_T_N_TGArray[2] = {1, 1};
	double GTF_NozCor_X_N_TtVecTG[2] = {300, 10000};
    char   GTF_NozCor_BlkNm[11] = "GTF_NozCor\0";
    int    GTF_NozCor_IWork[16] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

	/* Dimensions of parameter arrays */
    int GTF_NozCor_A = 7;
    int GTF_NozCor_B = 2;
    int GTF_NozCor_B1 = 2;
    int GTF_NozCor_C = 2;

    /*--- These aren't in the nozzle structure, they are in *.bin and will be ignored if iDes = 2, but you need to give them an initial value ---*/
    double GTF_NozCor_N_TArea_M = 393.43;
    double GTF_NozCor_N_EArea_M = 110.7;

    struct NozzleStruct GTF_NozCor = {
	    GTF_NozCor_SwitchType,
	    GTF_NozCor_flowLoss,
	    GTF_NozCor_IDes,
	    GTF_NozCor_WDes,
	    GTF_NozCor_CfgEn,
    
	    /* Vector & array data */
	    &GTF_NozCor_Y_N_FARVec[0],
	    &GTF_NozCor_T_N_RtArray[0],
	    &GTF_NozCor_X_N_TtVec[0],
	    &GTF_NozCor_T_N_MAP_gammaArray[0],
	    &GTF_NozCor_X_N_PEQPaVec[0],
	    &GTF_NozCor_T_N_CdThArray[0],
	    &GTF_NozCor_T_N_CvArray[0],
	    &GTF_NozCor_T_N_CfgArray[0],
	    &GTF_NozCor_T_N_TGArray[0],
	    &GTF_NozCor_X_N_TtVecTG[0],
        &GTF_NozCor_BlkNm[0],
        &GTF_NozCor_IWork[0],
    
	    /* Dimensions of parameter arrays */
        GTF_NozCor_A,
        GTF_NozCor_B,
        GTF_NozCor_B1,
        GTF_NozCor_C,
    };

    /*--- Define GTF Gearbox ---*/
    double GTF_gearbox_GearRatio = 3.1;
    double GTF_gearbox_Eff = 1.0;

    /*--- Define GTF LP Shaft structure ---*/
    double GTF_lpshaft_Inertia_M = 17.44087229;
    double GTF_lpshaft_Eff = 0.99;

    struct ShaftStruct GTF_lpshaft = {
        GTF_lpshaft_Inertia_M,
    };

    /*--- Define GTF HP Shaft structure ---*/
    double GTF_hpshaft_Inertia_M = 1.86055038;
    
    struct ShaftStruct GTF_hpshaft = {
        GTF_hpshaft_Inertia_M,
    };

    /*===================================================================*/
    /*===================================================================*/
    /* Check for proper number of arguments. */
    if (nrhs != 4) {
    mexErrMsgTxt("3 inputs to MEX engine model required");
    } else if (nlhs != 5) {
    mexErrMsgTxt("5 output arguments to MEX engine model required");
    }
    
    m = mxGetM(ENV_IN); 
    n = mxGetN(ENV_IN);
    if (!mxIsDouble(ENV_IN) || mxIsComplex(ENV_IN) || 
	(MAX(m,n) != 3) || (MIN(m,n) != 1)) { 
	mexErrMsgTxt("Requires that ENV_IN be a 3 x 1 vector."); 
    } 

    m = mxGetM(CMD_IN); 
    n = mxGetN(CMD_IN);
    if (!mxIsDouble(CMD_IN) || mxIsComplex(CMD_IN) || 
	(MAX(m,n) != 14) || (MIN(m,n) != 1)) { 
	mexErrMsgTxt("Requires that CMD_IN be a 14 x 1 vector."); 
    } 

    m = mxGetM(TAR_OUT); 
    n = mxGetN(TAR_OUT);
    if (!mxIsDouble(TAR_OUT) || mxIsComplex(TAR_OUT) || 
	(MAX(m,n) != 3) || (MIN(m,n) != 1)) { 
	mexErrMsgTxt("Requires that TAR_OUT be a 3 x 1 vector."); 
    } 

    m = mxGetM(HEALTH_PARAMS_IN); 
    n = mxGetN(HEALTH_PARAMS_IN);
    if (!mxIsDouble(HEALTH_PARAMS_IN) || mxIsComplex(HEALTH_PARAMS_IN) || 
	(MAX(m,n) != 13) || (MIN(m,n) != 1)) { 
	mexErrMsgTxt("Requires that HEALTH_PARAMS_IN be a 13 x 1 vector."); 
    } 
    
    /* Create a matrix for the return argument */ 
    DEP_OUT = mxCreateDoubleMatrix(12, 1, mxREAL); 
    X_OUT = mxCreateDoubleMatrix(2, 1, mxREAL); 
    U_OUT = mxCreateDoubleMatrix(3, 1, mxREAL);
    Y_OUT = mxCreateDoubleMatrix(64, 1, mxREAL);
    E_OUT = mxCreateDoubleMatrix(13, 1, mxREAL); 

    /* Assign pointers to the various I/O parameters */ 
    DEP = mxGetPr(DEP_OUT);
    X = mxGetPr(X_OUT);
    Y = mxGetPr(Y_OUT);
    U = mxGetPr(U_OUT);
    E = mxGetPr(E_OUT);

    /*--- Environmental inputs ---*/
    env = mxGetPr(ENV_IN);
    AltIn    = env[0]; /* Altitude */
    MNIn     = env[1]; /* Mach */
    dTambIn  = env[2]; /* dTamb */

    /*--- Independent and command inputs ---*/
    cmd = mxGetPr(CMD_IN);
    WIn      = cmd[0]; /* Mass flow */
    FAN_RLIn = cmd[1]; /* Fan Rline */
    LPC_RLIn = cmd[2]; /* LPC Rline */
    HPC_RLIn = cmd[3]; /* HPC Rline */
    BPRIn    = cmd[4]; /* Bypass Ratio */
    HPT_PRIn = cmd[5]; /* HPT PRIn */
    LPT_PRIn = cmd[6]; /* LPT PRIn */
    WfIn     = cmd[7]; /* Fuel flow */
    VAFNIn   = cmd[8]; /* VAFN position */
    VBVIn    = cmd[9]; /* VBV position */
    N2In     = cmd[10]; /* LP shaft speed */
    N3In     = cmd[11]; /* HP shaft speed */
    HPpwrIn  = cmd[12]; /* HP shaft power extraction */
    LPpwrIn  = cmd[13]; /* LP shaft power extraction */

    /*--- Target outputs ---*/
    tar = mxGetPr(TAR_OUT);
    LPC_SM_target  = tar[0];
    Fnet_target  = tar[1];
    T45_target = tar[2];

    /*--- Health Parameters ---*/
    health_params  = mxGetPr(HEALTH_PARAMS_IN);
    // Fan
    GTF_fan_WcMod  = health_params[0];  
    GTF_fan_PRMod  = health_params[1];
    GTF_fan_EffMod = health_params[2];
    GTF_fan_hp_En  = 1;
    // LPC
    GTF_lpc_WcMod  = health_params[3];  
    GTF_lpc_PRMod  = health_params[4];
    GTF_lpc_EffMod = health_params[5];
    GTF_lpc_hp_En  = 1;
    // HPC
    GTF_hpc_WcMod  = health_params[6];  
    GTF_hpc_PRMod  = health_params[7];
    GTF_hpc_EffMod = health_params[8];
    GTF_hpc_hp_En  = 1;
    // HPT
    GTF_hpt_WcMod  = health_params[9]; 
    GTF_hpt_EffMod = health_params[10];
    GTF_hpt_hp_En  = 1;
    // LPT
    GTF_lpt_WcMod  = health_params[11]; 
    GTF_lpt_EffMod = health_params[12];
    GTF_lpt_hp_En  = 1;

    /*--- Ambient ---*/
    amb_u[0] = AltIn;
    amb_u[1] = dTambIn;
    amb_u[2] = MNIn;

    Ambient_TMATS_body(&amb_y[0], &amb_u[0], &GTF_ambient);
    W0 = WIn;
    ht0 = amb_y[0];
    Tt0 = amb_y[1];
    Pt0 = amb_y[2];
    FAR0 = amb_y[3];
    Ps0 = amb_y[4];
    Ts0 = amb_y[5];
    Veng = amb_y[6];

    // For this calculation only, the old Simulink model used value of gravity with fewer signficiant digits, 
    // leading to minor disagreement at Fdrag calculation. After rectifying the issue the two models agree.
    Fdrag = WIn * Veng / C_GRAVITY;

    /*--- Inlet ---*/
    inlet_u[0] = W0;
    inlet_u[1] = ht0;
    inlet_u[2] = Tt0;
    inlet_u[3] = Pt0;
    inlet_u[4] = FAR0;
    inlet_u[5] = Ps0;

    Inlet_TMATS_body(&inlet_y[0], &inlet_u[0], &GTF_inlet);
    W2 = inlet_y[0];
    ht2 = inlet_y[1];
    Tt2 = inlet_y[2];
    Pt2 = inlet_y[3];
    FAR2 = inlet_y[4];

    /*--- Fan --- */
    compressor_u[0] = W2;
    compressor_u[1] = ht2;
    compressor_u[2] = Tt2;
    compressor_u[3] = Pt2;
    compressor_u[4] = FAR2;
    compressor_u[5] = N2In / GTF_gearbox_GearRatio; /* Nmech */
    compressor_u[6] = FAN_RLIn; /*Rline; */

    compressor_u[7] = GTF_fan_Alpha;
    compressor_u[8] = GTF_fan_s_C_Nc;
    compressor_u[9] = GTF_fan_s_C_Wc * (1 + GTF_fan_WcMod * GTF_fan_hp_En);
    compressor_u[10] = GTF_fan_s_C_PR * (1 + GTF_fan_PRMod * GTF_fan_hp_En);
    compressor_u[11] = GTF_fan_s_C_Eff * (1 + GTF_fan_EffMod * GTF_fan_hp_En);

    Compressor_TMATS_body(&compressor_y[0], &compressor_y1[0], &compressor_y2[0], &compressor_u[0], &GTF_fan_Wcust[0], &GTF_fan_FracWbld[0], &GTF_fan);
    W21 = compressor_y[0];
    ht21 = compressor_y[1];
    Tt21 = compressor_y[2];
    Pt21 = compressor_y[3];
    FAR21 = compressor_y[4];
    Trq21 = compressor_y[5];
    Nerr21 = compressor_y[6];
    SMavail21 = compressor_y[7];
    C_Nc21 = compressor_y[8];            /* Corrected shaft speed scalar */
    C_Wc21 = compressor_y[9];             /* Corrected flow scalar */
    C_PR21 = compressor_y[10];            /* Pressure Ratio scalar */
    C_Eff21 = compressor_y[11];          /* Efficiency scalar */
    Wcin21 = compressor_y[12];           /* Corrected input flow [pps] */
    Nc21 = compressor_y[13];             /* Corrected speed [rpm]*/
    PR21 = compressor_y[14];             /* Pressure ratio */
    NcMap21 = compressor_y[15];          /* Map corrected speed */
    WcMap21 = compressor_y[16];          /* Map corrected flow */
    PRMap21 = compressor_y[17];          /* Map pressure ratio */
    EffMap21 = compressor_y[18];         /* Map efficiency */
    SPR21 = compressor_y[19];            /* Surge pressure ratio */
    Wbleeds21 = compressor_y[20];        /* Bleed flow [pps]*/
    Pwrb4bleed21 = compressor_y[21];     /* Power if there was no bleed [hp]*/
    PwrBld21 = compressor_y[22];         /* Power loss due to bleed [hp] */
    Pwrout21 = compressor_y[23];         /* Output power [hp]*/
    SMMap21 = compressor_y[24];          /* Stall margin calculated from map values [%]*/
    SPRMap21 = compressor_y[25];         /* Map stall pressure ratio*/    

    /*--- Splitter --- */
    splitter_u[0] = W21;
    splitter_u[1] = ht21;
    splitter_u[2] = Tt21;
    splitter_u[3] = Pt21;
    splitter_u[4] = FAR21;

    splitter_u1[0] = BPRIn;

    Splitter_TMATS(&splitter_y[0], &splitter_y1[0], &splitter_u[0], &splitter_u1[0]);
    W13 = splitter_y[0];
    ht13 = splitter_y[1];
    Tt13 = splitter_y[2];
    Pt13 = splitter_y[3];
    FAR13 = splitter_y[4];

    W22 = splitter_y1[0];
    ht22 = splitter_y1[1];
    Tt22 = splitter_y1[2];
    Pt22 = splitter_y1[3];
    FAR22 = splitter_y1[4];

    /*--- Duct 2 (between splitter and LPC)---*/
    duct_u[0] = W22;
    duct_u[1] = ht22;
    duct_u[2] = Tt22;
    duct_u[3] = Pt22;
    duct_u[4] = FAR22;
    Duct_TMATS_body(&duct_y[0],&duct_u[0],&GTF_duct2);
    W23 = duct_y[0];
    ht23 = duct_y[1];
    Tt23 = duct_y[2];
    Pt23 = duct_y[3];
    FAR23 = duct_y[4];

    /*--- LPC --- */
    compressor_u[0] = W23;
    compressor_u[1] = ht23;
    compressor_u[2] = Tt23;
    compressor_u[3] = Pt23;
    compressor_u[4] = FAR23;
    compressor_u[5] = N2In; /* Nmech */
    compressor_u[6] = LPC_RLIn; /*Rline; */

    compressor_u[7] = GTF_lpc_Alpha;
    compressor_u[8] = GTF_lpc_s_C_Nc;
    compressor_u[9] = GTF_lpc_s_C_Wc * (1 + GTF_lpc_WcMod * GTF_lpc_hp_En);
    compressor_u[10] = GTF_lpc_s_C_PR * (1 + GTF_lpc_PRMod * GTF_lpc_hp_En);
    compressor_u[11] = GTF_lpc_s_C_Eff * (1 + GTF_lpc_EffMod * GTF_lpc_hp_En);

    Compressor_TMATS_body(&compressor_y[0], &compressor_y1[0], &compressor_y2[0], &compressor_u[0], &GTF_lpc_Wcust[0], &GTF_lpc_FracWbld[0], &GTF_lpc);
    W24a = compressor_y[0];
    ht24a = compressor_y[1];
    Tt24a = compressor_y[2];
    Pt24a = compressor_y[3];
    FAR24a = compressor_y[4];
    Trq24a = compressor_y[5];
    Nerr24a = compressor_y[6];
    SMavail24a = compressor_y[7];
    C_Nc24a = compressor_y[8];            /* Corrected shaft speed scalar */
    C_Wc24a = compressor_y[9];             /* Corrected flow scalar */
    C_PR24a = compressor_y[10];            /* Pressure Ratio scalar */
    C_Eff24a = compressor_y[11];          /* Efficiency scalar */
    Wcin24a = compressor_y[12];           /* Corrected input flow [pps] */
    Nc24a = compressor_y[13];             /* Corrected speed [rpm]*/
    PR24a = compressor_y[14];             /* Pressure ratio */
    NcMap24a = compressor_y[15];          /* Map corrected speed */
    WcMap24a = compressor_y[16];          /* Map corrected flow */
    PRMap24a = compressor_y[17];          /* Map pressure ratio */
    EffMap24a = compressor_y[18];         /* Map efficiency */
    SPR24a = compressor_y[19];            /* Surge pressure ratio */
    Wbleeds24a = compressor_y[20];        /* Bleed flow [pps]*/
    Pwrb4bleed24a = compressor_y[21];     /* Power if there was no bleed [hp]*/
    PwrBld24a = compressor_y[22];         /* Power loss due to bleed [hp] */
    Pwrout24a = compressor_y[23];         /* Output power [hp]*/
    SMMap24a = compressor_y[24];          /* Stall margin calculated from map values [%]*/
    SPRMap24a = compressor_y[25];         /* Map stall pressure ratio*/ 

    /*--- VBV ---*/
    vbv_u[0] = Pt13;
    vbv_u[1] = VBVIn;
    vbv_u[2] = W24a;
    vbv_u[3] = Tt24a;
    vbv_u[4] = Pt24a;

    Valve_TMATS_body(&vbv_y[0], &vbv_u[0], &GTF_vbv);
    vbv_Wth = vbv_y[0];
    vbv_Test = vbv_y[1];

    W24 = W24a - vbv_Wth; /*--- core flow aft of VBV ---*/
    ht24 = ht24a;
    Tt24 = Tt24a;
    Pt24 = Pt24a;
    FAR24 = FAR24a;

    W15 = W13 + vbv_Wth; /*--- bypass flow aft of VBV ---*/
    ht15 = ht13;
    Tt15 = Tt13;
    Pt15 = Pt13;
    FAR15 = FAR13;

    /*--- Duct 25 (aft of LPC and VBV) ---*/
    duct_u[0] = W24;
    duct_u[1] = ht24;
    duct_u[2] = Tt24;
    duct_u[3] = Pt24;
    duct_u[4] = FAR24;
    Duct_TMATS_body(&duct_y[0],&duct_u[0],&GTF_duct25);
    W25 = duct_y[0];
    ht25 = duct_y[1];
    Tt25 = duct_y[2];
    Pt25 = duct_y[3];
    FAR25 = duct_y[4];

    /*--- Duct 17 (in bypass) ---*/
    duct_u[0] = W15;
    duct_u[1] = ht15;
    duct_u[2] = Tt15;
    duct_u[3] = Pt15;
    duct_u[4] = FAR15;
    Duct_TMATS_body(&duct_y[0],&duct_u[0],&GTF_duct17);
    W17 = duct_y[0];
    ht17 = duct_y[1];
    Tt17 = duct_y[2];
    Pt17 = duct_y[3];
    FAR17 = duct_y[4];

    /*--- Bypass Nozzle ---*/
    nozzle_u[0] = W17;
    nozzle_u[1] = ht17;
    nozzle_u[2] = Tt17;
    nozzle_u[3] = Pt17;
    nozzle_u[4] = FAR17;
    nozzle_u[5] = Ps0;

    if (GTF_NozByp_IDes < 1.5)
    {
        nozzle_u[6] = GTF_NozByp_N_TArea_M;
        nozzle_u[7] = GTF_NozByp_N_EArea_M;
    }
    else
    {
        nozzle_u[6] = VAFNIn;
        nozzle_u[7] = VAFNIn;
    }

    Nozzle_TMATS_body(&nozzle_y[0], &nozzle_u[0], &GTF_NozByp);
    W18 = nozzle_y[0];
    Fg18 = nozzle_y[1];
    NErr18 = nozzle_y[2];
    Ath18 = nozzle_y[3];
    Ax18 = nozzle_y[4];
    Psth18 = nozzle_y[5];
    Tsth18 = nozzle_y[6];
    MNth18 = nozzle_y[7];
    Vth18 = nozzle_y[8];
    Psx18 = nozzle_y[9];
    Tsx18 = nozzle_y[10];
    MNx18 = nozzle_y[11];
    Vx18 = nozzle_y[12];
    Woutcalc18 = nozzle_y[13];
    choked18 = nozzle_y[14];
    Vs18 = nozzle_y[15];
    Nozzle_Test18 = nozzle_y[16];

    /*--- HPC --- */
    compressor_u[0] = W25;
    compressor_u[1] = ht25;
    compressor_u[2] = Tt25;
    compressor_u[3] = Pt25;
    compressor_u[4] = FAR25;
    compressor_u[5] = N3In; /* Nmech */
    compressor_u[6] = HPC_RLIn; /*Rline; */

    compressor_u[7] = GTF_hpc_Alpha;
    compressor_u[8] = GTF_hpc_s_C_Nc;
    compressor_u[9] = GTF_hpc_s_C_Wc * (1 + GTF_hpc_WcMod * GTF_hpc_hp_En);
    compressor_u[10] = GTF_hpc_s_C_PR * (1 + GTF_hpc_PRMod * GTF_hpc_hp_En);
    compressor_u[11] = GTF_hpc_s_C_Eff * (1 + GTF_hpc_EffMod * GTF_hpc_hp_En);

    Compressor_TMATS_body(&compressor_y[0], &compressor_y1[0], &compressor_y2[0], &compressor_u[0], &GTF_hpc_Wcust[0], &GTF_hpc_FracWbld[0], &GTF_hpc);
    W36 = compressor_y[0];
    ht36 = compressor_y[1];
    Tt36 = compressor_y[2];
    Pt36 = compressor_y[3];
    FAR36 = compressor_y[4];
    Trq36 = compressor_y[5];
    Nerr36 = compressor_y[6];
    SMavail36 = compressor_y[7];
    C_Nc36 = compressor_y[8];            /* Corrected shaft speed scalar */
    C_Wc36 = compressor_y[9];             /* Corrected flow scalar */
    C_PR36 = compressor_y[10];            /* Pressure Ratio scalar */
    C_Eff36 = compressor_y[11];          /* Efficiency scalar */
    Wcin36 = compressor_y[12];           /* Corrected input flow [pps] */
    Nc36 = compressor_y[13];             /* Corrected speed [rpm]*/
    PR36 = compressor_y[14];             /* Pressure ratio */
    NcMap36 = compressor_y[15];          /* Map corrected speed */
    WcMap36 = compressor_y[16];          /* Map corrected flow */
    PRMap36 = compressor_y[17];          /* Map pressure ratio */
    EffMap36 = compressor_y[18];         /* Map efficiency */
    SPR36 = compressor_y[19];            /* Surge pressure ratio */
    Wbleeds36 = compressor_y[20];        /* Bleed flow [pps]*/
    Pwrb4bleed36 = compressor_y[21];     /* Power if there was no bleed [hp]*/
    PwrBld36 = compressor_y[22];         /* Power loss due to bleed [hp] */
    Pwrout36 = compressor_y[23];         /* Output power [hp]*/
    SMMap36 = compressor_y[24];          /* Stall margin calculated from map values [%]*/
    SPRMap36 = compressor_y[25];         /* Map stall pressure ratio*/ 

    /*---- call StaticCalc to Station 36 Ps and Ts --- */
    static_u[0] = W36;
    static_u[1] = ht36;
    static_u[2] = Tt36;
    static_u[3] = Pt36;
    static_u[4] = FAR36;
    StaticCalc_TMATS_body(&static_y[0], &compressor_y[0], &GTF_hpcstatic);
    Ts36 = static_y[0];
    Ps36 = static_y[1]; 

    /*--- Burner ---*/
    burner_u[0] = WfIn;
    burner_u[1] = W36;
    burner_u[2] = ht36;
    burner_u[3] = Tt36;
    burner_u[4] = Pt36;
    burner_u[5] = FAR36;
    Burner_TMATS_body(&burner_y[0],&burner_u[0],&GTF_burner);
    W4 = burner_y[0];
    ht4 = burner_y[1];
    Tt4 = burner_y[2];
    Pt4 = burner_y[3];
    FAR4 = burner_y[4];
    Test4 = burner_y[5];

    /*--- HPT ---*/
    turbine_u[0] = W4;
    turbine_u[1] = ht4;
    turbine_u[2] = Tt4;
    turbine_u[3] = Pt4;
    turbine_u[4] = FAR4;
    turbine_u[5] = N3In;
    turbine_u[6] = HPT_PRIn;
    turbine_u[7] = GTF_hpt_s_T_Nc;
    turbine_u[8] = GTF_hpt_s_T_Wc * (1 + GTF_hpt_WcMod * GTF_hpt_hp_En);
    turbine_u[9] = GTF_hpt_s_T_PR;
    turbine_u[10] = GTF_hpt_s_T_Eff * (1 + GTF_hpt_EffMod * GTF_hpt_hp_En);
    turbine_u[11] = GTF_hpt_cfWidth;
    
    turbinecool_u[0] = compressor_y2[5];
    turbinecool_u[1] = compressor_y2[6];
    turbinecool_u[2] = compressor_y2[7];
    turbinecool_u[3] = compressor_y2[8];
    turbinecool_u[4] = compressor_y2[9];
    turbinecool_u[5] = compressor_y2[10];
    turbinecool_u[6] = compressor_y2[11];
    turbinecool_u[7] = compressor_y2[12];
    turbinecool_u[8] = compressor_y2[13];
    turbinecool_u[9] = compressor_y2[14];
    
    Turbine_TMATS_body(&turbine_y[0], &turbine_u[0], &turbinecool_u[0], &GTF_hpt);

    W45 = turbine_y[0];
    ht45 = turbine_y[1];
    Tt45 = turbine_y[2];
    Pt45 = turbine_y[3];
    FAR45 = turbine_y[4];
    Trq45 = turbine_y[5];
    Nerr45 = turbine_y[6];
    s_T_Nc45 = turbine_y[7];
    s_T_Wc45 = turbine_y[8];
    s_T_PR45 = turbine_y[9];
    s_T_Eff45 = turbine_y[10];
    Wcin45 = turbine_y[11];
    Wcs1in45 = turbine_y[12];
    Nc45 = turbine_y[13];
    NcMap45 = turbine_y[14];
    WcMap45 = turbine_y[15];
    PRMap45 = turbine_y[16];
    EffMap45 = turbine_y[17];
    Pwrout45 = turbine_y[18];
    Test45 = turbine_y[19];

    /*--- Duct 48 (between HPT and LPT) ---*/
    duct_u[0] = W45;
    duct_u[1] = ht45;
    duct_u[2] = Tt45;
    duct_u[3] = Pt45;
    duct_u[4] = FAR45;
    Duct_TMATS_body(&duct_y[0],&duct_u[0],&GTF_duct45);
    W48 = duct_y[0];
    ht48 = duct_y[1];
    Tt48 = duct_y[2];
    Pt48 = duct_y[3];
    FAR48 = duct_y[4];

    /*--- LPT ---*/
    turbine_u[0] = W48;
    turbine_u[1] = ht48;
    turbine_u[2] = Tt48;
    turbine_u[3] = Pt48;
    turbine_u[4] = FAR48;
    turbine_u[5] = N2In;
    turbine_u[6] = LPT_PRIn;
    turbine_u[7] = GTF_lpt_s_T_Nc;
    turbine_u[8] = GTF_lpt_s_T_Wc * (1 + GTF_lpt_WcMod * GTF_lpt_hp_En);
    turbine_u[9] = GTF_lpt_s_T_PR;
    turbine_u[10] = GTF_lpt_s_T_Eff * (1 + GTF_lpt_EffMod * GTF_lpt_hp_En);
    turbine_u[11] = GTF_lpt_cfWidth;
    
    turbinecool_u[0] = compressor_y2[0];
    turbinecool_u[1] = compressor_y2[1];
    turbinecool_u[2] = compressor_y2[2];
    turbinecool_u[3] = compressor_y2[3];
    turbinecool_u[4] = compressor_y2[4];
    
    Turbine_TMATS_body(&turbine_y[0], &turbine_u[0], &turbinecool_u[0], &GTF_lpt);

    W5 = turbine_y[0];
    ht5 = turbine_y[1];
    Tt5 = turbine_y[2];
    Pt5 = turbine_y[3];
    FAR5 = turbine_y[4];
    Trq5 = turbine_y[5];
    Nerr5 = turbine_y[6];
    s_T_Nc5 = turbine_y[7];
    s_T_Wc5 = turbine_y[8];
    s_T_PR5 = turbine_y[9];
    s_T_Eff5 = turbine_y[10];
    Wcin5 = turbine_y[11];
    Wcs1in5 = turbine_y[12];
    Nc5 = turbine_y[13];
    NcMap5 = turbine_y[14];
    WcMap5 = turbine_y[15];
    PRMap5 = turbine_y[16];
    EffMap5 = turbine_y[17];
    Pwrout5 = turbine_y[18];
    Test5 = turbine_y[19];

    /*--- Duct 5 (between HPT and LPT)---*/
    duct_u[0] = W5;
    duct_u[1] = ht5;
    duct_u[2] = Tt5;
    duct_u[3] = Pt5;
    duct_u[4] = FAR5;
    Duct_TMATS_body(&duct_y[0],&duct_u[0],&GTF_duct5);
    W7 = duct_y[0];
    ht7 = duct_y[1];
    Tt7 = duct_y[2];
    Pt7 = duct_y[3];
    FAR7 = duct_y[4];
    
    /*--- Core Nozzle ---*/
    nozzle_u[0] = W7;
    nozzle_u[1] = ht7;
    nozzle_u[2] = Tt7;
    nozzle_u[3] = Pt7;
    nozzle_u[4] = FAR7;
    nozzle_u[5] = Ps0;
    nozzle_u[6] = GTF_NozCor_N_TArea_M;
    nozzle_u[7] = GTF_NozCor_N_EArea_M;
    Nozzle_TMATS_body(&nozzle_y[0], &nozzle_u[0], &GTF_NozCor);
    W8 = nozzle_y[0];
    Fg8 = nozzle_y[1];
    NErr8 = nozzle_y[2];
    Ath8 = nozzle_y[3];
    Ax8 = nozzle_y[4];
    Psth8 = nozzle_y[5];
    Tsth8 = nozzle_y[6];
    MNth8 = nozzle_y[7];
    Vth8 = nozzle_y[8];
    Psx8 = nozzle_y[9];
    Tsx8 = nozzle_y[10];
    MNx8 = nozzle_y[11];
    Vx8 = nozzle_y[12];
    Woutcalc8 = nozzle_y[13];
    choked8 = nozzle_y[14];
    Vs8 = nozzle_y[15];
    Nozzle_Test8 = nozzle_y[16];

    /*--- LP Shaft ----*/
    shaft_u[0] = (Trq21 / GTF_gearbox_GearRatio) + Trq24a + (Trq5 * GTF_lpshaft_Eff); /*--- Fan, LPC, LPt ---*/
    shaft_u[1] = LPpwrIn;
    shaft_u[2] = N2In;
    Shaft_TMATS_body(&shaft_y[0],&shaft_u[0],&GTF_lpshaft);
    N2mechOut = shaft_y[0];
    N2dot = shaft_y[1];

    /*--- HP Shaft ----*/
    shaft_u[0] = Trq36 + Trq45; /*--- HPC, HPT ---*/
    shaft_u[1] = HPpwrIn;
    shaft_u[2] = N3In;
    Shaft_TMATS_body(&shaft_y[0],&shaft_u[0],&GTF_hpshaft);
    N3mechOut = shaft_y[0];
    N3dot = shaft_y[1];

    /*--- SFCCalc ---*/
    SFCCalc_u[0] = WfIn; /*--- Wf fuel flow ---*/
    SFCCalc_u[1] = Fg18 + Fg8; /*--- bypass and core gross thrust ---*/
    SFCCalc_u[2] = Fdrag;
    SFCCalc_TMATS(&SFCCalc_y[0], &SFCCalc_u[0]);
    TSFC = SFCCalc_y[0];
    Fnet = SFCCalc_y[1];

    /* ================================================================= */
    /*--- Assign Outputs ---*/
    /* ================================================================= */
    // Dependent variables
    DEP[0] = Nerr21;
    DEP[1] = Nerr24a;
    DEP[2] = Nerr36;
    DEP[3] = Nerr45;
    DEP[4] = Nerr5;
    DEP[5] = NErr8;
    DEP[6] = NErr18;
    DEP[7] = N2dot;
    DEP[8] = N3dot;
    DEP[9] = SMMap24a - LPC_SM_target;
    DEP[10] = Fnet - Fnet_target;
    DEP[11] = Tt45 - T45_target;

    // States
    X[0] = N2mechOut;
    X[1] = N3mechOut;

    // Inputs
    U[0] = WfIn;
    U[1] = HPpwrIn;
    U[2] = LPpwrIn;

    // Outputs
    Y[0] = N2mechOut/GTF_gearbox_GearRatio;
    Y[1] = N2mechOut;
    Y[2] = N3mechOut;
    Y[3] = W0;
    Y[4] = Tt0;
    Y[5] = Pt0;
    Y[6] = W2;
    Y[7] = Tt2;
    Y[8] = Pt2;
    Y[9] = W21;
    Y[10] = Tt21;
    Y[11] = Pt21;
    Y[12] = W13;
    Y[13] = Tt13;
    Y[14] = Pt13;
    Y[15] = W15;
    Y[16] = Tt15;
    Y[17] = Pt15;
    Y[18] = W17;
    Y[19] = Tt17;
    Y[20] = Pt17;
    Y[21] = W22;
    Y[22] = Tt22;
    Y[23] = Pt22;
    Y[24] = W23;
    Y[25] = Tt23;
    Y[26] = Pt23;
    Y[27] = W24a;
    Y[28] = Tt24a;
    Y[29] = Pt24a;
    Y[30] = W24;
    Y[31] = Tt24;
    Y[32] = Pt24;
    Y[33] = W25;
    Y[34] = Tt25;
    Y[35] = Pt25;
    Y[36] = W36;
    Y[37] = Tt36;
    Y[38] = Pt36;
    Y[39] = Ps36;
    Y[40] = W4;
    Y[41] = Tt4;
    Y[42] = Pt4;
    Y[43] = W45;
    Y[44] = Tt45;
    Y[45] = Pt45;
    Y[46] = W48;
    Y[47] = Tt48;
    Y[48] = Pt48;
    Y[49] = W5;
    Y[50] = Tt5;
    Y[51] = Pt5;
    Y[52] = W7;
    Y[53] = Tt7;
    Y[54] = Pt7;
    Y[55] = Fdrag;
    Y[56] = Fg18 + Fg8;
    Y[57] = Fnet;
    Y[58] = Fg18;
    Y[59] = Fg8;
    Y[60] = TSFC;
    Y[61] = SMMap21;
    Y[62] = SMMap24a;
    Y[63] = SMMap36;

    // Diagnostic output
    E[0] = Nc24a;
    E[1] = Nc36;
    E[2] = NcMap21; /*--- Fan NcMap ---*/
    E[3] = NcMap24a; /*--- LPC NcMap ---*/
    E[4] = NcMap36; /*--- HPC NcMap ---*/
    E[5] = NcMap45; /*--- HPT NcMap ---*/
    E[6] = NcMap5; /*--- LPT NcMap ---*/
    E[7] = Trq21; /*--- Fan Torque ---*/
    E[8] = Trq24a; /*--- LPC Torque ---*/
    E[9] = Trq36; /*--- HPC Torque ---*/
    E[10] = Trq45; /*--- HPT Torque ---*/
    E[11] = Trq5; /*--- LPT Torque ---*/
    E[12] = Ps0;
}    