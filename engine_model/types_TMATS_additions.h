struct InletStruct{
    double eRambase_M;
    
    /* Vector & array data */
    double *X_eRamVec_M;
    double *T_eRamtbl_M;
    char *BlkNm;
    int *IWork;
    
    /* Dimensions of parameter arrays */
    int A;
    
};
typedef struct InletStruct InletStruct;

struct DuctStruct{
    double dP_M;
    double MNdes;
    double Ath;
    char *BlkNm;
};
typedef struct DuctStruct DuctStruct;

struct ShaftStruct{
    double Inertia_M;
};
typedef struct ShaftStruct ShaftStruct;

struct PwrSavingStruct{
    /* Vector & array data */
    double *X_MNVec_M;
    double *T_PwrSaveCoeffVec_M;
    char *BlkNm;
    int *IWork;
    
    /* Dimensions of parameter arrays */
    int A;
};
typedef struct PwrSavingStruct PwrSavingStruct;
