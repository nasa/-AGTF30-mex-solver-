% This is a make file for the standalone Ambient MEX function. The 
% standalone ambient MEX function is called to determine inlet conditions, 
% which inform initial guess estimates before calling the main engine model.

mex Ambient_C.c Ambient_TMATS_body.c  ...
    t2hc_TMATS.c pt2sc_TMATS.c interp1Ac_TMATS.c interp2Ac_TMATS.c sp2tc_TMATS.c functions_TMATS.c 
