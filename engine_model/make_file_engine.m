% This is a make file for the AGTF30 MEX engine model. This function needs
% to be run to recompile the MEX function whenever any of the files listed 
% below are changed.

mex MEX_engine_model.c Ambient_TMATS_body.c Inlet_TMATS_body.c Compressor_TMATS_body.c ...
Duct_TMATS_body.c Valve_TMATS_body.c Nozzle_TMATS_body.c Burner_TMATS_body.c ...
Turbine_TMATS_body.c t2hc_TMATS.c pt2sc_TMATS.c interp1Ac_TMATS.c interp2Ac_TMATS.c ...
interp3Ac_TMATS.c sp2tc_TMATS.c h2tc_TMATS.c functions_TMATS.c PcalcStat_TMATS.c SFCCalc_TMATS.c ...
Splitter_TMATS.c StaticCalc_TMATS_body.c Shaft_TMATS_body.c
