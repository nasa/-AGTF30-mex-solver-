% get_initial_guess.m
% Written by Thomas Kennings
% NASA Glenn Research Center, Cleveland, OH
% May 30, 2024

% Provides initial guess for Newton-Raphson solver by interpolating
% data from the original AGTF30 data. Utilizes the interpolant objects
% created in construct_gridded_interpolants.m to accelerate execution.

function solver_initial_guess = get_initial_guess(altitude, mach_number, N1c, dTamb, IC_interpolants)

    dT_adj = 1 + dTamb/1000;

    air_flow = IC_interpolants.Wc(altitude, N1c, mach_number) / dT_adj; % Flow (pps)
    HPT_pressure_ratio = IC_interpolants.HPT_PR(altitude, N1c, mach_number) / (1 + (dT_adj-1) * mach_number);   % HPT Pressure Ratio
    HPC_Rline = IC_interpolants.HPC_Rline(altitude, N1c, mach_number) / (1 + (dT_adj-1) * mach_number); % HPC Rline
    FAN_Rline = IC_interpolants.FAN_Rline(altitude, N1c, mach_number);  % Fan_Rline
    bypass_ratio = IC_interpolants.BPR(altitude, N1c, mach_number); % Branch Pressure Ratio
    LPC_Rline = IC_interpolants.LPC_Rline(altitude, N1c, mach_number);  % LPC_Rline
    LPT_pressure_ratio = IC_interpolants.LPT_PR(altitude, N1c, mach_number);    % LPT Pressure Ratio
    N2 = IC_interpolants.N2(altitude, N1c, mach_number) * dT_adj;   % Low Pressure Shaft Speed (rpm)
    N3 = IC_interpolants.N3(altitude, N1c, mach_number) * dT_adj;   % High Pressure Shaft Speed (rpm)
    fuel_flow = IC_interpolants.Wf(altitude, N1c, mach_number) * dT_adj;    % Fuel Flow (pps)
    VAFN = IC_interpolants.VAFN(altitude, N1c, mach_number) * dT_adj;   % Variable area fan nozzle
    VBV = IC_interpolants.VBV(altitude, N1c, mach_number);  % Variable bleed valve
    HP_shaft_power_injection = -350;    % High-pressure shaft power extraction
    LP_shaft_power_injection = 0;   % Low-pressure shaft power extraction

    solver_initial_guess = [air_flow, FAN_Rline, LPC_Rline, HPC_Rline, bypass_ratio, HPT_pressure_ratio, LPT_pressure_ratio, fuel_flow, VAFN, VBV, N2, N3, HP_shaft_power_injection, LP_shaft_power_injection]';
    return;

end
