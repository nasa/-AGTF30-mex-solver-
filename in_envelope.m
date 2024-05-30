% in_envelope.m
% Written by Thomas Kennings and Jeffryes Chapman
% NASA Glenn Research Center, Cleveland, OH
% May 30, 2024

% This function determines if the AGTF30 engine is within its flight 
% envelope. If beyond the flight envelope, data for initial guesses won't 
% be defined. This function will produce a warning if an operating 
% condition is beyond the flight envelope.

function within_envelope = in_envelope(altitude, mach_number, dTamb)
within_envelope = 1;

mach_number_max = 0.8;
mach_number_min = 0;

mach_number_hi  = [mach_number_min, 0.2, 0.5, 0.6, 0.7, mach_number_max];
altitude_hi = [1.0, 1.0, 2.5, 3.5, 4.0, 4.0]*10^4;

mach_number_low  = [mach_number_min, 0.5, 0.6, 0.7, mach_number_max];
altitude_low = [0.0, 0, 1.0, 2.0, 25]*10^3;

dTamb_hi = 30;
dTamb_low = -30;

% Determine if dTamb is in bounds
if dTamb_hi < dTamb || dTamb_low > dTamb
    within_envelope = 0;
end

% Determine if mach_number is in bounds
if mach_number_max < mach_number || mach_number_min > mach_number
    within_envelope = 0;
end

% Determine if altitude is in bounds
altitude_hi_calc = interp1(mach_number_hi,altitude_hi,mach_number);
altitude_low_calc = interp1(mach_number_low,altitude_low,mach_number);
if altitude > altitude_hi_calc || altitude < altitude_low_calc
    within_envelope = 0;
end

if within_envelope==0
    disp(['Out of flight envelope. Alt = ' num2str(altitude) ', MN = ' num2str(mach_number) ', dT = ' num2str(dTamb)]);
end

return;

end
