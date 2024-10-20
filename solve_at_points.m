% solve_at_points.m
% Written by Thomas Kennings
% NASA Glenn Research Center, Cleveland, OH
% May 30, 2024

% This script loads in engine model data, loads in the user's chosen 
% operating conditions (specified in inputs.xlsx), and iterates through 
% each condition. At each condition, a Newton-Raphson solver is invoked 
% to solve for the engine's steady-state. Outputs are written to
% outputs.mat.

clear; clc;

%% Definition of constants
DO_ELECTRIC_MOTORS = true; % if true, then the U-vector will include electric motor powers
ENABLE_DEBUG = true; % setting to true will enable error and warning messages in the terminal

STANDARD_DAY_TEMPERATURE_R = 518.67; % defined by International Standard Atmosphere
GEAR_RATIO = 3.1; % AGTF30 gear ratio between low-pressure shaft and fan

solver_independents_selection = [   true;  %--- WIn ---
    true;  %--- FAN_RLIn ---
    true;  %--- LPC_RLIn ---
    true;  %--- HPC_RLIn ---
    true;  %--- BPR ---
    true;  %--- HPT_PR ---
    true;  %--- LPT_PR ---
    true;  %--- WfIn ---
    false;  %--- VAFNIn ---
    false;  %--- VBVIn ---
    false;  %--- N2In ---
    true;  %--- N3In ---
    false;  %--- HPpwrIn ---
    false]; %--- LPpwrIn ---

solver_dependents_selection = [ true;  %--- W21err ---
    true;   %--- W24aerr ---
    true;   %--- W36err ---
    true;   %--- W45err ---
    true;   %--- W5err ---
    true;   %--- W8err ---
    true;   %--- W18err ---
    true;   %--- N2dot ---
    true;   %--- N3dot ---
    false;  %--- LPC SM error ---
    false;  %--- Fnet error ---
    false];  %--- T45_target ---

solver_targets = [  NaN;    %--- LPC_SM_target ---
                    NaN;    %--- Fnet_target ---
                    NaN];  %--- T45_target ---


%% Setup simulations
addpath('engine_model');

load("AGTF30_simulink_data.mat");
construct_gridded_interpolants;

[inputs_array, num_inputs] = load_inputs_from_csv();

outputs = repmat(struct('altitude', NaN, ...
    'mach_number', NaN, ...
    'N1c', NaN, ...
    'dTamb', NaN, ...
    'health_params', NaN, ...
    'solver_independents_solution', NaN, ...
    'X', NaN, ...
    'Y', NaN, ...
    'U', NaN, ...
    'E', NaN, ...
    'A', NaN, ...
    'B', NaN, ...
    'C', NaN, ...
    'D', NaN, ...
    'linearization_failure_mode', NaN), ...
    num_inputs, 1);


%% Iterate through all specified operating conditions
for input_num = 1:num_inputs
    altitude = inputs_array(input_num).altitude;
    mach_number = inputs_array(input_num).mach_number;
    N1c = inputs_array(input_num).N1c;
    dTamb = inputs_array(input_num).dTamb;
    health_params = inputs_array(input_num).health_params;

    if ~in_envelope(altitude, mach_number, dTamb)
        disp(['Altitude ' num2str(altitude) ', Mach Number ' num2str(mach_number) ...
            ', dTamb ' num2str(dTamb) ' is beyond engine flight envelope. ' ...
            'Convergence is highly unlikely.']);
    end


    %% Get environmental conditions at the operating condition
    environmental_conditions = [altitude, mach_number, dTamb]';
    ambient_conditions = Ambient_C(environmental_conditions);
    T2 = ambient_conditions(1);


    %% Get initial guess for solver
    solver_initial_guess = get_initial_guess(altitude, mach_number, N1c, dTamb, IC_interpolants);

    % Overwrite VAFN and VBV inputs if we want them to be open-loop scheduled
    solver_initial_guess(9) = VAFN_interpolant(mach_number, N1c);
    solver_initial_guess(10) = VBV_interpolant(mach_number, N1c);
    
    % Overwrite N2 input since we are targeting a specific N1c
    solver_initial_guess(11) = N1c * ...
        sqrt(T2/STANDARD_DAY_TEMPERATURE_R) * GEAR_RATIO;

    % These lines can be used to manually set electric motor power injections.
    % Negative values correspond to power extraction.
    % High pressure shaft power injection (default is -350)
    solver_initial_guess(13) = -350;
    % Low pressure shaft power injection (default is 0)
    solver_initial_guess(14) = 0;

    
    %% HPC bleeds
    % Customer bleeds (lbm/sec)
    bleeds(1) = 0;

    % Cooling bleeds (as a fraction of total HPC flow). 
    % Location of each bleed flow's reintroduction is noted.
    bleeds(2:4) = [ 0.02, ...   % LPT exit
                    0.0693, ... % HPT exit
                    0.0625];    % HPT forward


    %% Run the solver
    [solver_dependents_solution, solver_independents_solution, X, U, Y, E, convergence_reached, ...
        solver_iterations] = nr_solver(environmental_conditions, solver_initial_guess, ...
        solver_targets, health_params, bleeds, solver_independents_selection, solver_dependents_selection, ENABLE_DEBUG);
    
    if (Y(55) < E(13))
        % Core nozzle backflow
        convergence_reached = 0;
    end

    if convergence_reached
        [A, B, C, D, linearization_failure_mode] = do_linearization(solver_independents_solution, ...
            X, Y, altitude, mach_number, N1c, VAFN_interpolant, VBV_interpolant, ...
            environmental_conditions, health_params, bleeds, DO_ELECTRIC_MOTORS, ENABLE_DEBUG);
    else
        A = [];
        B = [];
        C = [];
        D = [];
        linearization_failure_mode = "Not attempted";
    end


    %% Load data into output struct/array
    outputs(input_num).altitude = altitude;
    outputs(input_num).mach_number = mach_number;
    outputs(input_num).N1c = N1c;
    outputs(input_num).dTamb = dTamb;
    outputs(input_num).health_params = health_params;
    outputs(input_num).converged = convergence_reached;
    outputs(input_num).solver_independents_solution = solver_independents_solution;
    outputs(input_num).X = X;
    outputs(input_num).Y = Y;

    if DO_ELECTRIC_MOTORS
        outputs(input_num).U = U;
    else
        outputs(input_num).U = U(1);
    end

    outputs(input_num).E = E;
    outputs(input_num).A = A;
    outputs(input_num).B = B;
    outputs(input_num).C = C;
    outputs(input_num).D = D;
    outputs(input_num).linearization_failure_mode = linearization_failure_mode;
    
    
    %% Display to terminal
    if convergence_reached
        disp(['Alt = ' num2str(altitude) ' MN = ' num2str(mach_number) ' N1c = ' num2str(N1c) ...
            ' Solved successfully. Solver iterations: ' num2str(solver_iterations)]);
    else
        disp(['Alt = ' num2str(altitude) ' MN = ' num2str(mach_number) ' N1c = ' num2str(N1c) ...
            ' DID NOT CONVERGE! Solver iterations: ' num2str(solver_iterations)]);
    end

end


%% Write outputs to file
save('outputs.mat', 'outputs');
disp('Finished, saved outputs to file');
