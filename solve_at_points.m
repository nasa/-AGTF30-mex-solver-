% solve_at_points.m
% Written by Thomas Kennings
% NASA Glenn Research Center, Cleveland, OH
% May 30, 2024

% This script loads in engine model data, loads in the user's chosen 
% operating conditions (specified in inputs.csv), and iterates through 
% each condition. At each condition, a Newton-Raphson solver is invoked 
% to solve for the engine's steady-state. Outputs are written to
% outputs.mat.

clear; clc;

%% Definition of constants
DO_ELECTRIC_MOTORS = true; % if true, then the U-vector will include electric motor powers
DO_ML_CHALLENGE_PROBLEM = true; % enables some scripts for machine-learning challenge problems
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
if DO_ML_CHALLENGE_PROBLEM
    addpath('ML_challenge_problem');
end

load("AGTF30_simulink_data.mat");
construct_gridded_interpolants;

[inputs_array, num_inputs] = load_inputs_from_csv();

outputs = repmat(struct('altitude', NaN, ...
    'mach_number', NaN, ...
    'N1c', NaN, ...
    'dTamb', NaN, ...
    'health_params', NaN, ...
    'biases', NaN, ...
    'solver_independents_solution', NaN, ...
    'X', NaN, ...
    'Y', NaN, ...
    'U', NaN, ...
    'E', NaN, ...
    'A', NaN, ...
    'B', NaN, ...
    'C', NaN, ...
    'D', NaN, ...
    'converged', NaN, ...
    'linearization_failure_mode', NaN), ...
    num_inputs, 1);


%% Iterate through all specified operating conditions
for input_num = 1:num_inputs
    altitude_actual = inputs_array(input_num).altitude;
    mach_number_actual = inputs_array(input_num).mach_number;
    N1c_actual = inputs_array(input_num).N1c;
    dTamb_actual = inputs_array(input_num).dTamb;
    health_params = inputs_array(input_num).health_params;
    biases = inputs_array(input_num).biases;

    if ~in_envelope(altitude_actual, mach_number_actual, dTamb_actual) 
        if ENABLE_DEBUG
        disp(['Altitude ' num2str(altitude_actual) ', Mach Number ' num2str(mach_number_actual) ...
            ', dTamb ' num2str(dTamb_actual) ' is beyond engine flight envelope. ' ...
            'Convergence is highly unlikely.']);
        end
    end


    %% Calculate ambient conditions
    environmental_conditions = [altitude_actual, mach_number_actual, dTamb_actual]';
    ambient_conditions = Ambient_C(environmental_conditions);


    %% Apply pre-model sensor biases
    % Only biases which affect engine steady-state (e.g. Tt2 is used to 
    % calculate sensed N1c, which affects scheduling of VBV and VAFN). 
    % Other biases will be applied after the model is called.

    % Tt2 sensor bias
    Tt2_actual = ambient_conditions(1); % Tt2 = Tt0 in the AGTF30 model
    Tt2_sensed = Tt2_actual + biases(3);

    % Ptamb sensor bias
    Pamb_actual = ambient_conditions(4); % Ambient pressure = Ps0
    Pamb_sensed = Pamb_actual + biases(1);

    Pt0_actual = ambient_conditions(2);

    % Using the exact equations and tables from the AGTF30 model to get 
    % the pressure drop across the inlet.
    inlet_struct.Rambase = 1.0;
    inlet_struct.RamVec = [0.9, 1, 1.007, 1.028, 1.065, 1.117, 1.276, 1.525, 1.692];
    inlet_struct.Ramtbl = [0.995, 0.995, 0.996, 0.997, 0.997, 0.998, 0.998, 0.998, 0.998];
    inlet_struct.Ram_sf = interp1(inlet_struct.RamVec, inlet_struct.Ramtbl, Pt0_actual/Pamb_actual);

    Pt2_actual = Pt0_actual * inlet_struct.Rambase * inlet_struct.Ram_sf;
    Pt2_sensed = Pt2_actual + biases(2);

    Pt0_sensed = Pt2_sensed / (inlet_struct.Rambase * inlet_struct.Ram_sf);


    %% Calculate 'sensed' Altitude, Mach, and N1c.
    % Sensor biases may change *sensed* flight conditions, leading to off-schedule actuator positioning.
    % The engine model will still be run at the actual flight conditions specified by the user.
    % If no biases are specified in inputs.csv, sensed = actual.

    % Sensed altitude, looked up using Pamb_sensed and data from the TMATS ambient block
    ambient_struct.AltVec = [-5000, 0, 5000, 10000, 15000, 20000, 25000, 30000, 35000, 40000, 45000, 50000, 60000, 70000, 80000];
    ambient_struct.PsVec = [17.554, 14.696, 12.228, 10.108, 8.297, 6.759, 5.461, 4.373, 3.468, 2.73, 2.149, 1.692, 1.049, 0.651, 0.406];
    altitude_sensed = interp1(ambient_struct.PsVec, ambient_struct.AltVec, Pamb_sensed);

    % Sensed Mach value
    GAMMA = 1.4; % typical value used by NASA T-MATS and many others
    mach_number_sensed = sqrt((2/(GAMMA-1)) * ((Pt0_sensed/Pamb_sensed)^((GAMMA-1)/GAMMA)-1));
    if imag(mach_number_sensed) ~= 0
        if ENABLE_DEBUG
        disp(['Warning: Sensed Mach number calculation resulted in imaginary number ' ...
            'due to Pt0_sensed > Pt2_sensed. Setting sensed Mach to 0.']);
        end
        mach_number_sensed = 0;
    end

    % Sensed N1c value
    N1c_sensed = (N1c_actual * sqrt(Tt2_actual/STANDARD_DAY_TEMPERATURE_R) + biases(4)) ...
    / sqrt(Tt2_sensed/STANDARD_DAY_TEMPERATURE_R);


    %% Get initial guess for solver
    solver_initial_guess = get_initial_guess(altitude_actual, mach_number_actual, N1c_actual, dTamb_actual, IC_interpolants);

    % Overwrite VAFN and VBV inputs if we want them to be open-loop scheduled
    solver_initial_guess(9) = VAFN_interpolant(mach_number_sensed, N1c_sensed) + biases(6);
    solver_initial_guess(9) = min(8000, max(0, solver_initial_guess(9)));
    solver_initial_guess(10) = VBV_interpolant(mach_number_sensed, N1c_sensed) + biases(5);
    solver_initial_guess(10) = min(1, max(0, solver_initial_guess(10)));

    % Overwrite N2 input since we are targeting a specific N1c
    solver_initial_guess(11) = N1c_actual * sqrt(Tt2_actual/STANDARD_DAY_TEMPERATURE_R) * GEAR_RATIO;

    % These lines can be used to manually set electric motor power injections.
    % Negative values correspond to power extraction.
    % High pressure shaft power injection (default is -350)
    solver_initial_guess(13) = -350 - biases(7);
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
            X, Y, altitude_actual, mach_number_actual, N1c_actual, VAFN_interpolant, VBV_interpolant, ...
            environmental_conditions, health_params, bleeds, DO_ELECTRIC_MOTORS, ENABLE_DEBUG);
    else
        A = [];
        B = [];
        C = [];
        D = [];
        linearization_failure_mode = "Not attempted";
    end


    %% Apply post-model sensor biases
    % These are sensor biases which do NOT affect the engine's steady-state condition.
    U(1) = U(1) + biases(9);                % Wf

    Y(2) = GEAR_RATIO * (Y(1) + biases(4)); % N2mech
    Y(3) = Y(3) + biases(8);                % N3mech
    Y(35) = Y(35) + biases(10);             % Tt25
    Y(36) = Y(36) + biases(11);             % Pt25
    Y(38) = Y(38) + biases(12);             % Tt3
    Y(40) = Y(40) + biases(13);             % Ps3
    Y(45) = Y(45) + biases(14);             % Tt45
    Y(51) = Y(51) + biases(15);             % Tt5


    %% Load data into output struct/array
    outputs(input_num).altitude = altitude_actual;
    outputs(input_num).mach_number = mach_number_actual;
    outputs(input_num).N1c = N1c_sensed;
    outputs(input_num).dTamb = dTamb_actual;
    outputs(input_num).health_params = health_params;
    outputs(input_num).biases = biases;
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
        disp(['Alt = ' num2str(altitude_actual) ' MN = ' num2str(mach_number_actual) ' N1c = ' num2str(N1c_actual) ...
            ' Solved successfully. Solver iterations: ' num2str(solver_iterations)]);
    else
        disp(['Alt = ' num2str(altitude_actual) ' MN = ' num2str(mach_number_actual) ' N1c = ' num2str(N1c_actual) ...
            ' DID NOT CONVERGE! Solver iterations: ' num2str(solver_iterations)]);
    end


    %% Do outputs formatting for machine-learning challenge problem
    if DO_ML_CHALLENGE_PROBLEM
        make_training_set_ML_challenge;
        make_test_set_ML_challenge;
    end

end


%% Write outputs to file
save('outputs.mat', 'outputs');
if DO_ML_CHALLENGE_PROBLEM
    save('ML_challenge_problem/training_set_challenge_problem.mat', 'training_set_challenge_problem');
    save('ML_challenge_problem/test_set_challenge_problem.mat', 'test_set_challenge_problem');
end
disp('Finished, saved outputs to file');
