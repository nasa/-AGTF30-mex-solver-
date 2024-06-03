% do_linearization.m
% Written by Thomas Kennings
% NASA Glenn Research Center, Cleveland, OH
% May 30, 2024

% This function perturbs each state and input to generate linear 
% state-space matrices (A, B, C, D). Partial derivative are computed 
% by taking the average derivatives resulting from perturbations in the 
% positive and negative direction.

function [A, B, C, D, failure_mode] = do_linearization(solver_independents_solution_trim, ...
    X_trim, Y_trim, altitude, mach_number, N1c, VAFN_interpolant, VBV_interpolant, ...
    environmental_conditions, health_params, DO_ELECTRIC_MOTORS)

PERTURBATION_FRACTION = 0.0003;  % 0.0003 = 0.03%
PWR_TRQ_FORMULA_CONSTANT = 5252.113; % Conversion factor to maintain units of lb-ft on torque perturbations

failure_mode = "None"; % As long as no failures happen, this value should persist

solver_independents_selection = [   true;  % 1) WIn
    true;  % 2) FAN_RLIn
    true;  % 3) LPC_RLIn
    true;  % 4) HPC_RLIn
    true;  % 5) BPR
    true;  % 6) HPT_PR
    true;  % 7) LPT_PR
    false;  % 8) WfIn
    false;  % 9) VAFNIn
    false;  % 10) VBVIn
    false;  % 11) N2In
    false;  % 12) N3In
    false;  % 13) HPpwrIn
    false]; % 14) LPpwrIn

solver_dependents_selection = [ true;  % 1) W21err
    true;  % 2) W24aerr
    true;  % 3) W36err
    true;  % 4) W45err
    true;  % 5) W5err
    true;  % 6) W8err
    true;  % 7) W18err
    false;  % 8) N2dot
    false;  % 9) N3dot
    false;  % 10) LPC SM error
    false;  % 11) LPT Pwr Ratio error
    false;  % 12) Fnet error
    false;  % 13) T4 error
    false]; % 14) HPCNcMap error

solver_targets = [   NaN;     % LPC_SM_target
    NaN;     % LPT_HPXratio_target
    NaN];    % Fnet_target

%% State 1: Low-pressure shaft speed (N2)
N1c_perturbation = N1c * PERTURBATION_FRACTION;
N2_perturbation = X_trim(1) * PERTURBATION_FRACTION;

% Positive perturbation
solver_initial_guess = solver_independents_solution_trim;
solver_initial_guess(9) = VAFN_interpolant(mach_number, N1c + N1c_perturbation);
solver_initial_guess(10) = VBV_interpolant(mach_number, N1c + N1c_perturbation);
solver_initial_guess(11) = X_trim(1) + N2_perturbation;

[solver_dependents_solution, solver_independents_solution, X, U, Y, E, convergence_reached, ...
        solver_iterations] = nr_solver(environmental_conditions, ...
        solver_initial_guess, solver_targets, health_params, ...
        solver_independents_selection, solver_dependents_selection);

if (convergence_reached ~=1)
    disp(['(' num2str(altitude), ', ' num2str(mach_number) ', ' num2str(N1c) ')']);
    disp('X1 positive perturbation did not converge');
    failure_mode = "N2+";
    return;
end

if (Y(55) < E(13))
    disp(['(' num2str(altitude), ', ' num2str(mach_number) ', ' num2str(N1c) ')']);
    disp('X1 positive perturbation resulted in core nozzle backflow');
    failure_mode = "N2+";
    return;
end

% Store A and C positive matrix column 1
A_col1p = [solver_dependents_solution(8) solver_dependents_solution(9)]' ./ N2_perturbation;
C_col1p = (Y - Y_trim) ./ N2_perturbation;

% Negative perturbation
solver_initial_guess = solver_independents_solution_trim;
solver_initial_guess(9) = VAFN_interpolant(mach_number, N1c - N1c_perturbation);
solver_initial_guess(10) = VBV_interpolant(mach_number, N1c - N1c_perturbation);
solver_initial_guess(11) = X_trim(1) - N2_perturbation;

[solver_dependents_solution, solver_independents_solution, X, U, Y, E, convergence_reached, ...
        solver_iterations] = nr_solver(environmental_conditions, ...
        solver_initial_guess, solver_targets, health_params, ...
        solver_independents_selection, solver_dependents_selection);

if (convergence_reached ~=1)
    disp(['(' num2str(altitude), ', ' num2str(mach_number) ', ' num2str(N1c) ')']);
    disp('X1 negative perturbation did not converge');
    failure_mode = "N2-";
    return;
end

if (Y(55) < E(13))
    disp(['(' num2str(altitude), ', ' num2str(mach_number) ', ' num2str(N1c) ')']);
    disp('X1 negative perturbation resulted in core nozzle backflow');
    failure_mode = "N2-";
    return;
end

% Store A and C negative matrix column 1
A_col1n = -[solver_dependents_solution(8) solver_dependents_solution(9)]' ./ N2_perturbation;
C_col1n = -(Y - Y_trim) ./ N2_perturbation;

% Average positive and negative perturbations results
A_col1 = (A_col1p + A_col1n)/2;
C_col1 = (C_col1p + C_col1n)/2;

%% State 2: High-pressure shaft speed (N3)
N3_perturbation = X_trim(2) * PERTURBATION_FRACTION;

% Positive perturbation
solver_initial_guess = solver_independents_solution_trim;
solver_initial_guess(12) = X_trim(2) + N3_perturbation;

[solver_dependents_solution, solver_independents_solution, X, U, Y, E, convergence_reached, ...
        solver_iterations] = nr_solver(environmental_conditions, ...
        solver_initial_guess, solver_targets, health_params, ...
        solver_independents_selection, solver_dependents_selection);

if (convergence_reached ~= 1)
    disp(['(' num2str(altitude), ', ' num2str(mach_number) ', ' num2str(N1c) ')']);
    disp('X2 positive perturbation did not converge');
    failure_mode = "N3+";
    return;
end

if (Y(55) < E(13))
    disp(['(' num2str(altitude), ', ' num2str(mach_number) ', ' num2str(N1c) ')']);
    disp('X2 positive perturbation resulted in core nozzle backflow');
    failure_mode = "N3+";
    return;
end

% Store A and C positive matrix column 2
A_col2p = [solver_dependents_solution(8) solver_dependents_solution(9)]' ./ N3_perturbation;
C_col2p = (Y - Y_trim) ./ N3_perturbation;

% Negative perturbation
solver_initial_guess = solver_independents_solution_trim;
solver_initial_guess(12) = X_trim(2) - N3_perturbation;

[solver_dependents_solution, solver_independents_solution, X, U, Y, E, convergence_reached, ...
        solver_iterations] = nr_solver(environmental_conditions, ...
        solver_initial_guess, solver_targets, health_params, ...
        solver_independents_selection, solver_dependents_selection);

if (convergence_reached ~= 1)
    disp(['(' num2str(altitude), ', ' num2str(mach_number) ', ' num2str(N1c) ')']);
    disp('X2 negative perturbation did not converge');
    failure_mode = "N3-";
    return;
end

if (Y(55) < E(13))
    disp(['(' num2str(altitude), ', ' num2str(mach_number) ', ' num2str(N1c) ')']);
    disp('X2 negative perturbation resulted in core nozzle backflow');
    failure_mode = "N3-";
    return;
end

% Store A and C negative matrix column 2
A_col2n = -[solver_dependents_solution(8) solver_dependents_solution(9)]' ./ N3_perturbation;
C_col2n = -(Y - Y_trim) ./ N3_perturbation;

% Average positive and negative perturbations results
A_col2 = (A_col2p + A_col2n)/2;
C_col2 = (C_col2p + C_col2n)/2;

%% Input 1: Fuel flow (Wf)
fuel_flow_perturbation = solver_independents_solution_trim(8) * PERTURBATION_FRACTION;

% Positive perturbation
solver_initial_guess = solver_independents_solution_trim;
solver_initial_guess(8) = solver_independents_solution_trim(8) + fuel_flow_perturbation;

[solver_dependents_solution, solver_independents_solution, X, U, Y, E, convergence_reached, ...
        solver_iterations] = nr_solver(environmental_conditions, ...
        solver_initial_guess, solver_targets, health_params, ...
        solver_independents_selection, solver_dependents_selection);

if (convergence_reached ~= 1)
    disp(['(' num2str(altitude), ', ' num2str(mach_number) ', ' num2str(N1c) ')']);
    disp('U1 positive perturbation did not converge');
    failure_mode = "Wf+";
    return;
end

if (Y(55) < E(13))
    disp(['(' num2str(altitude), ', ' num2str(mach_number) ', ' num2str(N1c) ')']);
    disp('U1 positive perturbation resulted in core nozzle backflow');
    failure_mode = "Wf+";
    return;
end

% Store B and D positive matrix column 1
B_col1p = [solver_dependents_solution(8) solver_dependents_solution(9)]' ./ fuel_flow_perturbation;
D_col1p = (Y - Y_trim) ./ fuel_flow_perturbation;

% Negative perturbation
solver_initial_guess = solver_independents_solution_trim;
solver_initial_guess(8) = solver_independents_solution_trim(8) - fuel_flow_perturbation;

[solver_dependents_solution, solver_independents_solution, X, U, Y, E, convergence_reached, ...
        solver_iterations] = nr_solver(environmental_conditions, ...
        solver_initial_guess, solver_targets, health_params, ...
        solver_independents_selection, solver_dependents_selection);

if (convergence_reached ~= 1)
    disp(['(' num2str(altitude), ', ' num2str(mach_number) ', ' num2str(N1c) ')']);
    disp('U1 negative perturbation did not converge');
    failure_mode = "Wf-";
    return;
end

if (Y(55) < E(13))
    disp(['(' num2str(altitude), ', ' num2str(mach_number) ', ' num2str(N1c) ')']);
    disp('U1 negative perturbation resulted in core nozzle backflow');
    failure_mode = "Wf-";
    return;
end

% Store B and D negative matrix column 1
B_col1n = -[solver_dependents_solution(8) solver_dependents_solution(9)]' ./ fuel_flow_perturbation;
D_col1n = -(Y - Y_trim) ./ fuel_flow_perturbation;

% Average positive and negative perturbations
B_col1 = (B_col1p + B_col1n)/2;
D_col1 = (D_col1p + D_col1n)/2;

%% Input 2: High-pressure shaft power extraction (from electric motor)
HP_pwr_perturbation = solver_independents_solution_trim(13) * PERTURBATION_FRACTION;
N3_mech = Y_trim(3);
HP_trq_perturbation = PWR_TRQ_FORMULA_CONSTANT*HP_pwr_perturbation/N3_mech;

% Positive perturbation
solver_initial_guess = solver_independents_solution_trim;
solver_initial_guess(13) = solver_independents_solution_trim(13) + HP_pwr_perturbation;

[solver_dependents_solution, solver_independents_solution, X, U, Y, E, convergence_reached, ...
        solver_iterations] = nr_solver(environmental_conditions, ...
        solver_initial_guess, solver_targets, health_params, ...
        solver_independents_selection, solver_dependents_selection);

if (convergence_reached ~= 1)
    disp(['(' num2str(altitude), ', ' num2str(mach_number) ', ' num2str(N1c) ')']);
    disp('U2 positive perturbation did not converge');
    failure_mode = "HPpwr+";
    return;
end

if (Y(55) < E(13))
    disp(['(' num2str(altitude), ', ' num2str(mach_number) ', ' num2str(N1c) ')']);
    disp('U2 positive perturbation resulted in core nozzle backflow');
    failure_mode = "HPpwr+";
    return;
end

% Store B and D positive matrix column 2
% Dividing by delta-torque so that units of B- and D-matrices are with respect to torque.
B_col2p = [solver_dependents_solution(8) solver_dependents_solution(9)]' ./ HP_trq_perturbation;
D_col2p = (Y - Y_trim) ./ HP_trq_perturbation;

% Negative perturbation
solver_initial_guess = solver_independents_solution_trim;
solver_initial_guess(13) = solver_independents_solution_trim(13) - HP_pwr_perturbation;

[solver_dependents_solution, solver_independents_solution, X, U, Y, E, convergence_reached, ...
        solver_iterations] = nr_solver(environmental_conditions, ...
        solver_initial_guess, solver_targets, health_params, ...
        solver_independents_selection, solver_dependents_selection);

if (convergence_reached ~= 1)
    disp(['(' num2str(altitude), ', ' num2str(mach_number) ', ' num2str(N1c) ')']);
    disp('U2 negative perturbation did not converge');
    failure_mode = "HPpwr-";
    return;
end

if (Y(55) < E(13))
    disp(['(' num2str(altitude), ', ' num2str(mach_number) ', ' num2str(N1c) ')']);
    disp('U2 negative perturbation resulted in core nozzle backflow');
    failure_mode = "HPpwr-";
    return;
end

% Store B and D negative matrix column 2
% Dividing by delta-torque so that units of B- and D-matrices are with respect to torque.
B_col2n = -[solver_dependents_solution(8) solver_dependents_solution(9)]' ./ HP_trq_perturbation;
D_col2n = -(Y - Y_trim) ./ HP_trq_perturbation;

% Average positive and negative perturbations
B_col2 = (B_col2p + B_col2n)/2;
D_col2 = (D_col2p + D_col2n)/2;

%% Input 3: Low-pressure shaft power extraction (from electric motor)
% Perturbing LP from 0 must use addition of constant rather than multiplication.
% Made to be the same amount as HP pwr perturbation.
LP_pwr_perturbation = HP_pwr_perturbation;
N2_mech = Y_trim(2);
LP_trq_perturbation = PWR_TRQ_FORMULA_CONSTANT*LP_pwr_perturbation/N2_mech;

% Positive perturbation
solver_initial_guess = solver_independents_solution_trim;
solver_initial_guess(14) = solver_initial_guess(14) + LP_pwr_perturbation;

[solver_dependents_solution, solver_independents_solution, X, U, Y, E, convergence_reached, ...
        solver_iterations] = nr_solver(environmental_conditions, ...
        solver_initial_guess, solver_targets, health_params, ...
        solver_independents_selection, solver_dependents_selection);

if (convergence_reached ~= 1)
    disp(['(' num2str(altitude), ', ' num2str(mach_number) ', ' num2str(N1c) ')']);
    disp('U3 positive perturbation did not converge');
    failure_mode = "LPpwr+";
    return;
end

if (Y(55) < E(13))
    disp(['(' num2str(altitude), ', ' num2str(mach_number) ', ' num2str(N1c) ')']);
    disp('U3 positive perturbation resulted in core nozzle backflow');
    failure_mode = "LPpwr+";
    return;
end

% Store B and D positive matrix column 3
% Dividing by delta-torque so that units of B- and D-matrices are with respect to torque.
B_col3p = [solver_dependents_solution(8) solver_dependents_solution(9)]' ./ LP_trq_perturbation;
D_col3p = (Y - Y_trim) ./ LP_trq_perturbation;

% Negative perturbation
solver_initial_guess = solver_independents_solution_trim;
solver_initial_guess(14) = solver_initial_guess(14) - LP_pwr_perturbation;

[solver_dependents_solution, solver_independents_solution, X, U, Y, E, convergence_reached, ...
        solver_iterations] = nr_solver(environmental_conditions, ...
        solver_initial_guess, solver_targets, health_params, ...
        solver_independents_selection, solver_dependents_selection);

if (convergence_reached ~= 1)
    disp(['(' num2str(altitude), ', ' num2str(mach_number) ', ' num2str(N1c) ')']);
    disp('U3 negative perturbation did not converge');
    failure_mode = "LPpwr-";
    return;
end

if (Y(55) < E(13))
    disp(['(' num2str(altitude), ', ' num2str(mach_number) ', ' num2str(N1c) ')']);
    disp('U3 negative perturbation resulted in core nozzle backflow');
    failure_mode = "LPpwr-";
    return;
end

% Store B and D negative matrix column 3
% Dividing by delta-torque so that units of B- and D-matrices are with respect to torque.
B_col3n = -[solver_dependents_solution(8) solver_dependents_solution(9)]' ./ LP_trq_perturbation;
D_col3n = -(Y - Y_trim) ./ LP_trq_perturbation;

% Average positive and negative perturbations
B_col3 = (B_col3p + B_col3n)/2;
D_col3 = (D_col3p + D_col3n)/2;

%% State-space matrices
A = [A_col1 A_col2];
C = [C_col1 C_col2];

if DO_ELECTRIC_MOTORS
    B = [B_col1 B_col2 B_col3];
    D = [D_col1 D_col2 D_col3];
else
    B = B_col1;
    D = D_col1;
end
