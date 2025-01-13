% This file reformats information for use in machine-learning challenge
% problems. Only outputs are included since this script makes a 'test' 
% dataset.

if ~exist('test_set_challenge_problem', 'var')
    % Initialize machine-learning challenge problem outputs structure
    test_set_challenge_problem = repmat(struct('altitude_sensed', NaN, ...
    'mach_number_sensed', NaN, ...
    'U_sensed', NaN, ...
    'Y_sensed', NaN), ...
    num_inputs, 1);
end


% Sensed Outputs
test_set_challenge_problem(input_num).altitude_sensed = altitude_sensed;
test_set_challenge_problem(input_num).mach_number_sensed = mach_number_sensed;

test_set_challenge_problem(input_num).U_sensed = [ 
    U(1);                       % Wf
    solver_initial_guess(9);    % VAFN
    solver_initial_guess(10)];  % VBV

test_set_challenge_problem(input_num).Y_sensed = [ 
    Y(2);           % LP shaft sensed speed
    Y(3);           % HP shaft sensed speed
    Pamb_sensed;    % Pamb
    Tt2_sensed;     % Tt2
    Pt2_sensed;     % Pt2
    Y(35);          % Tt25
    Y(36);          % Pt25
    Y(38);          % Tt3
    Y(40);          % Ps3
    Y(45);          % Tt45
    Y(51)];         % Tt5

