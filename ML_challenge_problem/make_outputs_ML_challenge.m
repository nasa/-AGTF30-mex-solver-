% This file reformats information for use in machine-learning challenge problems

if ~exist('outputs_challenge_problem', 'var')
    % Initialize machine-learning challenge problem outputs structure
    outputs_challenge_problem = repmat(struct('altitude', NaN, ...
    'mach_number', NaN, ...
    'U', NaN, ...
    'Y', NaN), ...
    num_inputs, 1);
end

outputs_challenge_problem(input_num).altitude = altitude_sensed; % altitude_sensed
outputs_challenge_problem(input_num).mach_number = mach_number_sensed; % mach_number_sensed

outputs_challenge_problem(input_num).U = [ U(1); % wf
                                solver_initial_guess(9); % VAFN
                                solver_initial_guess(10)]; % VBV

outputs_challenge_problem(input_num).Y = [ Y(2); % LP shaft sensed speed
                                Y(3); % HP shaft sensed speed
                                Pamb_sensed; % Pamb
                                Tt2_sensed; % Tt2
                                Pt2_sensed; % Pt2
                                Y(35); % Tt25
                                Y(36); % Pt25
                                Y(38); % Tt3
                                Y(40); % Ps3
                                Y(45); % Tt45
                                Y(51)];% Tt5

