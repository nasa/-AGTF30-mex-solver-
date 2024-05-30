% load_inputs_from_csv.m
% Written by Thomas Kennings
% NASA Glenn Research Center, Cleveland, OH
% May 30, 2024

% This script loads in input data from the excel file "inputs.xlsx".
% The input data represents the operating conditions of
% the engine to be evaluated.

function [inputs_array, num_inputs] = load_inputs_from_csv()
    %% Load inputs from file
    inputs = readmatrix('inputs.csv');
    num_inputs = size(inputs, 1); 

    %% Preallocate inputs_array object
    inputs_array = repmat(struct('altitude', NaN, ...
        'mach_number', NaN, ...
        'N1c', NaN, ...
        'dTamb', NaN, ...
        'health_params', NaN), ...
        num_inputs, 1);
    
    %% Package into inputs_array
    for row = 1:num_inputs
        inputs_array(row).altitude = inputs(row, 1);
        inputs_array(row).mach_number = inputs(row, 2);
        inputs_array(row).N1c = inputs(row, 3);
        inputs_array(row).dTamb = inputs(row, 4);
        inputs_array(row).health_params = inputs(row, 5:17);
    end
end
