% nr_solver.m
% Written by Thomas Kennings and Donald Simon
% NASA Glenn Research Center, Cleveland, OH
% May 30, 2024

% This function iteratively executes the MEX engine model, varying
% independent variables to drive dependent variables to zero.

function [DEP,CMD,X,U,Y,E,converged, solver_iterations] = nr_solver(ENV_IN,CMD_IN,TAR_OUT,HEALTH_PARAMS_IN,BLDS_IN,Ivec,Dvec,ENABLE_DEBUG)
%% Set solver parameters
% Arrays have sets of parameters which are progressively used by the
% solver as needed. For example, if the parameters in the first indices 
% don't provide convergence, the solver will try using the parameters 
% in the second indices.
MaxIter_array = [20 100]; % Maximum iterations before giving up
NRASS_array = [10 5]; % Number of iterations before recalculating Jacobian
JPerSS_array = [0.001 0.001]; % Jacobian perturbation size
NumJPerSS_array = MaxIter_array; % Number of iterations before Jacobian perturbation size is adjusted

for i=1:length(MaxIter_array)
    % Define Independent Vector Min/Max Range -
    IMinMax_array(i, :, :) = ...
        [0       Inf;  % 1) WIn 
        1       3.5;  % 2) FAN_RLIn 
        1       3.2;  % 3) LPC_RLIn 
        1       3.0;  % 4) HPC_RLIn 
        1       Inf;  % 5) BPR 
        2.5478  6.417;  % 6) HPT_PR [turbine PR range = ("PRvec range" - 1) * s_T_PR + 1] 
        1       11.785;  % 7) LPT_PR [turbine PR range = ("PRvec range" - 1) * s_T_PR + 1] 
        0       Inf;  % 8) WfIn 
        0       Inf;  % 9) VAFNIn 
        0       Inf;  % 10) VBVIn 
        0       Inf;  % 11) N2In 
        0       Inf;  % 12) N3In 
        -Inf    Inf;  % 13) HPpwrIn 
        -Inf    Inf];  % 14) LPpwrIn 
    
    % Define Dependent Vector Tolerances -
    Dtol_array(i, :, :) = ...
        [1e-5;  % W21err 
        1e-5;  % W24aerr 
        1e-5;  % W36err 
        1e-5;  % W45err 
        1e-5;  % W5err 
        1e-5;  % W8err 
        1e-5;  % W18err 
        1e-5;  % N2dot 
        1e-5;  % N3dot 
        1e-5;  % LPC SM error 
        1e-5;  % Fnet error 
        1e-5]; % T45 error 
    
    % Define Component Map Range
    MapRange_array(i, :, :) = ...
        [0.3 1.10; % FAN NcMap
        0.3 1.25; % LPC NcMap
        0.5 1.05; % HPC NcMap
        60  130;  % HPT NcMap
        20  120]; % LPT NcMap
end

%% Make sure number of independents equals number of dependents
if (sum(Ivec) ~= sum(Dvec))
    if ENABLE_DEBUG
    disp('Must have same number of Independents and Dependents!');
    end
    DEP = NaN;
    CMD = NaN;
    X = NaN;
    U = NaN;
    Y = NaN;
    E = NaN;
    converged = 0;
    solver_iterations = 0;
    return;
end

%% Run solver with each set of parameters specified
for solver_paramater_index = 1:length(MaxIter_array)
    solver_iterations = 0;
    
    IMinMax = squeeze(IMinMax_array(solver_paramater_index, :, :));
    Dtol = squeeze(Dtol_array(solver_paramater_index, :, :));
    MapRange = squeeze(MapRange_array(solver_paramater_index, :, :));
    MaxIter = MaxIter_array(solver_paramater_index);
    NRASS = NRASS_array(solver_paramater_index);
    JPerSS = JPerSS_array(solver_paramater_index);
    NumJPerSS = NumJPerSS_array(solver_paramater_index);

    %% Initial call to mex function
    CMD = CMD_IN;
    
    CMD(CMD > IMinMax(:,2)) = IMinMax(CMD > IMinMax(:,2),2); % Set any max violations to maximum 
    CMD(CMD < IMinMax(:,1)) = IMinMax(CMD < IMinMax(:,1),1); % Set any min violations to minimum 
    
    [DEP,X,U,Y,E] = MEX_engine_model(ENV_IN, CMD, TAR_OUT, HEALTH_PARAMS_IN, BLDS_IN, ENABLE_DEBUG);
    
    % check for convergence 
    if (max(abs(DEP(Dvec) ./ Dtol(Dvec))) < 1.0)
        converged = 1;
        return;
    end
    
    %% Initial Jacobian calculation 
    Jpos = NaN(sum(Ivec),sum(Dvec)); % Initialize positive perturbation matrix 
    Jneg = NaN(sum(Ivec),sum(Dvec)); % Initialize negative perturbation matrix 
    J = NaN(sum(Ivec),sum(Dvec)); % Initialize total perturbation matrix 
    
    Ivec_range = find(Ivec);
    Dvec_range = find(Dvec);
    
    DEP0 = DEP;     % DEP0 and CMD0 permanently represent the unperturbed dependents and independents
    CMD0 = CMD_IN;

    % Positive Perturbation Matrix Calculation 
    for i1 = 1:sum(Ivec)
        CMD = CMD0;
        CMD(Ivec_range(i1)) = CMD(Ivec_range(i1)) * (1 + JPerSS);
    
        if (CMD(Ivec_range(i1)) <= IMinMax(Ivec_range(i1),2))
            [DEP,X,U,Y,E] = MEX_engine_model(ENV_IN, CMD, TAR_OUT, HEALTH_PARAMS_IN, BLDS_IN, ENABLE_DEBUG);
            
            % check for convergence 
            if (max(abs(DEP(Dvec) ./ Dtol(Dvec))) < 1.0)
                converged = 1;
                return;
            end

            Jpos(:,i1) = (DEP(Dvec_range) - DEP0(Dvec_range)) / (CMD(Ivec_range(i1))*JPerSS);
        end
    end
    
    % Negative Perturbation Matrix Calculation 
    for i1 = 1:sum(Ivec)
        CMD = CMD0;
        CMD(Ivec_range(i1)) = CMD(Ivec_range(i1)) * (1 - JPerSS);
    
        if (CMD(Ivec_range(i1)) >= IMinMax(Ivec_range(i1),1))
            [DEP,X,U,Y,E] = MEX_engine_model(ENV_IN, CMD, TAR_OUT, HEALTH_PARAMS_IN, BLDS_IN, ENABLE_DEBUG);
            
            % check for convergence 
            if (max(abs(DEP(Dvec) ./ Dtol(Dvec))) < 1.0)
                converged = 1;
                return;
            end
            
            Jneg(:,i1) = (DEP(Dvec_range) - DEP0(Dvec_range)) / (-1.0*CMD(Ivec_range(i1))*JPerSS);
        end
    end
    
    % Form Jacobian
    for Jcol = 1:sum(Ivec)
        if all(isfinite(Jpos(:,Jcol))) && all(isfinite(Jneg(:,Jcol)))
            J(:,Jcol) = (Jpos(:,Jcol) + Jneg(:,Jcol))/2;
        elseif all(isfinite(Jpos(:,Jcol)))
            J(:,Jcol) = Jpos(:,Jcol);
        elseif all(isfinite(Jneg(:,Jcol)))
            J(:,Jcol) = Jneg(:,Jcol);
        else
            if ENABLE_DEBUG
            disp("Cannot form invertible Jacobian matrix.");
            end
            continue;
        end
    end
    Jinv = inv(J);
    
    %% Iterate until convergence reached or MaxIter reached 
    while (solver_iterations < MaxIter)
        solver_iterations = solver_iterations + 1;
    
        CMD(Ivec_range) = CMD0(Ivec_range) - Jinv * DEP0(Dvec_range);
    
        % If VBV independent active, make sure VBV is > 0. Otherwise convergence issues will arise 
        if ((Ivec(10) == 1) && (CMD(10) <= 0))
            CMD(10) = 0.0001;
        end
        
        CMD(CMD > IMinMax(:,2)) = IMinMax(CMD > IMinMax(:,2),2); % Set any max violations to maximum 
        CMD(CMD < IMinMax(:,1)) = IMinMax(CMD < IMinMax(:,1),1); % Set any min violations to minimum 
        [DEP,X,U,Y,E] = MEX_engine_model(ENV_IN, CMD, TAR_OUT, HEALTH_PARAMS_IN, BLDS_IN, ENABLE_DEBUG);
        
        % check for convergence 
        if (max(abs(DEP(Dvec) ./ Dtol(Dvec))) < 1.0)
            converged = 1;
            return;
        end
    
        % Check for component map violation. Return non convergence on any map violations 
        if ((max(E(3:7) ./ MapRange(:,2)) > 1.0) || (min(E(3:7) ./ MapRange(:,1)) < 1.0))
            if ENABLE_DEBUG
            display(['Component map violation with parameter index ' num2str(solver_paramater_index) ' NcMaps: ' num2str(E(3)) ' ' num2str(E(4)) ' ' num2str(E(5)) ' ' num2str(E(6)) ' ' num2str(E(7))]);
            end
            continue;
        end
    
        % Update baselines for command and dependent vectors 
        CMD0 = CMD;
        DEP0 = DEP;

        % check if Jacobian perturbation size should be adjusted 
        if (rem(solver_iterations,NumJPerSS) == 0)
            JPerSS = JPerSS/10;
        end
    
        % Update Jacobian every NRASS iterations 
        if (rem(solver_iterations,NRASS) == 0)
    
            Jpos = NaN(sum(Ivec),sum(Dvec)); % Initialize positive perturbation matrix 
            Jneg = NaN(sum(Ivec),sum(Dvec)); % Initialize negative perturbation matrix 
    
            % Positive Perturbation Matrix Calculation 
            for i1 = 1:sum(Ivec)
                CMD = CMD0;
                CMD(Ivec_range(i1)) = CMD(Ivec_range(i1)) * (1 + JPerSS);
    
                if (CMD(Ivec_range(i1)) <= IMinMax(Ivec_range(i1),2))
                    [DEP,X,U,Y,E] = MEX_engine_model(ENV_IN, CMD, TAR_OUT, HEALTH_PARAMS_IN, BLDS_IN, ENABLE_DEBUG);
                    
                    % check for convergence 
                    if (max(abs(DEP(Dvec) ./ Dtol(Dvec))) < 1.0)
                        converged = 1;
                        return;
                    end
                    
                    Jpos(:,i1) = (DEP(Dvec_range) - DEP0(Dvec_range)) / (CMD(Ivec_range(i1))*JPerSS);
                end
            end
            
            % Negative Perturbation Matrix Calculation 
            for i1 = 1:sum(Ivec)
                CMD = CMD0;
                CMD(Ivec_range(i1)) = CMD(Ivec_range(i1)) * (1 - JPerSS);
    
                if (CMD(Ivec_range(i1)) >= IMinMax(Ivec_range(i1),1))
                    [DEP,X,U,Y,E] = MEX_engine_model(ENV_IN, CMD, TAR_OUT, HEALTH_PARAMS_IN, BLDS_IN, ENABLE_DEBUG);
                    
                    % check for convergence 
                    if (max(abs(DEP(Dvec) ./ Dtol(Dvec))) < 1.0)
                        converged = 1;
                        return;
                    end
                    
                    Jneg(:,i1) = (DEP(Dvec_range) - DEP0(Dvec_range)) / (-1.0*CMD(Ivec_range(i1))*JPerSS);
                end
            end
            
            for Jcol = 1:sum(Ivec)
                if all(isfinite(Jpos(:,Jcol))) && all(isfinite(Jneg(:,Jcol)))
                    J(:,Jcol) = (Jpos(:,Jcol) + Jneg(:,Jcol))/2;
                elseif all(isfinite(Jpos(:,Jcol)))
                    J(:,Jcol) = Jpos(:,Jcol);
                elseif all(isfinite(Jneg(:,Jcol)))
                    J(:,Jcol) = Jneg(:,Jcol);
                else
                    if ENABLE_DEBUG
                    disp("Cannot form invertible Jacobian matrix.");
                    end
                    continue;
                end
            end
            Jinv = inv(J);
    
        end
    
    end

end % while loop

%% = Reaching this point means convergence not achieved. Return zero. 
converged = 0;
return;








