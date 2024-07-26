# MATLAB Executable (MEX) Steady-State Solver and Linearization Tool for the Advanced Geared Turbofan 30,000lbf (AGTF30)

This is a MATLAB application for analyzing steady-state performance of the NASA Advanced Geared Turbofan 30,000lbf (AGTF30) engine. A modified version of the AGTF30 is included in the tool, with extra inputs representing torques from electric motors on each shaft. Engine models are created using the NASA Toolbox for the Modeling and Analysis of Thermodynamic Systems (T-MATS) and are pre-compiled into a MATLAB Executable (MEX) function for rapid repeated execution. The provided engine mode can be modified, or even swapped for a different engine model, as described in the full user guide.

## Getting Started

### Installation
To use the MEX Steady-State Solver and Linearization Tool, MATLAB must be installed. MATLAB versions R2021b through R2023b were tested and were compatible. Other versions of MATLAB will most likely be compatible with the MEX solver tool, especially if the version is newer than R2023b. 

Once MATLAB is installed, clone the MEX Steady-State Solver and Linearization Tool from the NASA GitHub (https://github.com/nasa/-AGTF30-mex-solver-).


### Usage

NOTE: A detailed user guide is provided in the *documentation* folder.

#### 1. Specify program inputs
Specify a set of operating conditions and engine health parameters in the file *inputs.csv*. Each row represents a single operating condition which will be solved at steady-state. To specify multiple operating conditions, put each operating condition on its own row. The script *load_inputs_from_csv.m* will load the inputs into a struct. The method of loading input data into the struct can be adapted to individual users' needs.

#### 2. Execute the solver routine
Run *solve_at_points.m* using MATLAB. The program will attempt to solve the engine system at each steady-state operating condition specified in inputs.csv.  The program will perform linearization at each operating condition by methodically perturbing model inputs and measuring model outputs until the defined accuracy is achieved. If the solver is unable to find a solution for a given operating condition, it will give up and move on to the next operating condition.

#### 3. Access outputs
The program will output a MATLAB structured array to *outputs.mat*. Output will include:

- The altitude, mach number, corrected fan speed, ambient temperature differential, and health parameters describing the operating condition.
- Plant inputs, internal states, and outputs at the solution
- State-space matrices (A, B, C, D) representing a linearization of the engine at the operating condition
- Other solver-related information

The outputs can be accessed by loading *outputs.mat* into the MATLAB workspace.