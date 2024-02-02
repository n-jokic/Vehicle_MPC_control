home_directory = string(java.lang.System.getProperty("user.home"));
casadi_folder = 'casadi-3.6.4-windows64-matlab2018b';
casad_path = fullfile(home_directory, casadi_folder);
addpath(casad_path);

folder = fullfile(cd, '..');
addpath(folder);

%          model parameters:    %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

model_init;

%simularion parameters:

simulation_init;

% create mpc:

N = 20; % prediction horizon
Nc = 2; % control horizon

% weighing matrix (states)
Q = zeros(4, 4); 
Q(1,1) = 10; Q(2,2) = 12; Q(3,3) = 1; Q(4,4)=57;


% weighing matrix (controls)
R = zeros(5,5); 
R(1,1) = 10; R(2,2) = 20e-2; R(3,3) = 20e-2; R(4,4) = 20e-2; R(5,5) = 20e-2;


T_min = -600;
T_max = 600;

delta_min = -10;
delta_max = 10;

full_nonlinear_model_casadi;

rmpath(folder);