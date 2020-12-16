%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% CONFIGURATION 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%Simulation/Implementation time:
T = 400; % Final time
Tsim = 0.01; %Simulation time step:

%Initial state
X0 = [0;0;-0.7854*2];

% Parameters for the reference trajectory
omega = -0.007;%-0.007/8
radius = 2;
Phi = -pi;
x_offset = 2;%-0.5-0.9;
y_offset = 0.0;%-0.5-0.9;

%Define the limit for the state constraint
y_constraint=-0.6;