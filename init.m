clear all;

load('supply.mat');
load('supplyABC.mat');
load('thrusters_sup.mat');

% Initial position x, y, z, phi, theta, psi
eta0 = [0,0,0,0,0,0]';
% Initial velocity u, v, w, p, q, r
nu0 = [0,0,0,0,0,0]';

%% Current 
V_c = 0; % Currrent magnitude
beta_c = 135; % Current angle degrees

%% Lineary varying current angle
switch_condition = 1; % 0: constant, 1: lineary varying 

%% Setpoints and simulations
% Need to change the current magnitude, current angle and switch condition
% depndent on which simulation 

% Simulation 1 & 2
setPoint_sim12 = [0 0 0]; 

% Simulation 3
init_setPoint = [0 0 0];
setPoint_change_time = 100;
setPoint_sim3= [10 10 3*pi/2];

% Simulation 4
setPoint0 = [0 0 0];
setPoint1 = [50 0 0];
setPoint2 = [50 -50 0];
setPoint3 = [50 -50 -pi/4];
setPoint4 = [0 -50 -pi/4];
setPoint5 = [0 0 0];

%% Tunning of reference model
omg = [0.12 0.12 0.08];
zeta = [1.32 1.32 1.4];

omg2 = omg.*omg;
Z2omg = 2*omg.*zeta;
satMax = [3 3 0.05];


%% Tunning of controller
Kp = [4e7 4e7 2e8]; % Kp = [kp_surge kp_sway kp_yaw];
Kd = [7e6 7e6 1e9]; % Kd = [kd_surge kd_sway kd_yaw];
Ki = [3e6 3e6 1e6]; % Ki = [ki_surge ki_sway ki_yaw];

% Initial tuning values
% Kp = [1.1e5 1.35e5 5.9e7]; % Kp = [kp_surge kp_sway kp_yaw];
% Kd = [1.23e6 1.5e6 6.6e8]; % Kd = [kd_surge kd_sway kd_yaw];
% Ki = [1.1e4 1.35e4 5.9e6]; % Ki = [ki_surge ki_sway ki_yaw];


%% Simulation
t_set = 800;
dt = 0.1;   

%sim("part2.slx");

