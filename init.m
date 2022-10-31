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
switch_condition = 0; % 0: constant, 1: lineary varying 

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
Kp = [0.009e7 0.017e7 7e7]; % Kp = [kp_surge kp_sway kp_yaw];
Kd = [0.0007e8 0.0011e8 5.5908e8]; % Kd = [kd_surge kd_sway kd_yaw];
Ki = [0.0044e5 0.0141e5 2.342e5]; % Ki = [ki_surge ki_sway ki_yaw];

% Kp = [1e6 1.35e5 5.90e7]; % Kp = [kp_surge kp_sway kp_yaw];
% Kd = [7e6 1.50e6 6.60e8]; % Kd = [kd_surge kd_sway kd_yaw];
% Ki = [1e4 1.35e4 5.90e6]; % Ki = [ki_surge ki_sway ki_yaw];

% Kp = [4e7 4e7 2e8]; % Kp = [kp_surge kp_sway kp_yaw];
% Kd = [7e6 7e6 1e9]; % Kd = [kd_surge kd_sway kd_yaw];
% Ki = [1e6 3e6 1e6]; % Ki = [ki_surge ki_sway ki_yaw];

% Initial tuning values
% Kp = [1.1e5 1.35e5 5.9e7]; % Kp = [kp_surge kp_sway kp_yaw];
% Kd = [1.23e6 1.5e6 6.6e8]; % Kd = [kd_surge kd_sway kd_yaw];
% Ki = [1.1e4 1.35e4 5.9e6]; % Ki = [ki_surge ki_sway ki_yaw];

%% Passive nonlinear observer

% Mass matrix
M = [6.8177e6 0 0; 0 7.8784e6 -2.5955e6; 0 -2.5955e6 3.57e9];

% Damping matrix
D = [2.6486e5 0 0; 0 8.8164e5 0; 0 0 3.3774e8];

% Bias time constants
Tb = diag([1000,1000,1000]);

% Tuning of wave-estimator
T_i = 6; % Ti should be in the interval from 5s to 20s.
omega_i = 2*pi/T_i;

% zeta_i should be in the interval from 0.05 to 0.10
zeta1 = 0.05;
zeta2 = 0.075;
zeta3 = 0.10;

% Aw
Aw = [0 0 0 1 0 0;
    0 0 0 0 1 0;
    0 0 0 0 0 1;
    -diag([omega_i,omega_i,omega_i]).^2 -2*diag([zeta1,zeta2,zeta3])*diag([omega_i,omega_i,omega_i])];

% Cw
Cw = [0 0 0 1 0 0;
    0 0 0 0 1 0;
    0 0 0 0 0 1];

% Cut off frequency - Should be bigger than natural frequency 1.25-1.3*omega_n 
omega_ci = 1.25*omega_i; 

% Tuning parameters
zeta_ni = 1;
zeta_i = 0.1;

% Tuning of gain matrices
% k1, k2, k3
k_1 = -2*(zeta_ni - zeta_i)*omega_ci/omega_i;
[k_2, k_3] = deal(k_1,k_1);

% k4, k5, k6
k_4 = 2*omega_i*(zeta_ni - zeta_i);
[k_5, k_6] = deal(k_4,k_4);

% k7, k8, k9
[k_7,k_8,k_9] = deal(omega_ci);


K1 = [diag([k_1,k_2,k_3]);  % works good with multipling by 0.005 ?
    diag([k_4,k_5,k_6])];

K2 = diag([k_7,k_8,k_9]);

K4 = diag([diag(M)]); % should be proprional to the mass of each componentet on recomadation from Dong

K3 = 0.05*K4;  % shoud be from be in the interval [0.01, 0.1]


%% Extended kalman filter

% Aw, T, M, D, Cw 

% Ew, Eb, Q, R, H

% Design matrix
H = [Cw eye(3) zeros(3) zeros(3)];

B = [zeros(6,3); zeros(3); zeros(3); inv(M)];

Kw = diag([1,1,1]);

% Disturbance matrix
Ew = [zeros(3); Kw];

% Bias diagonal scaling matrix
Eb = eye(3);

% Disturbance matrix for kalman filter
E = [Ew zeros(6,3);
    zeros(3) zeros(3);
    zeros(3) Eb;
    zeros(3) zeros(3)];

%% tuning matrix
q11 = 1e3; q22 = 1e3; q33 = 0;
q44 = 0; q55 = 0; q66 = 1;
Q_tun = diag([q11, q22, q33, q44, q55, q66]);

r11 = 1; r22 = 1; r33 = 1;
R_tun = diag([r11, r22, r33]);

n = 15;
I = eye(15);

% Initial values
x0 = zeros(1,15);
P0 = eye(15);


%% Simulation
t_set = 800;
dt = 0.1;   
%sim("part2.slx");



