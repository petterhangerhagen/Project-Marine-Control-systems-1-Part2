%% last row in f(x) matrix
syms x y yaw

% Defining R(psi)
% R = [cos(psi) -sin(psi) 0;
%     sin(psi) cos(psi) 0;
%     0 0 1];

%df = diff(R,yaw);

%subs(R,yaw,pi);

% Defining M and D matrices
syms m11 m22 m23 m32 m33 d11 d22 d33
M = [m11 0 0; 0 m22 m23; 0 m32 m33];
D = [d11 0 0; 0 d22 0; 0 0 d33];

% Definind nu and b vectors
syms u v r b1 b2 b3
nu = [u; v; r];
b = [b1; b2; b3];
eta = [x y yaw];


% % f_4 = -M^{-1}*D*nu + M^{-1}*R^{T}(psi)*b
% f_4 = -inv(M)*D*nu + inv(M)*transpose(R)*b;
% 
% % f_4 derivateded with respect to eta
% df4_eta = [diff(f_4,x) diff(f_4,y) diff(f_4,psi)];
% 
% % f_4 derivateded with respect to bias
% df4_bias = [diff(f_4,b1) diff(f_4,b2) diff(f_4,b3)];
% 
% % f_4 derivateded with respect to bias
% df4_nu = [diff(f_4,u) diff(f_4,v) diff(f_4,r)];

%%

% Mass matrix
M = [6.8177e6 0 0; 0 7.8784e6 -2.5955e6; 0 -2.5955e6 3.57e9];

% Damping matrix
D = [2.6486e5 0 0; 0 8.8164e5 0; 0 0 3.3774e8];

% Bias time constants
Tb = diag([1000,1000,1000]);

% Tuning of wave-estimator
T_i = 10; % Ti should be in the interval from 5s to 20s.
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




%% Taking out the states
x_hat_k = rand(15,1);

xi = x_hat_k(1:6);
eta = x_hat_k(7:9);
b = x_hat_k(10:12);
nu = x_hat_k(13:15);

y_k = [10; 10; -pi/2];
tau_est = [100; 100; 0; 0; 0; pi/2];


x = eta(1); y = eta(2); psi = eta(3);
u = nu(1); v = nu(2); r = nu(3);
b1 = b(1); b2 = b(2); b3 = b(3);

% Rotation matrix
R = [cos(psi) -sin(psi) 0;
    sin(psi) cos(psi) 0;
    0 0 1];

%% 

df1_depsilon = Aw;
df1_deta = zeros(6,3);
df1_db = zeros(6,3);
df1_dnu = zeros(6,3);


df2_depsilon = zeros(3,6);
df2_deta = [0 0 -sin(psi)*u-cos(psi)*v;
            0 0 cos(psi)*u - sin(psi)*v;
            0 0 r];
df2_db = zeros(3,3);
df2_dnu = R;

df3_depsilon = zeros(3,6);
df3_deta = zeros(3,3);
df3_db = -inv(Tb);
df3_dnu = zeros(3,3);

dm = M(2,2)*M(3,3) - M(2,3)*M(3,2);

df4_depsilon = zeros(3,6);
df4_deta = [0 0 -(b1/M(1,1)) * sin(psi) + (b2/M(1,1)) * cos(psi);
            0 0 -(b1*M(3,3)/dm)*cos(psi) - (b2*M(3,3)/dm)*sin(psi);
            0 0 (b1*M(3,2)/dm)*cos(psi) + (b2*M(3,2)/dm)*sin(psi)];

df4_db =[(1/M(1,1)*cos(psi)) (1/M(1,1)*sin(psi)) 0;
        -(M(3,3)/dm)*sin(psi) (M(3,3)/dm)*cos(psi) -(M(2,3)/dm);
        (M(3,2)/dm)*sin(psi) -(M(3,2)/dm)*cos(psi) (M(2,2)/dm)];
df4_dnu = [-D(1,1)/M(1,1) 0 0;
            0 -D(2,2)*M(3,3)/dm D(3,3)*M(2,3)/dm;
            0 D(2,2)*M(3,2)/dm -D(3,3)*M(2,2)/dm];


% total derivated f(x)
df = [df1_depsilon df1_deta df1_db df1_dnu;
    df2_depsilon df2_deta df2_db df2_dnu;
    df3_depsilon df3_deta df3_db df3_dnu;
    df4_depsilon df4_deta df4_db df4_dnu];


