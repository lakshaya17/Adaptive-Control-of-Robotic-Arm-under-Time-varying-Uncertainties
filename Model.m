function [dx] = Model(t,x)
global m2_t M2_est
 % Uncomment for the cases mentioned 
% Case 1 
m2_t = sin(t) + 3;

%Case 2 
%m2_t = 3;  

%Case 3
%m2_t = 0.5*sin(3*t) + 3;


% desired trajectories

%Desired Trajector 1
%q_d = [sin(t);sin(t)];  % q_d = [theta_1d;theta_2d]
%q_d_dot = [cos(t);cos(t)]; % q_d dot
%q_d_ddot = [-sin(t);-sin(t)]; % q_d ddot

%Desired Trajectory 2 
q_d = [0;sin(t)];  % q_d = [theta_1d;theta_2d]
q_d_dot = [0;cos(t)]; % q_d dot
q_d_ddot = [0;-sin(t)]; % q_d ddot




q = x(1:2);
q_dot = x(3:4);


global lambda e_q de_q acc vel e m1 l1 l2 g

lambda = 100;

e_q = q - q_d;
de_q = q_dot - q_d_dot;
acc = q_d_ddot - (lambda*de_q);
vel = q_d_dot - (lambda*e_q);
e = de_q + (lambda*e_q);


global error_j error_r
error_j = [error_j e_q];  % Joint tracking error vector
error_r = [error_r e]; % Reference velocity error vector

Q_m2 = 8*eye(11);  % Adaptive gain matrix


global M_1 G_1 M_2 C_2 G_2 M C G P_m P_g P_c

% Model dynamics (Actual)
%for link 1
M_1 = [(1/3)*m1*(l1)^2  0 ;0 0]; %2x2 
G_1 = [(1/2)*m1*g*l1*cos(x(1));0]; %2x1

%for link 2
P_m = [l1^2+(1/3)*l2^2+l1*l2*cos(x(2)) (1/3)*l2^2+(1/2)*l1*l2*cos(x(2));(1/3)*l2^2+(1/2)*l1*l2*cos(x(2)) (1/3)*l2^2]; %2x2
P_c = [0 -((1/2)*l1*l2*sin(x(2))*x(4)+l1*l2*sin(x(2))*x(3));(1/2)*l1*l2*sin(x(2))*x(3) 0]; %2x2
P_g = [(1/2)*g*l1*cos(x(1))+(1/2)*g*l2*cos(x(1)+x(2));(1/2)*g*l2*cos(x(1)+x(2))];  % 2x1 

M_2 = m2_t*P_m;
G_2 = m2_t*P_g;
C_2 = m2_t*P_c;
M = M_1 + M_2;
C = C_2;
G = G_1 + G_2;

M_inv = inv(M);
M_C_inv = inv(M)*C;

% Fourier Series for FAT
Z_m2 = [(1/2); cos((pi*(t))/5);sin((pi*(t))/5);cos((2*pi*(t))/5);sin((2*pi*(t))/5);...
    cos((3*pi*(t))/5);sin((3*pi*(t))/5);cos((4*pi*(t))/5);sin((4*pi*(t))/5);...
    cos((5*pi*(t))/5);sin((5*pi*(t))/5)];

%Z_m2 = [1;cos((pi*(t))/5)*sin((pi*(t))/5);cos((2*pi*(t))/5)*sin((2*pi*(t))/5);cos((3*pi*(t))/5)*sin((3*pi*(t))/5);...
 %    cos((4*pi*(t))/5)*sin((4*pi*(t))/5);cos((5*pi*(t))/5)*sin((5*pi*(t))/5);cos((6*pi*(t))/5)*sin((6*pi*(t))/5);...
 %    cos((7*pi*(t))/5)*sin((7*pi*(t))/5);cos((8*pi*(t))/5)*sin((8*pi*(t))/5);cos((9*pi*(t))/5)*sin((9*pi*(t))/5);...
  %   cos((10*pi*(t))/5)*sin((10*pi*(t))/5)];

m2_t_tilde = x(5);

% Model Equations (Estimated)
global M_tilde C_tilde G_tilde M_2_tilde C_2_tilde G_2_tilde

M_2_tilde = m2_t_tilde*P_m;
C_2_tilde = m2_t_tilde*P_c;
G_2_tilde = m2_t_tilde*P_g;

M_tilde = M_1 + M_2_tilde;
C_tilde = C_2_tilde;
G_tilde = G_1 + G_2_tilde;

% Control
tau = Torque_control(q_d, q_d_dot, q_d_ddot, q, q_dot);

global W_m2_up


dx=zeros(5,1);
dx(1) = x(3);
dx(2) = x(4);
dx(3:4) = -M_C_inv* x(3:4) - M_inv*G + M_inv*tau; 

W_m2_tilde = -inv(Q_m2)*[Z_m2*transpose(e)*P_m*acc + Z_m2*transpose(e)*P_c*vel + Z_m2*transpose(e)*P_g];  %% W_m2 estimated- update law 
W_m2_up = [W_m2_up W_m2_tilde]; 

dx(5) = transpose(W_m2_tilde)*Z_m2;

end