clc; 
clear all; 
close all

tic

global m1 l1 l2 m2_t g 

m1 = 0.8; 
l1 = 0.18; 
l2 = 0.26; 
g = 9.81;

%Final time 
tf = 20;

global error_j error_r W_m2_up M2_est


error_j = [];
error_r = [];
W_m2_up = [];
M2_est = [];

%initial condition
x0 = [0.05,0.1,0.05,0.1,5];
%x0 = [-5.61097705566496,15.0142808713243,8104.08344217732,-18245.5811813394,112.198621321705];
%x0 = [-0.0843921140883215,-0.123002831913685,1.27379574360574,0.842356274843916,-390.177852681805];
%x0 = [0 0 0 0 5];
tspan = [0:0.001:tf];
%tspan = [tf:-0.01:0];
%options = odeset('RelTol',1e-4,'AbsTol',[1e-4, 1e-4, 1e-4, 1e-4]);
[T,X] = ode45(@(t,x) Model(t,x),tspan,x0);

% Trajectory Response
figure()
grid on
plot(T, X(:,1),'r-')
grid on
hold on
%plot(T, sin(T), 'k--')
plot(T,0*ones(size(T,1),1), 'k--')
legend('\theta_{1,actual}', '\theta_{1,desired}')
xlabel('time (sec)')
ylabel('\theta_{1} (rad)')
title('Theta-1 for FAT')


figure()
plot(T, X(:,2),'r-')
grid on
hold on
plot(T, sin(T), 'k--')
legend('\theta_{2,actual}','\theta_{2,desired}')
xlabel('time (sec)')
ylabel('\theta_{2} (rad)')
title('Theta-2 for FAT')


% Joint tracking error 

figure()
plot(T,error_j(1,1:length(T)),'r--')
grid on
hold on
plot(T,error_j(2,1:length(T)), 'b--')
legend('Joint 1', 'Joint 2')
xlabel('time (sec)')
ylabel('Tracking Error (rad)')
title('Tracking Error for Joints')

% Error Mass
figure()
plot(T,X(:,5),'r-')
grid on
hold on
plot(T,sin(T)+3, 'k--')
%plot(T,3*ones(size(T,1),1), 'k--')
%aa = 0.5*sin(3*T)+3;
%plot(T,aa, 'k--')
%plot(T, M2_est(1,1:size(T,1)), 'k--')
legend('Estimated Mass', 'Actual Mass')
xlabel('time (sec)')
ylabel('Mass (kg)')
title('Error in Mass')




toc