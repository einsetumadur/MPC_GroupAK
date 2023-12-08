addpath(fullfile('..', 'src'));

close all;
clear;
clc;

Ts = 1/20; % Sample time
rocket = Rocket(Ts);
[xs, us] = rocket.trim();
sys = rocket.linearize(xs, us);
[sys_x, sys_y, sys_z, sys_roll] = rocket.decompose(sys, xs, us);

% Design MPC controller
H = 3; % Horizon length in seconds
mpc_x = MpcControl_x(sys_x, Ts, H);

% Evaluate once and plot optimal open−loop trajectory,
% pad last input to get consistent size with time and state
x_x = [0, 0, 0, 3]';
[u, T_opt, X_opt, U_opt] = mpc_x.get_u(x_x);
U_opt(:,end+1) = NaN;
% Account for linearization point
X_opt = X_opt + xs;
U_opt = U_opt + us;
ph = rocket.plotvis_sub(T_opt, X_opt, U_opt, sys_x, xs, us); % Plot as usual

%% TODO: This file should produce all the plots for the deliverable
