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
H = 1; % Horizon length in seconds
mpc_x = MpcControl_x(sys_x, Ts, H);

% Evaluate once and plot optimal open−loop trajectory,
% pad last input to get consistent size with time and state
x_x = [0, 0, 0, 3]';
[u, T_opt, X_opt, U_opt] = mpc_x.get_u(x_x);
U_opt(:,end+1) = NaN;
% Account for linearization point
X_opt = X_opt + [xs(2); xs(5); xs(7); xs(10)];
U_opt = U_opt + us(2);

[T, X_sub, U_sub] = rocket.simulate_f(sys_x, x_x, 5, @mpc_x.get_u, 0);
ph = rocket.plotvis_sub(T, X_sub, U_sub, sys_x, xs, us);

%% TODO: This file should produce all the plots for the deliverable
