addpath(fullfile('..', 'src'));

close all;
clear;
clc;

Ts = 1/20; % Sample time
rocket = Rocket(Ts);
[xs, us] = rocket.trim();
sys = rocket.linearize(xs, us);
[sys_x, sys_y, sys_z, sys_roll] = rocket.decompose(sys, xs, us);

%% Z Control
H = 3;
mpc_z = MpcControl_z(sys_z,Ts,H);
x_z = [0,-3]';

% [u_z, T_opt, X_opt, U_opt] = mpc_z.get_u(x_z);
% U_opt(:,end+1) = NaN;
% X_opt = X_opt + xs([9,12]);
% U_opt = U_opt + us(3);
% ph = rocket.plotvis_sub(T_opt, X_opt, U_opt, sys_z, xs, us);

Tf = 7.0;
x_z0 = [0,-3]';
xref = 0;
[T, X_sub, U_sub] = rocket.simulate_f(sys_z, x_z0, Tf, @mpc_z.get_u,xref);
ph = rocket.plotvis_sub(T, X_sub, U_sub, sys_z, xs, us);


%% X Control
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
