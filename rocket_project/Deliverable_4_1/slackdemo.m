addpath(fullfile('..', 'src'));

close all;
clear;
clc;

Ts = 1/20;
rocket = Rocket(Ts);
[xs, us] = rocket.trim();
sys = rocket.linearize(xs, us);
[sys_x, sys_y, sys_z, sys_roll] = rocket.decompose(sys, xs, us);

%% controller setup
Tf = 7;
H = 5.0;

mpc_x = MpcControl_x(sys_x, Ts, H);
mpc_y = MpcControl_y(sys_y, Ts, H);
mpc_z = MpcControl_z(sys_z, Ts, H);
mpc_roll = MpcControl_roll(sys_roll, Ts, H);
% Merge four sub−system controllers into one full−system controller
mpc = rocket.merge_lin_controllers(xs, us, mpc_x, mpc_y, mpc_z, mpc_roll);

%% Linear simulation
lx0 = zeros(4, 1);
ref_x = 10;
rocket.anim_rate = 1.0;
[T, X_sub, U_sub] = rocket.simulate_f(sys_x, lx0, Tf, @mpc_x.get_u, ref_x);
rocket.plotvis_sub(T, X_sub, U_sub, sys_x, xs, us, ref_x);

%% Non Linear Simulation
x0 = zeros(12, 1);
refx = [10,0, 0, 0]';

Tf = 7;
[T, X, U, Ref] = rocket.simulate(x0, Tf, @mpc.get_u, refx);

% Visualize
rocket.anim_rate = 1; % Increase this to make the animation faster
ph = rocket.plotvis(T, X, U, Ref);
ph.fig.Name = 'Merged lin. MPC in nonlinear simulation'; % Set a figure title
