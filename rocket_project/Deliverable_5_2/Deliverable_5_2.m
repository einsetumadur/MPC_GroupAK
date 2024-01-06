addpath(fullfile('..', 'src'));

close all;
clear;
clc;

Ts = 1/20;
rocket = Rocket(Ts);
[xs, us] = rocket.trim();
sys = rocket.linearize(xs, us);
[sys_x, sys_y, sys_z, sys_roll] = rocket.decompose(sys, xs, us);

H = 7.0;
mpc_x = MpcControl_x(sys_x, Ts, H);
mpc_y = MpcControl_y(sys_y, Ts, H);
mpc_roll = MpcControl_roll(sys_roll, Ts, H);
%%
mpc_z = MpcControl_z_plus(sys_z, Ts, H);

% Merge four sub−system controllers into one full−system controller
mpc = rocket.merge_lin_controllers(xs, us, mpc_x, mpc_y, mpc_z, mpc_roll);

x0 = [zeros(1,9), 1 0 3]';

% undisturbed target
ref = [1.2,0,3,0]';

% Simulate
Tf = 7;
rocket.mass = 2.13;
rocket.mass_rate = -0.27;
[T, X, U, Ref] = rocket.simulate_est_z(x0, Tf, @mpc.get_u, ref,mpc_z,sys_z);

% Visualize
rocket.anim_rate = 1; % Increase this to make the animation faster
ph = rocket.plotvis(T, X, U, Ref);
ph.fig.Name = 'Merged lin. MPC in nonlinear simulation with const disturbance'; % Set a figure title

%% TODO: This file should produce all the plots for the deliverable