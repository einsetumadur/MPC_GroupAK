addpath(fullfile('..', 'src'));

close all;
clear;
clc;

Ts = 1/20;
rocket = Rocket(Ts);
[xs, us] = rocket.trim();
sys = rocket.linearize(xs, us);
[sys_x, sys_y, sys_z, sys_roll] = rocket.decompose(sys, xs, us);

H = 2.0;
mpc_x = MpcControl_x(sys_x, Ts, H);
mpc_y = MpcControl_y(sys_y, Ts, H);
mpc_z = MpcControl_z(sys_z, Ts, H);
mpc_roll = MpcControl_roll(sys_roll, Ts, H);

Tf = 7.0;
x0_x = [0, 0, 0, 3]';
x0_y = [0, 0, 0, 3]';
x0_z = [0, 3]';
x0_roll = [0, deg2rad(40)]';

rocket.anim_rate = 2.0;
[T, X_sub, U_sub] = rocket.simulate_f(sys_x, x0_x, Tf, @mpc_x.get_u, 0);
rocket.plotvis_sub(T, X_sub, U_sub, sys_x, xs, us);
[T, X_sub, U_sub] = rocket.simulate_f(sys_y, x0_y, Tf, @mpc_y.get_u, 0);
rocket.plotvis_sub(T, X_sub, U_sub, sys_y, xs, us);
[T, X_sub, U_sub] = rocket.simulate_f(sys_z, x0_z, Tf, @mpc_z.get_u, 0);
rocket.plotvis_sub(T, X_sub, U_sub, sys_z, xs, us);
[T, X_sub, U_sub] = rocket.simulate_f(sys_roll, x0_roll, Tf, @mpc_roll.get_u, 0);
rocket.plotvis_sub(T, X_sub, U_sub, sys_roll, xs, us);

%% TODO: This file should produce all the plots for the deliverable
