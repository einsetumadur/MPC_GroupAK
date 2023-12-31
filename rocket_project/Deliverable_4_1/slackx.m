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

Tf = 7.0;
x0 = zeros(4, 1);
ref_x = -4;

rocket.anim_rate = 1.0;
[T, X_sub, U_sub] = rocket.simulate_f(sys_x, x0, Tf, @mpc_x.get_u, ref_x);
rocket.plotvis_sub(T, X_sub, U_sub, sys_x, xs, us, ref_x);

%% TODO: This file should produce all the plots for the deliverable