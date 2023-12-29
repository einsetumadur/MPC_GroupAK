addpath(fullfile('..', 'src'));

%close all
%clear all
%clc

%% TODO: This file should produce all the plots for the deliverable

Ts = 1/20;
rocket = Rocket(Ts);

[xs, us] = rocket.trim();
sys = rocket.linearize(xs, us);
[sys_x, sys_y, sys_z, sys_roll] = rocket.decompose(sys, xs, us);
H= 2.0;

mpc_x = MpcControl_x(sys_x, Ts, H);
mpc_y = MpcControl_y(sys_y, Ts, H);
mpc_z = MpcControl_z(sys_z, Ts, H);
mpc_roll = MpcControl_roll(sys_roll, Ts, H);

mpc = rocket.merge_lin_controllers(xs, us, mpc_x, mpc_y, mpc_z, mpc_roll);
x0 = zeros(12,1);
rocket.anim_rate = 0.2;
ref = [0, 0, 4, 0, 0, 0, 0, 0, 0, 0, 0, 0];
% Manipulate mass for simulation
Tf= 7.0;
ref = @(t_ , x_ ) ref_TVC(t_);
rocket.mass = 2.13;
[T, X, U, Ref] = rocket.simulate(x0, Tf, @mpc.get_u, ref);
ph = rocket.plotvis(T, X, U, Ref);