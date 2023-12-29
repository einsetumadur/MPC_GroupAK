
%close all
%clear all
%clc

%% TODO: This file should produce all the plots for the deliverable

Ts = 1/20;
rocket = Rocket(Ts);

[xs, us] = rocket.trim();
sys = rocket.linearize(xs, us);
[sys_x, sys_y, sys_z, sys_roll] = rocket.decompose(sys, xs, us);
H= 5.0;

mpc_x = MpcControl_x(sys_x, Ts, H);
mpc_y = MpcControl_y(sys_y, Ts, H);
mpc_z = MpcControl_z(sys_z, Ts, H);
mpc_roll = MpcControl_roll(sys_roll, Ts, H);

mpc = rocket.merge_lin_controllers(xs, us, mpc_x, mpc_y, mpc_z, mpc_roll);
x0 = zeros(12,1);
rocket.anim_rate = 0.2;
Tf= 7.0;
ref = @(t_ , x_ ) ref_TVC(t_ );
% Manipulate mass for simulation
rocket.mass = 2.0;
[T, X, U, Ref, Z_hat] = rocket.simulate_est_z(x0, Tf, @mpc.get_u, ref, mpc_z, sys_z);
ph = rocket.plotvis(T, X, U, Ref);