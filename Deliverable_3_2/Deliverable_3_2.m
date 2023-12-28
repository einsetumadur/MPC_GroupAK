
%close all
%clear all
%clc

%% TODO: This file should produce all the plots for the deliverable
Ts = 1/20;
%todo 2.1
rocket = Rocket(Ts);

[xs, us] = rocket.trim();
sys = rocket.linearize(xs, us);
[sys_x, sys_y, sys_z, sys_roll] = rocket.decompose(sys, xs, us);

H=2.0;
Tf= 7.0;
mpc_z = MpcControl_z(sys_z, Ts, H);
mpc_y = MpcControl_y(sys_y, Ts, H);
mpc_x = MpcControl_x(sys_x, Ts, H);
mpc_roll = MpcControl_roll(sys_roll, Ts, H);


z0 = zeros(2, 1);
ref_z = -4;
y0 = zeros(4,1);
ref_y= -4;
x0= zeros(4, 1);
ref_x = -4;
roll0 = zeros(2,1);
ref_roll = deg2rad(35);

rocket.anim_rate=2.0;
[T, X_sub, U_sub] = rocket.simulate_f(sys_roll, roll0, Tf, @mpc_roll.get_u, ref_roll);
rocket.plotvis_sub(T, X_sub, U_sub, sys_roll, xs, us, ref_roll);

