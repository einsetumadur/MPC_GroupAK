addpath(fullfile('..', 'src'));

close all;
clear;
clc;

Ts = 1/20;
rocket = Rocket(Ts);
[xs, us] = rocket.trim();
sys = rocket.linearize(xs, us);
[sys_x, sys_y, sys_z, sys_roll] = rocket.decompose(sys, xs, us);

H = 5.0;

mpc_roll = MpcControl_roll(sys_roll, Ts, H);
mpc_x = MpcControl_x(sys_x, Ts, H);
mpc_y = MpcControl_y(sys_y, Ts, H);
mpc_z = MpcControl_z(sys_z, Ts, H);

% Merge four sub−system controllers into one full−system controller
mpc = rocket.merge_lin_controllers(xs, us, mpc_x, mpc_y, mpc_z, mpc_roll);

%% Z tuning
H = 5.0;
mpc_z = MpcControl_z(sys_z, Ts, H);
Tf = 7;
lz0 = zeros(2, 1);
ref_z = 2;
rocket.anim_rate = 1.0;
[T, X_sub, U_sub] = rocket.simulate_f(sys_z, lz0, Tf, @mpc_z.get_u, ref_z);
rocket.plotvis_sub(T, X_sub, U_sub, sys_z, xs, us, ref_z);

%% Linear simulation x
Tf = 7;
lx0 = zeros(4, 1);
ref_x = 10;
rocket.anim_rate = 1.0;
[T, X_sub, U_sub] = rocket.simulate_f(sys_x, lx0, Tf, @mpc_x.get_u, ref_x);
ph = rocket.plotvis_sub(T, X_sub, U_sub, sys_x, xs, us, ref_x);
ph.fig.Name = 'slack in linear simulation';

%% Non Linear Simulation x
Tf = 7;
x0 = zeros(12, 1);
refx = [2,0,0, 0]';

[T, X, U, Ref] = rocket.simulate(x0, Tf, @mpc.get_u, refx);
rocket.anim_rate = 1.0;
ph = rocket.plotvis(T, X, U, Ref);
ph.fig.Name = 'slack in nonlinear simulation';
