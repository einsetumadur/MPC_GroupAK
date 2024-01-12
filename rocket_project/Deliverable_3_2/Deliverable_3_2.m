close all;
clear;
clc;

% setting up rocket
Ts = 1/20;
rocket = Rocket(Ts);
[xs, us] = rocket.trim();
sys = rocket.linearize(xs, us);
[sys_x, sys_y, sys_z, sys_roll] = rocket.decompose(sys, xs, us);

% setting up mpc controllers
H = 5.0;
mpc_x = MpcControl_x(sys_x, Ts, H);
mpc_y = MpcControl_y(sys_y, Ts, H);
mpc_z = MpcControl_z(sys_z, Ts, H);
mpc_roll = MpcControl_roll(sys_roll, Ts, H);

% setting the refs
Tf = 7.0;
x0 = zeros(4, 1);
ref_x = -4;
y0 = zeros(4, 1);
ref_y = -4;
z0 = zeros(2, 1);
ref_z = -4;
roll0 = zeros(2, 1);
ref_roll = deg2rad(35);

% plotting
% plotting x
[T, X_sub, U_sub] = rocket.simulate_f(sys_x, x0, Tf, @mpc_x.get_u, ref_x);
[~, T_opt, X_opt, U_opt] = mpc_x.get_u(x0, ref_x);
% taking trimming into account
X_opt = X_opt + [xs(2); xs(5); xs(7); xs(10)];
U_opt = U_opt + us(2);
U_opt(end+1) = NaN;
rocket.plotvis_sub(T, X_sub, U_sub, sys_x, xs, us, ref_x);
rocket.plotvis_sub(T_opt, X_opt, U_opt, sys_x, xs, us, ref_x);

% plotting y
[T, X_sub, U_sub] = rocket.simulate_f(sys_y, y0, Tf, @mpc_y.get_u, ref_y);
[~, T_opt, X_opt, U_opt] = mpc_y.get_u(y0, ref_y);
X_opt = X_opt + [xs(1); xs(4); xs(8); xs(11)];
U_opt = U_opt + us(1);
U_opt(end+1) = NaN;
rocket.plotvis_sub(T, X_sub, U_sub, sys_y, xs, us, ref_y);
rocket.plotvis_sub(T_opt, X_opt, U_opt, sys_y, xs, us, ref_y);

%plotting z
[T, X_sub, U_sub] = rocket.simulate_f(sys_z, z0, Tf, @mpc_z.get_u, ref_z);
[~, T_opt, X_opt, U_opt] = mpc_z.get_u(z0, ref_z);
X_opt = X_opt + [xs(9); xs(12)];
U_opt = U_opt + us(3);
U_opt(end+1) = NaN;
rocket.plotvis_sub(T, X_sub, U_sub, sys_z, xs, us, ref_z);
rocket.plotvis_sub(T_opt, X_opt, U_opt, sys_z, xs, us, ref_z);

%plotting roll
[T, X_sub, U_sub] = rocket.simulate_f(sys_roll, roll0, Tf, @mpc_roll.get_u, ref_roll);
[~, T_opt, X_opt, U_opt] = mpc_roll.get_u(roll0, ref_roll);
X_opt = X_opt + [xs(3); xs(6)];
U_opt = U_opt + us(4);
U_opt(end+1) = NaN;
rocket.plotvis_sub(T, X_sub, U_sub, sys_roll, xs, us, ref_roll);
rocket.plotvis_sub(T_opt, X_opt, U_opt, sys_roll, xs, us, ref_roll);