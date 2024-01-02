addpath(fullfile('..', 'src'));

close all;
clear;
clc;

% initialise the rocket
Ts = 1/20;
rocket = Rocket(Ts);
[xs, us] = rocket.trim();
sys = rocket.linearize(xs, us);
[sys_x, sys_y, sys_z, sys_roll] = rocket.decompose(sys, xs, us);

%instantiate the mpc classes
H = 3.5;
mpc_x = MpcControl_x(sys_x, Ts, H);
mpc_y = MpcControl_y(sys_y, Ts, H);
mpc_z = MpcControl_z(sys_z, Ts, H);
mpc_roll = MpcControl_roll(sys_roll, Ts, H);

%set params
Tf = 7.0;
x0 = [0, 0, 0, 3]';
y0 = [0, 0, 0, 3]';
z0 = [0, 3]';
roll0 = [0, deg2rad(40)]';

% calculate open and closed loop for x
[T, X_sub, U_sub] = rocket.simulate_f(sys_x, x0, Tf, @mpc_x.get_u, 0);
[~, T_opt, X_opt, U_opt] = mpc_x.get_u(x0);
%take into account linearization point
X_opt = X_opt + [xs(2); xs(5); xs(7); xs(10)];
U_opt = U_opt + us(2);
U_opt(:, end+1) = NaN;
rocket.plotvis_sub(T, X_sub, U_sub, sys_x, xs, us);
rocket.plotvis_sub(T_opt, X_opt, U_opt, sys_x, xs, us);

% calculate open and closed loop for y
[T, X_sub, U_sub] = rocket.simulate_f(sys_y, y0, Tf, @mpc_y.get_u, 0);
[~, T_opt, X_opt, U_opt] = mpc_y.get_u(y0);
X_opt = X_opt + [xs(1); xs(4); xs(8); xs(11)];
U_opt = U_opt + us(1);
U_opt(end+1) = NaN;
rocket.plotvis_sub(T, X_sub, U_sub, sys_y, xs, us);
rocket.plotvis_sub(T_opt, X_opt, U_opt, sys_y, xs, us);

% calculate open and closed loop for z
[T, X_sub, U_sub] = rocket.simulate_f(sys_z, z0, Tf, @mpc_z.get_u, 0);
[~, T_opt, X_opt, U_opt] = mpc_z.get_u(z0);
X_opt = X_opt + [xs(9); xs(12)];
U_opt = U_opt + us(3);
U_opt(end+1) = NaN;
rocket.plotvis_sub(T, X_sub, U_sub, sys_z, xs, us);
rocket.plotvis_sub(T_opt, X_opt, U_opt, sys_z, xs, us);

% calculate open and closed loop for roll
[T, X_sub, U_sub] = rocket.simulate_f(sys_roll, roll0, Tf, @mpc_roll.get_u, 0);
[~, T_opt, X_opt, U_opt] = mpc_roll.get_u(roll0);
X_opt = X_opt + [xs(3); xs(6)];
U_opt = U_opt + us(4);
U_opt(end+1) = NaN;
rocket.plotvis_sub(T, X_sub, U_sub, sys_roll, xs, us);
rocket.plotvis_sub(T_opt, X_opt, U_opt, sys_roll, xs, us);