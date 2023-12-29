addpath(fullfile('..', 'src'));

close all;
clear;
clc;

Ts = 1/20; % Sample time
rocket = Rocket(Ts);
[xs, us] = rocket.trim();
sys = rocket.linearize(xs, us);
[sys_x, sys_y, sys_z, sys_roll] = rocket.decompose(sys, xs, us);

H = 1; % Horizon length in seconds
sTime = 7; %settling time max seven seconds

%% Z Control
% Design MPC controller
mpc_z = MpcControl_z(sys_z, Ts, H);

% Evaluate once and plot optimal open−loop trajectory,
% pad last input to get consistent size with time and state
x_z = [0, 3]';
[u, T_opt, X_opt, U_opt] = mpc_z.get_u(x_z);
U_opt(:,end+1) = NaN;
% Account for linearization point
X_opt = X_opt + [xs(9); xs(12);];
U_opt = U_opt + us(3);

[T, X_sub, U_sub] = rocket.simulate_f(sys_z,x_z, sTime, @mpc_z.get_u, 0);
ph = rocket.plotvis_sub(T, X_sub, U_sub, sys_z, xs, us);

%% X Control
% Design MPC controller
mpc_x = MpcControl_x(sys_x, Ts, H);

% Evaluate once and plot optimal open−loop trajectory,
% pad last input to get consistent size with time and state
x_x = [0, 0, 0, 3]';
[u, T_opt, X_opt, U_opt] = mpc_x.get_u(x_x);
U_opt(:,end+1) = NaN;
% Account for linearization point
X_opt = X_opt + [xs(2); xs(5); xs(7); xs(10)];
U_opt = U_opt + us(2);

[T, X_sub, U_sub] = rocket.simulate_f(sys_x, x_x, 2, @mpc_x.get_u, 0);
ph = rocket.plotvis_sub(T, X_sub, U_sub, sys_x, xs, us);

%% Y Control
% Design MPC controller
mpc_y = MpcControl_y(sys_y, Ts, H);

% Evaluate once and plot optimal open−loop trajectory,
% pad last input to get consistent size with time and state
x_y = [0, 0, 0, 3]';
[u, T_opt, X_opt, U_opt] = mpc_y.get_u(x_y);
U_opt(:,end+1) = NaN;
% Account for linearization point
X_opt = X_opt + [xs(1); xs(4); xs(8); xs(11)];
U_opt = U_opt + us(1);

[T, X_sub, U_sub] = rocket.simulate_f(sys_y, x_y, sTime, @mpc_y.get_u, 0);
ph = rocket.plotvis_sub(T, X_sub, U_sub, sys_y, xs, us);

%% Roll Control
mpc_roll = MpcControl_roll(sys_roll, Ts, H);

x_roll = [0, deg2rad(40)]';
[u, T_opt, X_opt, U_opt] = mpc_roll.get_u(x_roll);
U_opt(:,end+1) = NaN;
% Account for linearization point
X_opt = X_opt + [xs(3); xs(6)];
U_opt = U_opt + us(4);

[T, X_sub, U_sub] = rocket.simulate_f(sys_roll, x_roll, sTime, @mpc_roll.get_u, 0);
ph = rocket.plotvis_sub(T, X_sub, U_sub, sys_roll, xs, us);


%% TODO: This file should produce all the plots for the deliverable

% invariant set plots
% X control on state [omega_y, beta, vx, x] with d2 as input
%constraints {x | Fx <= f} and {u | Mu <= m}
F = [0,1,0,0;0,-1,0,0]; f = [0.1745;0.1745];
M = [1;-1]; m = [0.26;0.26];
Xpoly = Polyhedron(F,f);
Q = 10*eye(4);
R = eye(1);

sys = LTISystem('A', sys_x.A, 'B', sys_x.B);
sys.x.min(2) = -0.1745;
sys.x.max(2) = 0.1745;
sys.x.penalty = QuadFunction(Q);
sys.u.penalty = QuadFunction(R);
Qf = sys.LQRPenalty.weight;
Xf = sys.LQRSet;

figure
hold on; grid on;
Xpoly.projection(1:2).plot('alpha',0.1);
Xf.projection(1:2).plot('alpha',0.3);

% Y control on state [omega_x, alpha, vy, y] 

% Z control on state [vz, z] 

% Roll control on state [omega_z, zeta] 
