

%% TODO: This file should produce all the plots for the deliverable TODOs 1
% todo 1.1
Ts = 1/20;


% todo 1.2
rocket = Rocket(Ts);
Tf = 2.0; % Simulation end time
x0 = [deg2rad([0 0 0, 0 0 0]), 0 0 0, 0 0 0]'; % (w, phi, v, p) Initial state
u = [0, 0, 63, 30 ]'; % (d1 d2 Pavg Pdiff) Constant input
[T, X, U] = rocket.simulate(x0, Tf, u); % Simulate unknown, nonlinear model
rocket.anim_rate = 1.0; % Visualize at 1.0x real−time
rocket.vis(T, X, U);
%% todos 2
Ts = 1/20;
%todo 2.1
rocket = Rocket(Ts);

[xs, us] = rocket.trim();
sys = rocket.linearize(xs,us);
[sys_x, sys_y, sys_z, sys_roll] = rocket.decompose(sys, xs, us);

H=1;

mpc_x = MpcControl_x(sys_x, Ts, H);

x_x = [0, 0, 0, 3]';
[u, T_opt, X_opt, U_opt] = mpc_x.get_u(x_x);
U_opt(:,end+1) = NaN;
% Account for linearization point
X_opt = X_opt + [xs(2); xs(5); xs(7); xs(10)];
U_opt = U_opt + us(2);

[T, X_sub, U_sub] = rocket.simulate_f(sys_x, x_x, 5, @mpc_x.get_u, 0);
ph = rocket.plotvis_sub(T, X_sub, U_sub, sys_x, xs, us);
