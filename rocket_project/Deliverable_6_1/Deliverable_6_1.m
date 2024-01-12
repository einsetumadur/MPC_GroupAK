addpath(fullfile('..', 'src'));

close all;
clear;
clc;

Ts = 1/20;
rocket = Rocket(Ts);
x0 = zeros(12, 1);

% Open-loop
if false
    % Evaluate once and plot optimal open−loop trajectory,
    % pad last input to get consistent size with time and state
    H = 5.0; % For the open-loop plot, use a longer horizon to visualize
    % convergence to the reference
    nmpc = NmpcControl(rocket, H);
    ref = [0.5, 0, 1, deg2rad(50)]';
    [u, T_opt, X_opt, U_opt] = nmpc.get_u(x0, ref);
    U_opt(:,end+1) = nan;
    rocket.anim_rate = 5;
    rocket.plotvis(T_opt, X_opt, U_opt, ref);
end

H = 2.0;
nmpc = NmpcControl(rocket, H);
Tf = 30;

% Closed-loop with default maximum roll = 15 deg
if false
    ref = @(t_, x_) ref_TVC(t_);
    [T, X, U, Ref] = rocket.simulate(x0, Tf, @nmpc.get_u, ref);
    rocket.anim_rate = 5;
    rocket.plotvis(T, X, U, Ref);
end

% Closed-loop with specified maximum roll = 50 deg
if true
    roll_max = deg2rad(50);
    ref = @(t_, x_) ref_TVC(t_, roll_max);
    [T, X, U, Ref] = rocket.simulate(x0, Tf, @nmpc.get_u, ref);
    rocket.anim_rate = 5;
    rocket.plotvis(T, X, U, Ref);
end

% Closed-loop linear controller with specified maximum roll = 50 deg
if false
    roll_max = deg2rad(50);
    ref = @(t_, x_) ref_TVC(t_, roll_max);
    [xs, us] = rocket.trim();
    sys = rocket.linearize(xs, us);
    [sys_x, sys_y, sys_z, sys_roll] = rocket.decompose(sys, xs, us);
    mpc_x = MpcControl_x(sys_x, Ts, H);
    mpc_y = MpcControl_y(sys_y, Ts, H);
    mpc_z = MpcControl_z(sys_z, Ts, H);
    mpc_roll = MpcControl_roll(sys_roll, Ts, H);
    mpc = rocket.merge_lin_controllers(xs, us, mpc_x, mpc_y, mpc_z, mpc_roll);
    [T, X, U, Ref] = rocket.simulate(x0, Tf, @mpc.get_u, ref);
    rocket.anim_rate = 5;
    rocket.plotvis(T, X, U, Ref);
end
