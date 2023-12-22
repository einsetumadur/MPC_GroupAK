addpath(fullfile('..', 'src'));

close all;
clear;
clc;


Ts = 1/20;
rocket = Rocket(Ts);

% Compute steady−state for which 0 = f(xs, us)
[xs, us] = rocket.trim();

% Linearize the nonlinear model about trim point
sys = rocket.linearize(xs, us);

Tf = 2.0; % Simulation end time
x0 = [deg2rad([2 -2 0, -2 2 0]), 0 0 0, 0 0 0]'; % (w, phi, v, p) Initial state
u = [deg2rad([2 0]), 60, 0 ]'; % (d1 d2 Pavg Pdiff) Constant input
[T, X, U] = rocket.simulate(x0, Tf, u); % Simulate unknown, nonlinear model

rocket.anim_rate = 1.0; % Visualize at 1.0x real−time
rocket.vis(T, X, U);

%% TODO: This file should produce all the plots for the deliverable