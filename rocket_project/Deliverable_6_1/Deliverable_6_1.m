addpath(fullfile('..', 'src'));

close all;
clear;
clc;

Ts = 1/20;
rocket = Rocket(Ts);

H = 1.0; % Horizon length in seconds
nmpc = NmpcControl(rocket, H);

% MPC reference with default maximum roll = 15 deg
%ref = @(t_, x_) ref_TVC(t_);

% MPC reference with specified maximum roll = 50 deg
roll_max = deg2rad(50);
ref = @(t_, x_) ref_TVC(t_, roll_max);

x0 = zeros(12, 1);

% Evaluate once and plot optimal open−loop trajectory,
% pad last input to get consistent size with time and state
%x0(rocket.indx.pos(2)) = -2;
%ref = zeros(4, 1);
%ref(1) = 3;
%[u, T_opt, X_opt, U_opt] = nmpc.get_u(x0, ref);
%U_opt(:,end+1) = nan;
%rocket.plotvis(T_opt, X_opt, U_opt, ref);

% Closed-loop
Tf = 30;
[T, X, U, Ref] = rocket.simulate(x0, Tf, @nmpc.get_u, ref);
rocket.anim_rate = 5;
rocket.plotvis(T, X, U, Ref);

%% TODO: This file should produce all the plots for the deliverable