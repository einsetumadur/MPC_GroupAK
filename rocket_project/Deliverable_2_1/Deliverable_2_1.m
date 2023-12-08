addpath(fullfile('..', 'src'));

close all;
clear;
clc;


Ts = 1/20;
rocket = Rocket(Ts);

% Compute steady−state for which 0 = f(xs, us)
[xs, us] = rocket.trim();

% Linearize the nonlinear model about trim point
sys = rocket.linearize(xs, us)

%% TODO: This file should produce all the plots for the deliverable