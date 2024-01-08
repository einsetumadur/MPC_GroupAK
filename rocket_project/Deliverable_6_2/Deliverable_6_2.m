addpath(fullfile('..', 'src'));

close all;
clear;
clc;

Ts = 1/40;
rocket = Rocket(Ts);

H = 0.5;
expected_delay = 10;
nmpc = NmpcControl(rocket, H, expected_delay);

x0 = zeros(12, 1);
ref = [0.5, 0, 1, deg2rad(65)]';
Tf = 2.5;

rocket.mass = 1.75;
rocket.delay = 10;
[T, X, U, Ref] = rocket.simulate(x0, Tf, @nmpc.get_u, ref);

rocket.anim_rate = 1.0;
rocket.plotvis(T, X, U, Ref);

%% TODO: This file should produce all the plots for the deliverable