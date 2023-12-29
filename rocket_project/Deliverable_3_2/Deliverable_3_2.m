addpath(fullfile('..', 'src'));

close all;
clear;
clc;

Ts = 1/20; % Sample time
rocket = Rocket(Ts);
[xs, us] = rocket.trim();
sys = rocket.linearize(xs, us);
[sys_x, sys_y, sys_z, sys_roll] = rocket.decompose(sys, xs, us);

%% TODO: This file should produce all the plots for the deliverable

u_x = mpc_x.get_u([0,0,0,1]', )
