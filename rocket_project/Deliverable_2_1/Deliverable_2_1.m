addpath(fullfile('..', 'src'));

close all;
clear;
clc;


Ts = 1/20;
rocket = Rocket(Ts);

% Compute steady−state for which 0 = f(xs, us)
[xs, us] = rocket.trim()

% Linearize the nonlinear model about trim point
sys = rocket.linearize(xs, us);

%% TODO: This file should produce all the plots for the deliverable
xref = [0,0,0,0,0,0,0,0,0,2,2,2]';

sys_d = c2d(sys,Ts);
sys_d.UserData.xs = xs;
sys_d.UserData.us = us;
sys_d.UserData.idu = [];
[T, X, U,a,b] = rocket.simulate_f(sys_d, xs, 10.0,@control,xref);
rocket.anim_rate = 1.0; % Visualize at 1.0x real−time
rocket.vis(T, X, U);

function [uc] = control(x_,ref_)
uc = [0,0,0,0]';
    if nargin() == 0
       uc = [0,0,0,0]'; 
    else
        ex = ref_ - x_;
        uc(1) = ex(11)*0.002 - ex(4)*0.1;
        uc(2) = -ex(10)*0.002 - ex(5)*0.1; 
        uc(3) = ex(12)*0.3;
        uc(4) = 0;
    end
end