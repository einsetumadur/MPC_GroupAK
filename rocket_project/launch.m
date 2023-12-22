clear all
clc 
%casadi
addpath('C:\Users\gille\Documents\casadi-3.6.4-windows64-matlab2018b')
addpath('src')

%U
d1 = 0.0;
d2 = 0.12;
Pavg = 62.5;
Pdiff = 0;
u = [d1,d2,Pavg,Pdiff]';
%X
w = [0,0,0];
phi = [0,0,0];
v = [0,0,0];
p = [0,0,0];

Ts = 1/20;
rocket = Rocket(Ts);

%% todo 1.2

Tf = 2.0; % Simulation end time
x0 = [deg2rad([0 0 0, 0 0 0]), 0 0 0, 0 0 0]'; % (w, phi, v, p) Initial state
u = [u,repmat([0,-0.005,62.5,0]',1,40)];
[T, X, U] = rocket.simulate(x0, Tf, u); % Simulate unknown, nonlinear model
rocket.anim_rate = 1.0; % Visualize at 1.0x real−time
rocket.vis(T, X, U);

%% todo 2.1

[xs,us] = rocket.trim(); %steady state (f(xs,us) = 0)
sys = rocket.linearize(xs,us); %system around steady state

%% todo 2.2 

[sys_x,sys_y,sys_z,sys_roll] = rocket.decompose(sys,xs,us);

%% todo 3.1

sys_d = c2d(sys,Ts);

Q = 10 * eye(nx);
R = 1;

% Constraints
% u in U = { u | Mu <= m }
M = [eye(nu);-eye(nu)];
m = [0.26;0.26;80;20;0.26;0.26;-50;20];
% x in X = { x | Fx <= f }
F = [0 0 0 1 0 0 0 0 0 0 0 0;...
     0 0 0 0 1 0 0 0 0 0 0 0;...
     0 0 0 0 0 1 0 0 0 0 0 0;...
     0 0 0 -1 0 0 0 0 0 0 0 0;...
     0 0 0 0 -1 0 0 0 0 0 0 0;...
     0 0 0 0 0 -1 0 0 0 0 0 0]; 
f = [0.1745;0.1745;3.14;0.1745;0.1745;3.14];

con = (X(:,2) == A*X(:,1) + B*U(:,1)) + (M*U(:,1) <= m);
obj = U(:,1)'*R*U(:,1);
for i = 1:N-1
    con = [con, X(:,i+1) == mpc.A*X(:,i) + mpc.B*X(:,i)]; % System dynamics
    con = [con, F*X(:,i) <= f]; % State constraints
    con = [con, M*U(:,i) <= m]; % Input constraints
    obj = obj + X(:,i)'*Q*X(:,i) + U(:,i)'*R*U(:,i); % Cost function
end
con = [con, Ff*X(:,N) <= ff]; % Terminal constraint
obj = obj + X(:,N)'*Qf*X(:,N); % Terminal weight


% YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Return YALMIP optimizer object
ctrl_opti = optimizer(con, obj, sdpsettings('solver','gurobi'), ...
    {X(:,1), x_ref, u_ref, d_est}, {U(:,1), X, U});









