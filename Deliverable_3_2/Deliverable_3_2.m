clear; close all; clc;
addpath(fullfile('..', 'src'));

Ts = 1/20; % Sample time
rocket = Rocket(Ts);

[xs, us] = rocket.trim();   % Compute steady-state for which 0 = f(xs,us)
sys = rocket.linearize(xs, us); % Linearize the nonlinear model about trim point
[sys_x, sys_y, sys_z, sys_roll] = rocket.decompose(sys, xs, us); % Decompose system in 4 independent sub systems

%% Define MPC tracking controllers, simulate and plot results
% sys_x
H = 3; % Horizon length in seconds
mpc_x = MPC_Control_x(sys_x, Ts, H);
x0 = [0,0,0,0]';
x_ref = -5;
Tf = 15;
[T, X_sub, U_sub] = rocket.simulate(sys_x, x0, Tf, @mpc_x.get_u, x_ref);
ph_x = rocket.plotvis_sub(T, X_sub, U_sub, sys_x, xs, us, x_ref);

% sys_y
H = 3; % Horizon length in seconds
mpc_y = MPC_Control_y(sys_y, Ts, H);
x0 = [0,0,0,0]';
x_ref = -5;
Tf = 15;
[T, X_sub, U_sub] = rocket.simulate(sys_y, x0, Tf, @mpc_y.get_u, x_ref);
ph_y = rocket.plotvis_sub(T, X_sub, U_sub, sys_y, xs, us, x_ref);

% sys_z
H = 3; % Horizon length in seconds
mpc_z = MPC_Control_z(sys_z, Ts, H);
x0 = [0,0]';
x_ref = -5;
Tf = 15;
[T, X_sub, U_sub] = rocket.simulate(sys_z, x0, Tf, @mpc_z.get_u, x_ref);
ph_z = rocket.plotvis_sub(T, X_sub, U_sub, sys_z, xs, us, x_ref);

% sys_roll
H = 3;
mpc_roll = MPC_Control_roll(sys_roll, Ts, H);
x0 = [0,0]';
x_ref = deg2rad(45);
Tf = 15;
[T, X_sub, U_sub] = rocket.simulate(sys_roll, x0, Tf, @mpc_roll.get_u, x_ref);
ph_roll = rocket.plotvis_sub(T, X_sub, U_sub, sys_roll, xs, us, x_ref);
 

