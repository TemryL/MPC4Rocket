clear; close all; clc;
addpath(fullfile('..', 'src'));

Ts = 1/10; % Note that we choose a larger Ts here to speed up the simulation
rocket = Rocket(Ts);

H = 1;
nmpc = NMPC_Control(rocket, H);

%% Simulate the system with NMPC under maximum roll of 15 deg
% MPC reference with default maximum roll = 15 deg
Tf = 30;
ref = @(t, x) rocket.MPC_ref(t , Tf);

x0 = zeros(12,1);
[T, X, U, Ref] = rocket.simulate_f(x0, Tf, nmpc, ref);

% Plot pose
rocket.anim_rate = 1; % Increase this to make the animation faster
ph_1 = rocket.plotvis(T, X, U, Ref);
ph_1.fig.Name = 'NMPC in nonlinear simulation under max roll angle of 15 deg'; 

%% Simulate the system with NMPC under maximum roll of 50 deg
% MPC reference with specified maximum roll = 50 deg
Tf = 30;
roll_max = deg2rad(50);
ref = @(t, x) rocket.MPC_ref(t, Tf, roll_max);

x0 = zeros(12,1);
[T, X, U, Ref] = rocket.simulate_f(x0, Tf, nmpc, ref);

% Plot pose
rocket.anim_rate = 1; % Increase this to make the animation faster
ph_2 = rocket.plotvis(T, X, U, Ref);
ph_2.fig.Name = 'NMPC in nonlinear simulation under max roll angle of 50 deg';

%% Simulate the system with MPC under maximum roll of 50 deg
clear
Ts = 1/20; % Sample time
rocket = Rocket(Ts);

[xs, us] = rocket.trim();   % Compute steady-state for which 0 = f(xs,us)
sys = rocket.linearize(xs, us); % Linearize the nonlinear model about trim point
[sys_x, sys_y, sys_z, sys_roll] = rocket.decompose(sys, xs, us); % Decompose system in 4 independent sub systems

% Define MPC tracking controllers
H = 3; % Horizon length in seconds
mpc_x = MPC_Control_x(sys_x, Ts, H);
mpc_y = MPC_Control_y(sys_y, Ts, H);
mpc_z = MPC_Control_z(sys_z, Ts, H);
mpc_roll = MPC_Control_roll(sys_roll, Ts, H);

% Merge four sub-system controllers into one full-system controller
mpc = rocket.merge_lin_controllers(xs, us, mpc_x, mpc_y, mpc_z, mpc_roll);

% MPC reference with specified maximum roll = 50 deg
Tf = 30;
roll_max = deg2rad(50);
ref = @(t, x) rocket.MPC_ref(t, Tf, roll_max);

x0 = zeros(12,1);
[T, X, U, Ref] = rocket.simulate_f(x0, Tf, mpc, ref);

% Plot pose
rocket.anim_rate = 1; % Increase this to make the animation faster
ph_3 = rocket.plotvis(T, X, U, Ref);
ph_3.fig.Name = 'Merged lin. MPC in nonlinear simulation under max roll angle of 50 deg'; 
