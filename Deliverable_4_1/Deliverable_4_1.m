clear; close all; clc;
addpath(fullfile('..', 'src'));

Ts = 1/20; % Sample time
rocket = Rocket(Ts);

[xs, us] = rocket.trim();   % Compute steady-state for which 0 = f(xs,us)
sys = rocket.linearize(xs, us); % Linearize the nonlinear model about trim point
[sys_x, sys_y, sys_z, sys_roll] = rocket.decompose(sys, xs, us); % Decompose system in 4 independent sub systems

%% Define MPC tracking controllers
H = 3; % Horizon length in seconds
mpc_x = MPC_Control_x(sys_x, Ts, H);

H = 3; % Horizon length in seconds
mpc_y = MPC_Control_y(sys_y, Ts, H);

H = 3; % Horizon length in seconds
mpc_z = MPC_Control_z(sys_z, Ts, H);

H = 3; % Horizon length in seconds
mpc_roll = MPC_Control_roll(sys_roll, Ts, H);

% Merge four sub-system controllers into one full-system controller
mpc = rocket.merge_lin_controllers(xs, us, mpc_x, mpc_y, mpc_z, mpc_roll);

%% Setup reference function and simulate the system
Tf = 30;
ref = @(t, x) rocket.MPC_ref(t, Tf);

x0 = zeros(12,1);
[T, X, U, Ref] = rocket.simulate_f(x0, Tf, mpc, ref);

%% Plot pose
rocket.anim_rate = 1; % Increase this to make the animation faster
ph = rocket.plotvis(T, X, U, Ref);
ph.fig.Name = 'Merged lin. MPC in nonlinear simulation'; 