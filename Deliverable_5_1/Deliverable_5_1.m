clear; close all; clc;
addpath(fullfile('..', 'src'));

Ts = 1/20; % Sample time
rocket = Rocket(Ts);

[xs, us] = rocket.trim();   % Compute steady-state for which 0 = f(xs,us)
sys = rocket.linearize(xs, us); % Linearize the nonlinear model about trim point
[sys_x, sys_y, sys_z, sys_roll] = rocket.decompose(sys, xs, us); % Decompose system in 4 independent sub systems

%% Define MPC offset-free tracking controllers
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

%% Change rocket mass and simulate without disturbance rejection
rocket.mass = 1.9; % Manipulate mass for simulation
[T_new_mass, X_new_mass, U_new_mass, ~] = rocket.simulate_f(x0, Tf, mpc, ref);

%% Change rocket mass and apply disturbance rejection
rocket.mass = 1.9; % Manipulate mass for simulation
[T_est, X_est, U_est, ~, Z_hat] = rocket.simulate_f_est_z(x0, Tf, mpc, ref, mpc_z, sys_z);

%% Plot results without disturbance rejection 
X_z = X([9,12],:);
U_z = U(3,:);
Ref_z = Ref(3,:);
plot_sys_z(rocket, T, X_z, U_z, sys_z, Ref_z);

hold on 
subplot(3,1,1)
plot(T_new_mass, U_new_mass(3,:),'LineWidth', 1, 'Color','#EDB120')
subplot(3,1,3)
plot(T_new_mass, X_new_mass(12,:),'LineWidth', 1, 'Color','#EDB120')
subplot(3,1,2)
plot(T_new_mass, X_new_mass(9,:),'LineWidth', 1, 'Color','#EDB120')
hold off

f=get(gca,'Children');
legend([f(3),f(1)],'Rocket mass = 1.7 [kg] (default)','Rocket mass = 1.9 [kg] (without disturbance rejection)','Location','southeast')

%% Plot results with disturbance rejection 
plot_sys_z(rocket, T, X_z, U_z, sys_z, Ref_z);

hold on 
subplot(3,1,1)
plot(T_est, U_est(3,:),'LineWidth', 1, 'Color','#77AC30')
subplot(3,1,3)
plot(T_est, Z_hat(12,:),'LineWidth', 1, 'Color','#77AC30')
subplot(3,1,2)
plot(T_est, Z_hat(9,:),'LineWidth', 1, 'Color','#77AC30')
hold off

f=get(gca,'Children');
legend([f(3),f(1)],'Rocket mass = 1.7 [kg] (default)','Rocket mass = 1.9 [kg] (with disturbance rejection)')