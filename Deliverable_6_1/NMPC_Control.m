function opti_eval = NMPC_Control(rocket, H)

import casadi.*
opti = casadi.Opti(); % Optimization problem

N = ceil(H/rocket.Ts); % MPC horizon
nx = 12; % Number of states
nu = 4;  % Number of inputs

% Decision variables (symbolic)
X_sym = opti.variable(nx, N);     % state trajectory
U_sym = opti.variable(nu, N-1);   % control trajectory

% Parameters (symbolic)
x0_sym  = opti.parameter(nx, 1);  % initial state
ref_sym = opti.parameter(4, 1);   % target position

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE

% Initial conditions
opti.subject_to(X_sym(:,1) == x0_sym);

% Define cost matrices
Q = eye(nx);
R = eye(nu);

% sys_x :
Q(2,2) = 5/10;
Q(5,5) = 1/10;
Q(7,7) = 1/10;
Q(10,10) = 10/10;
R(2,2) = 1/10;

% sys_y :
Q(1,1) = 5/10;
Q(4,4) = 1/10;
Q(8,8) = 1/10;
Q(11,11) = 10/10; 
R(1,1) = 1/10;

% sys_z :
Q(9,9) = 1/250;
Q(12,12) = 250/250;
R(3,3) = 1/250;

%sys_roll :
Q(3,3) = 1/250;
Q(6,6) = 250/250;
R(4,4) = 1/250;

%%%%%%%% Compute terminal cost by linearizing the system %%%%%%%%

% Linearize, discretize and define LTI system 
[xs, us] = rocket.trim();  
sys_lin = rocket.linearize(xs, us);
sys_d = c2d(sys_lin, rocket.Ts);
sys = LTISystem('A', sys_d.A, 'B', sys_d.B);

% Cost function
sys.x.penalty = QuadFunction(Q); 
sys.u.penalty = QuadFunction(R);

% State constraints 
sys.x.min(5) = deg2rad(-85);
sys.x.max(5) = deg2rad(85);

% Input constraints 
sys.u.min = [deg2rad(-15), deg2rad(-15), 50, -20]';
sys.u.max = [deg2rad(15), deg2rad(15), 80, 20]';

% Extract LQR gain
Qf = sys.LQRPenalty.weight;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

obj = 0;
idx_ref = [10, 11, 12, 6];
idx = [1, 2, 3, 4, 5, 7, 8, 9];

%Integration of the non-linear system using Runge-Kutta 4
f_discrete = @(x,u) RK4(rocket,x,u);

% Dynamic constraints
opti.subject_to(X_sym(:, 2) == f_discrete(X_sym(:,1), U_sym(:,1)));

% Input constraints
opti.subject_to(deg2rad(-15) <= U_sym(1,1) <= deg2rad(15));
opti.subject_to(deg2rad(-15) <= U_sym(2,1) <= deg2rad(15));
opti.subject_to(50 <= U_sym(3,1) <= 80);
opti.subject_to(-20 <= U_sym(4,1) <= 20);

% Cost function
obj = obj + (X_sym(idx_ref,1)-ref_sym)'*Q(idx_ref,idx_ref)*(X_sym(idx_ref,1)-ref_sym) + (U_sym(:,1)-us)'*R*(U_sym(:,1)-us);
obj = obj + X_sym(idx,1)'*Q(idx,idx)*X_sym(idx,1);

for i = 2:N-1
    % Dynamic constraints
    opti.subject_to(X_sym(:, i+1) == f_discrete(X_sym(:,i), U_sym(:,i)));
    
    % State constraints
    opti.subject_to(deg2rad(-85) <= X_sym(5,i) <= deg2rad(85)); 
    
    % Input constraints
    opti.subject_to(deg2rad(-15) <= U_sym(1,i) <= deg2rad(15));
    opti.subject_to(deg2rad(-15) <= U_sym(2,i) <= deg2rad(15));
    opti.subject_to(50 <= U_sym(3,i) <= 80);
    opti.subject_to(-20 <= U_sym(4,i) <= 20);
    
    % Cost function
    obj = obj + (X_sym(idx_ref,i)-ref_sym)'*Q(idx_ref,idx_ref)*(X_sym(idx_ref,i)-ref_sym) + (U_sym(:,i)-us)'*R*(U_sym(:,i)-us);
    obj = obj + X_sym(idx,i)'*Q(idx,idx)*X_sym(idx,i);
end

% Terminal weight
obj = obj + (X_sym(idx_ref,N)-ref_sym)'*Qf(idx_ref,idx_ref)*(X_sym(idx_ref,N)-ref_sym);      
obj = obj + X_sym(idx,N)'*Qf(idx,idx)*X_sym(idx,N);

% Objective
opti.minimize(obj);

% YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% ---- Setup solver ------
ops = struct('ipopt', struct('print_level', 0, 'tol', 1e-3), 'print_time', false);
opti.solver('ipopt', ops);

% Create function to solve and evaluate opti
opti_eval = @(x0_, ref_) solve(x0_, ref_, opti, x0_sym, ref_sym, U_sym);
end

function u = solve(x0, ref, opti, x0_sym, ref_sym, U_sym)

% ---- Set the initial state and reference ----
opti.set_value(x0_sym, x0);
opti.set_value(ref_sym, ref);

% ---- Solve the optimization problem ----
sol = opti.solve();
assert(sol.stats.success == 1, 'Error computing optimal input');

u = opti.value(U_sym(:,1));

% Use the current solution to speed up the next optimization
opti.set_initial(sol.value_variables());
opti.set_initial(opti.lam_g, sol.value(opti.lam_g));
end
