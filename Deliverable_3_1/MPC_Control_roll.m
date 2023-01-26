classdef MPC_Control_roll < MPC_Control
    
    methods
        % Design a YALMIP optimizer object that takes a steady-state state
        % and input (xs, us) and returns a control input
        function ctrl_opti = setup_controller(mpc, Ts, H)
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % INPUTS
            %   X(:,1)       - initial state (estimate)
            %   x_ref, u_ref - reference state/input
            % OUTPUTS
            %   U(:,1)       - input to apply to the system
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
            N = ceil(H/Ts); % Horizon steps
            
            [nx, nu] = size(mpc.B);
            
            % Steady-state targets (Ignore this before Todo 3.2)
            x_ref = sdpvar(nx, 1);
            u_ref = sdpvar(nu, 1);
            
            % Predicted state and input trajectories
            X = sdpvar(nx, N);
            U = sdpvar(nu, N-1);
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE
            
            % NOTE: The matrices mpc.A, mpc.B, mpc.C and mpc.D are
            %       the DISCRETE-TIME MODEL of your system
            
            % SET THE PROBLEM CONSTRAINTS con AND THE OBJECTIVE obj HERE
            
            % Define cost matrices
            Q = eye(nx);
            Q(2,2) = 100;
            R = eye(nu);

            % Define system dynamics
            sys = LTISystem('A', mpc.A, 'B', mpc.B);

            % Cost function
            sys.x.penalty = QuadFunction(Q); 
            sys.u.penalty = QuadFunction(R);
            
            % Input constraints such that : u in U = { u | Mu <= m }
            M = [1 ;-1] ; m = [20 ; 20];
            sys.u.min = -20;
            sys.u.max = 20;

            % Extract LQR gain and terminal set
            Qf = sys.LQRPenalty.weight;
            Xf = sys.LQRSet;
            
            % Plot terminal set
            figure()
            plot(Xf)
            xlabel("$\omega_z$ [rad/s]", "interpreter", "latex")
            ylabel("$\gamma$ [m]", "interpreter", "latex")
            title("Terminal invariant set $X_f$ of $sys_{roll}$", "interpreter","latex")
             
            % Defining the MPC controller
            con = [];
            obj = 0;
            
            con = [con, X(:,2) == mpc.A*X(:,1) + mpc.B*U(:,1)];     % System dynamics                                                           
            con = [con, M*U(:,1) <= m];                             % Input constraints
            obj = obj + U(:,1)'*R*U(:,1);                           % Cost function

            for i = 2:N-1
                con = [con, X(:,i+1) == mpc.A*X(:,i) + mpc.B*U(:,i)];       % System dynamics
                con = [con, M*U(:,i) <= m];                                 % Input constraints
                obj = obj + X(:,i)'*Q*X(:,i) + U(:,i)'*R*U(:,i);            % Cost function
            end
            con = [con, Xf.A*X(:,N) <= Xf.b];   % Terminal constraint
            obj = obj + X(:,N)'*Qf*X(:,N);      % Terminal weight
            
            % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
            % Return YALMIP optimizer object
            ctrl_opti = optimizer(con, obj, sdpsettings('solver','gurobi'), ...
                {X(:,1), x_ref, u_ref}, U(:,1));
        end
        
        % Design a YALMIP optimizer object that takes a position reference
        % and returns a feasible steady-state state and input (xs, us)
        function target_opti = setup_steady_state_target(mpc)
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % INPUTS
            %   ref    - reference to track
            % OUTPUTS
            %   xs, us - steady-state target
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
            % Steady-state targets
            nx = size(mpc.A, 1);
            xs = sdpvar(nx, 1);
            us = sdpvar;
            
            % Reference position (Ignore this before Todo 3.2)
            ref = sdpvar;
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE
            % You can use the matrices mpc.A, mpc.B, mpc.C and mpc.D
            obj = 0;
            con = [xs == 0, us == 0];
            
            % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
            % Compute the steady-state target
            target_opti = optimizer(con, obj, sdpsettings('solver', 'gurobi'), ref, {xs, us});
        end
    end
end
