classdef MPC_Control_x < MPC_Control
    
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
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
            N = ceil(H/Ts); % Horizon steps

            [nx, nu] = size(mpc.B);
            
            % Targets (Ignore this before Todo 3.2)
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
            R = eye(nu);

            % Define system dynamics
            sys = LTISystem('A', mpc.A, 'B', mpc.B);

            % Cost function
            sys.x.penalty = QuadFunction(Q); 
            sys.u.penalty = QuadFunction(R);

            % State constraints such that : x in X = { x | Fx <= f }
            F = [0 1 0 0 ; 0 -1 0 0]; f = [deg2rad(5) ; deg2rad(5)];
            sys.x.min(2) = -deg2rad(5);
            sys.x.max(2) = deg2rad(5);
            
            % Input constraints such that : u in U = { u | Mu <= m }
            M = [1 ;-1] ; m = [deg2rad(15) ; deg2rad(15)];
            sys.u.min = -deg2rad(15);
            sys.u.max = deg2rad(15);

            % Extract LQR gain 
            Qf = sys.LQRPenalty.weight;
            
            % Defining the MPC controller
            con = [];
            obj = 0;
            
            con = [con, X(:,2) == mpc.A*X(:,1) + mpc.B*U(:,1)];     % System dynamics                                                            
            con = [con, M*U(:,1) <= m];                             % Input constraints
            obj = obj + (U(:,1)-u_ref)'*R*(U(:,1)-u_ref);           % Cost function

            for i = 2:N-1
                con = [con, X(:,i+1) == mpc.A*X(:,i) + mpc.B*U(:,i)];                                       % System dynamics
                con = [con, F*X(:,i) <= f];                                                                 % State constraints
                con = [con, M*U(:,i) <= m];                                                                 % Input constraints
                obj = obj + (X(:,i)-x_ref)'*Q*(X(:,i)-x_ref) + (U(:,i)-u_ref)'*R*(U(:,i)-u_ref);            % Cost function
            end
            obj = obj + (X(:,N)-x_ref)'*Qf*(X(:,N)-x_ref);      % Terminal weight

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
            
            nx = size(mpc.A, 1);

            % Steady-state targets
            xs = sdpvar(nx, 1);
            us = sdpvar;
            
            % Reference position (Ignore this before Todo 3.2)
            ref = sdpvar;
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE
            % You can use the matrices mpc.A, mpc.B, mpc.C and mpc.D
            
            % State constraints such that : x in X = { x | Fx <= f }
            F = [0 1 0 0 ; 0 -1 0 0] ; f = [deg2rad(5) ; deg2rad(5)];
            
            % Input constraints such that : u in U = { u | Mu <= m }
            M = [1 ;-1] ; m = [deg2rad(15) ; deg2rad(15)];
            
            con = [xs == mpc.A*xs + mpc.B*us , ref == mpc.C*xs + mpc.D*us ];    % System dynamics
            con = [con, F*xs <= f];                                             % State constraints
            con = [con, M*us <= m];                                             % Input constraints
            
            obj = us^2;                                                         % Cost function

            % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
            % Compute the steady-state target
            target_opti = optimizer(con, obj, sdpsettings('solver', 'gurobi'), ref, {xs, us});
        end
    end
end
