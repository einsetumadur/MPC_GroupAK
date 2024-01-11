classdef MpcControl_x < MpcControlBase
    
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
            
            N_segs = ceil(H/Ts); % Horizon steps
            N = N_segs + 1;      % Last index in 1-based Matlab indexing

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
            
            % omega_y = X(1, :);
            beta = X(2, :);
            % v_x = X(3, :);
            % x = X(4, :);

            %soft constraints on beta
            E = sdpvar(1, N);
            S = 1000; % quadratic
            s = 100; % linear - exact 
            
            Q = 1*eye(nx);
            R = eye(nu);
            
            sys = LTISystem('A', mpc.A, 'B', mpc.B);
            sys.x.min(2) = -0.1745;
            sys.x.max(2) = 0.1745;
            sys.u.min = -0.26;
            sys.u.max = 0.26;
            sys.x.penalty = QuadFunction(Q);
            sys.u.penalty = QuadFunction(R);
            Qf = sys.LQRPenalty.weight;
            Xf = sys.LQRSet;

            con = (beta + E >= -0.1745) + (beta - E <= 0.1745);
            con = con + (E >= 0);
            con = con + (U >= -0.26) + (U <= 0.26);
            obj = 0;
            for i = 1:N-1
                con = con + (X(:,i+1) == mpc.A*X(:,i) + mpc.B*U(:,i));
                obj = obj + (X(:,i)-x_ref)'*Q*(X(:,i)-x_ref);
                obj = obj + (U(:,i)-u_ref)'*R*(U(:,i)-u_ref);
                obj = obj + (E(i)*S*E(i)) + s*abs(E(i));
            end
            con = con + (Xf.A*(X(:,N)-x_ref) <= Xf.b);
            obj = obj + (E(N)*S*E(N)) + s*abs(E(N));
            obj = obj + (X(:,N)-x_ref)'*Qf*(X(:,N)-x_ref);

            
            % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
            % Return YALMIP optimizer object
            ctrl_opti = optimizer(con, obj, sdpsettings('solver','gurobi'), {X(:,1), x_ref, u_ref}, {U(:,1), X, U});
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
            
            obj = us'*us;
            con = (mpc.A*xs + mpc.B*us == xs);
            con = con + (mpc.C*xs + mpc.D*us == ref);
            beta = xs(2, :);
            con = con + (beta >= -0.1745) + (beta <= 0.1745);
            con = con + (us >= -0.26) + (us <= 0.26);
            
            % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
            % Compute the steady-state target
            target_opti = optimizer(con, obj, sdpsettings('solver', 'gurobi'), ref, {xs, us});
        end
    end
end
