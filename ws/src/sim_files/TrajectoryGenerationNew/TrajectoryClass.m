classdef TrajectoryClass < handle
    properties
        coeffs_px;
        coeffs_py;
        coeffs_pz;
        order;
        Tfinal;
        t_v;
        Tr = struct('R', [], 't', []);
    end

    methods
        function obj = TrajectoryClass(order)
            obj.order = order;
            obj.Tr.R = eye(3);
            obj.Tr.t = zeros(3,1); 
        end
        
        function obj = generate(obj, s, e, t)
            % Solve the interpolation problem between 2 points:
            % Start point 's'
            % End point 'e'
            % Time span 't'
            obj.Tfinal = t;
            
            N = length(s);
            
            index_x = [1: 3: N];
            Nx = length(index_x);
            index_y = [2: 3: N];
            Ny = length(index_y);
            index_z = [3: 3: N];
            Nz = length(index_z);
            
            %X = zeros(obj.order + 1, 2);
            %Y = zeros(obj.order + 1, 2);
            %Z = zeros(obj.order + 1, 2);
            
            X = zeros(Nx, 2);
            Y = zeros(Ny, 2);
            Z = zeros(Nz, 2);
            
            X(1: Nx, :) = [s(index_x), e(index_x)];
            Y(1: Ny, :) = [s(index_y), e(index_y)];
            Z(1: Nz, :) = [s(index_z), e(index_z)];

            [Ax, bx] = buildInterpolationProblem(X, obj.order, t * ones(1,1), false);
            obj.coeffs_px = Ax\bx;
            
            [Ay, by] = buildInterpolationProblem(Y, obj.order, t * ones(1,1), false);
            obj.coeffs_py = Ay\by;
            
            [Az, bz] = buildInterpolationProblem(Z, obj.order, t * ones(1,1), false);
            obj.coeffs_pz = Az\bz;
        end
        
        function obj = gen_tvector(obj, t)
            obj.t_v = t_vec(t, obj.order);
        end
        
        function out = trj_eval(obj, t, der)
            % Evaluate the 'der' derivative of the trajectory at time 't'
            % in the reference 'ref'.
            % The 'ref' can be 'World' or 'Obstacle'.
            % Note: The 0 derivative is just the trajectory.
            val = zeros(3, 1);
            
            if (t > obj.Tfinal)
                %disp('Evaluation is outside the time span!');
                %fprintf('End time span: %f | Requested: %f', obj.Tfinal, t);
                t = obj.Tfinal;
            end
            
            obj = gen_tvector(obj, t);
            
            val(1) = dot(obj.coeffs_px.', polyder(obj.t_v, der));
            val(2) = dot(obj.coeffs_py.', polyder(obj.t_v, der));
            val(3) = dot(obj.coeffs_pz.', polyder(obj.t_v, der));
            
            out = val;
        end

    end
end
