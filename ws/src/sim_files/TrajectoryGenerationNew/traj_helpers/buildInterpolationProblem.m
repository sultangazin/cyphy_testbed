function [A, b] = buildInterpolationProblem(X, deg, T, is_abstime)
    %{
    Build the interpolation problem A * x = b

    Args
        X:   Matrix of the constraints Nconstr x Nwaypoints
        deg; Degree of the polynomial to fit to describe the pieces
        T:  Vector with the time information (Either the waypoint pass time
                                              or the duration for each piece)
        abstime: Flag to specify whether the t vector represents
                    durations or absolute time


    Output:
        A: Matrix of the system
        b: Vector of the known terms
    %}

    nCoef = deg + 1;         % Number of coefficient to describe each polynomial
    nConstr = size(X, 1);    % Number of constraints (flat outputs)
    nWp = size(X, 2);        % Number of waypoints
    nPl = nWp - 1;           % Number of polynomials

    nEq = numOfEquations(X); % Number of equation of the interpolation problem

    % Length of the overall vector containing the coefficients
    % of the polynomials, that is, (nCoef * NumberOfPieces)
    nC = (deg + 1) * nPl;

    % Instantiate the output variables
    A = zeros(nEq, nC);
    b = zeros(nEq, 1);

    % Define the time vector
    Dt = zeros(nPl, 1);

    if (is_abstime)
        for i = (1 : length(T) - 1)
            Dt(i) = T(i+1) - T(i);
        end
    else
        Dt = T;
    end

    counter = 1;
    for i = (1 : nWp)           % For each waypoint
        for j = (1 : nConstr)        % For each constraint
            % If not (First or Last)
            if (i > 1 && i < nWp)
                % If specified wp
                if (~isnan(X(j,i)))
                    % Waypoint "i" is connects Polynomial i-1 and Polynomial i
                    b(counter) = X(j,i);

                    v = t_vec(Dt(i-1), deg);
                    A(counter, selIndex(i-1, nCoef)) = polyder(v, j - 1);
                    counter = counter + 1;

                    b(counter) = X(j,i);
                    v = t_vec(0, deg);
                    A(counter, selIndex(i, nCoef)) = polyder(v, j - 1);
                % Only continuity constraint
                else
                    b(counter) = 0;

                    v = t_vec(Dt(i-1), deg);
                    A(counter, selIndex(i-1, nCoef)) = polyder(v, j - 1);

                    v = t_vec(0, deg);
                    A(counter, selIndex(i, nCoef)) = -1 * polyder(v, j - 1);
                end
            % If First of Last
            else
                if (i == 1)
                    b(counter) = X(j,i);
                    v = t_vec(0, deg);
                    sub_indexes = selIndex(i, nCoef);
                    pder = polyder(v, j - 1);
                    A(counter, sub_indexes) = pder;
                else
                    b(counter) = X(j,i);
                    v = t_vec(Dt(i-1), deg);
                    sub_indexes = selIndex(i-1, nCoef);
                    pder = polyder(v, j - 1);
                    A(counter, sub_indexes) = pder;
                end
            end
            counter = counter + 1;
        end
    end
end
                
