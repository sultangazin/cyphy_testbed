function nCr = numOfEquations(W)
    %{
    Count the number of equation necessary to set up the interpolation problem
    for trajectory generation.

    The number of equations depends on the number and type of constraints.
    There are "specific constraints" where the polynomials are required to
    pass through given points of the flat output trajectory and
    "Smoothness constraints", which simply ask to have a smooth connection
    between pieces of trajectory described by sequential polynomials.

    The number of equation is (WP0 + WPend) + 2 x (Fixed Cnstr) + (Smooth Cnstr)

    Input
        W: Matrix of the constraints (Nconstraints x Nwaypoints)

    Ouput
        nCr: Number of equations required to set up the interpolation problem
    %}

    nWr = size(W, 1);
    nWc = size(W, 2);

    % First and End condition (fixed points)
    nCr = 2 * nWr;

    % Junction points of the trajecotry
    % For each fixed point condition we will get 2 conditions, whilst, for
    % the smooth constraints we will obtain only 1 condition.

    % Select the waypoints, which are shared between polynomial
    Wshared = W(:, 2: nWc - 1);
    fWshared = Wshared(:);

    % Count how many fixed point condition and smooth condition I have
    nshared_fix = sum(~isnan(fWshared));
    nshared_smooth = sum(isnan(fWshared));
    nCr = nCr + 2 * nshared_fix + nshared_smooth;

end
