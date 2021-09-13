function A = constrMat(tBase, indConstrDer)
    %{
    Build the constraint matrix A for the interpolation problem (Ax = s)
    Takes as input the time polynomial and the indexes of the derivative,
    ehich constitutes the object to be constrained.

    Ex.
    tBase = [1 t0 t0^2 t0^3]
    indConstrDer = [0 1 2]

    Defines the problem:
    [1  t0 t0^2  t0^3 ]
    [0  1  2*t0 3*t0^2] * c = s0    ==> A * c = s0
    [0  0   2    6*t0 ]

    %}
    nconstr = lenght(indConstrDer);
    ncoeff = length(tBase);

    A = zeros(nconstr, ncoeff);

    for i = 1 : nconstr
        A(i, :) = polyder(tBase, indConstrDer(i)).';
    end
end