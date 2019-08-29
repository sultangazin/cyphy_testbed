"""
Support for trajectory generation

deg: degree of the polynomial used for describing the trajectory
der: number of derivatives of the flat output used to define the trajecotry

The trajectory will be generated modeling the problem as an optimization
problem.

The constraints will be in the form:
    c' * A = b,
where "c" is the vector of polynomial coefficients, "A" is the matrix mapping
coefficients with flat oututs and "b" is the vector of constraints in the
flat output space.

The conditions for the polynomial will be provided with a matrix X

Where X[i][j] is the requirement for the i-derivative of the j-th waypoint.

[1 t t^2 t^3 ...] * c1 = p1(t)
[1 t t^2 t^3 ...] * c2 = p2(t)

@author: l.pannocchi@gmail.com
"""
import numpy as np
import scipy.linalg as linalg
import csv

def null_space(A, atol=1e-13, rtol=0):
    A = np.atleast_2d(A)
    u, s, vh = linalg.svd(A)
    tol = max(atol, rtol * s[0])
    nnz = (s >= tol).sum()
    ns = vh[nnz:].conj().T
    return ns

## =================================================
## =================================================
def interpolPolys(X, deg, T, is_abstime):
    """
    Find the coefficients of the polynomials interpolating a given series
    of waypoints

    Args
        X:   Matrix of the constraints Nconstr x Nwaypoinst
        deg: Degree of the polynomial to fit to describe the pieces
        T: Knots points for the interpolation
        abstime: Flag to specify whether the t vector represents
                    durations or absolute time


    Output:
        sol:   Solution vector of the Ax = b problem (All coeff in a row)
        nullx: Null space of the A
        res:   Residuals of the interpolation problem
        polys: Matrix with the polynomial coefficients ([a_0 a_1 ...a_deg])
               for each piece of trajectory on the rows.
               The polynomial would be (a_0 + a_1*t + a_2*t^2 ...)

    """
    nCoeff = deg + 1

    nWp = X.shape[1]       # Number of waypoints

    [A, b] = buildInterpolationProblem(X, deg, T, is_abstime)

    nullx = null_space(A)
    
    if (nullx.size == 0):
        print("Null space is void")
    
    # Define the time vector
    Dt = np.zeros((nWp - 1), dtype=float)

    if (is_abstime):
        for i in range(len(T) - 1):
            Dt[i] = T[i+1] - T[i]
    else:
        Dt = T

    Q = genQ(Dt, deg, 2)

    # M1x = F^T * Q * F
    M1x = (np.matmul(np.matmul(nullx.transpose(), Q), nullx))
    # M2x = - M1x * F^T * Q
    M2x = -np.matmul(np.matmul(np.linalg.inv(M1x), nullx.transpose()), Q)

    # One solution 
    # I select the minimum norm solution to the problem
    inv_AA_T = np.linalg.inv(np.matmul(A, A.transpose()))
    pseudoInv =  np.matmul(A.transpose(), inv_AA_T) 
    sol_min_norm = np.matmul(pseudoInv, b)
    res = np.matmul(A, sol_min_norm)        
 
    vx = np.matmul(M2x, sol_min_norm)

    # Interpolate (with sol_min_norm) and minimize the snap 
    # with the free variables (vx)
    sol = sol_min_norm + np.matmul(nullx, vx)

    npolys = int(len(sol) / nCoeff)

    polys = np.zeros((npolys, nCoeff), dtype=float)
    for i in range(npolys):
        polys[i, :] = sol[i * nCoeff: (i + 1) * nCoeff]

    return (sol, nullx, res, polys)


## =================================================
## =================================================
def buildInterpolationProblem(X, deg, T, is_abstime = True):
    """
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
    """

    nCoef = deg + 1        # Number of coefficient to describe each polynomial
    nConstr = X.shape[0]   # Number of constraints (flat outputs)
    nWp = X.shape[1]       # Number of waypoints

    nEq = numOfEquations(X) # Number of equation of the interpolation problem

    # Length of the overall vector containing the coefficients
    # of the polynomials, that is, (nCoef * NumberOfPieces)
    nC = (deg + 1) * (nWp - 1)

    # Instantiate the output variables
    A = np.zeros((nEq, nC), dtype=float)
    b = np.zeros((nEq), dtype=float)

    # Define the time vector
    Dt = np.zeros((nWp - 1), dtype=float)

    if (is_abstime):
        for i in range(len(T) - 1):
            Dt[i] = T[i+1] - T[i]
    else:
        Dt = T

    if ((Dt < 0.0).any()):
        print("The time for completing a piece of trajectory \
              cannot be negative")
        return (A, b)


    counter = 0;
    for i in range(nWp):            # For each waypoint
        for j in range(nConstr):        # For each constraint
            # If not (First or Last)
            if (i > 0 and i < (nWp - 1)):
                # If specified wp
                if (not np.isnan(X[j,i])):
                    # Waypoint "i" is connects Polynomial i-1 and Polynomial i
                    b[counter] = X[j,i]

                    v = t_vec(Dt[i-1], deg)
                    A[counter, selIndex(i-1, nCoef)] = polyder(v, j)
                    counter = counter + 1

                    b[counter] = X[j,i]
                    v = t_vec(0, deg)
                    A[counter, selIndex(i, nCoef)] = polyder(v, j)
                # Only continuity constraint
                else:
                    b[counter] = 0

                    v = t_vec(Dt[i-1], deg)
                    A[counter, selIndex(i-1, nCoef)] = polyder(v, j)

                    v = t_vec(0, deg)
                    A[counter, selIndex(i, nCoef)] = -1 * polyder(v, j)

            # If First of Last
            else:
                if (i == 0):
                    b[counter] = X[j,i]
                    v = t_vec(0, deg)
                    A[counter, selIndex(i, nCoef)] = polyder(v, j)
                else:
                    b[counter] = X[j,i]
                    v = t_vec(Dt[i-1], deg)
                    A[counter, selIndex(i-1, nCoef)] = polyder(v, j)
            counter = counter + 1

    return (A,b)

## =================================================
## =================================================
def genQ(Dt, deg, der):
    """
    The time vector has a length of deg + 1, thus
    the Q matrix is (deg + 1) x (deg + 1).
    Minimizing the der means evaluating the der-th derivative,
    of the polynomial. The first der element of the 
    differentiated time vector are 0.
    The Q matrix for the single polynomial is then different
    from zero in the element with i and j > der.

    The integral is computed adding t / (index_i + index_y + 1) 
    to the outer product matrix.

    """
    num_poly = len(Dt)

    sizeQ = (num_poly * (deg + 1), num_poly * (deg + 1))
    Q = np.zeros(sizeQ, dtype=float)

    if (deg < der):
        print("Unable to minimize for the snap (Polynomial degree too low)")
        return Q    
   
    q = []
    for k in range(num_poly):
        # Time instant
        t_ = t_vec(Dt[k], deg)
        # Derivative of the time vector
        T = polyder(t_, der)
        TT = np.outer(T, T);
        # Compute the integral
        for i in range(deg - der):
            for j in range(deg - der):
                TT[i + der][j + der] = (TT[i + der][j + der]) * Dt[k]/(i + j + 1)
        
        Q[deg * k: deg * (k  + 1) + 1, deg * k: deg * (k  + 1) + 1] = TT 

    return Q


## =================================================
## =================================================
def genQ_snap(Dt, deg):
    """
    The time vector has a length of deg + 1, thus
    the Q matrix is (deg + 1) x (deg + 1).
    Minimizing the snap means evaluating the 4th derivative,
    of the polynomial. The first 4 element of the 
    differentiated time vector are 0.
    The Q matrix for the single polynomial is then different
    from zero in the element with i and j > 3.

    The integral is computed adding t / (index_i + index_y + 1) 
    to the outer product matrix.

    """
    num_poly = len(Dt)

    sizeQ = (num_poly * (deg + 1), num_poly * (deg + 1))
    Q = np.zeros(sizeQ, dtype=float)

    if (deg < 4):
        print("Unable to minimize for the snap (Polynomial degree too low)")
        return Q    
   
    q = []
    for k in range(num_poly):
        # Time instant
        t_ = t_vec(Dt[k], deg)
        # Derivative of the time vector
        T = polyder(t_, 4)
        TT = np.outer(T, T);
        # Compute the integral
        for i in range(deg - 4):
            for j in range(deg - 4):
                TT[i + 4][j + 4] = (TT[i + 4][j + 4]) * Dt[k]/(i + j + 1)
        
        Q[deg * k: deg * (k  + 1) + 1, deg * k: deg * (k  + 1) + 1] = TT 

    return Q


## =================================================
## =================================================
def pp2file(Dt, polysX, polysY, polysZ, polysW, filename):
    """
    Save the polynomial coefficients to file
    The file will have a row for each piece and the row has the
    following format:

    Time_interval x_poly_coeff y_poly_coeff z_poly_coeff w_poly_coeff

    """
    f = open(filename, "w")

    # I consider all the polynomial to be the same size
    nPieces = polysX.shape[0]
    pollen = polysX.shape[1]

    f.write('Pieces: ' + str(nPieces) + ', ' + 'Ncoef: ' + str(pollen) + '\n')
    for i in range(nPieces):
        f.write('{:5f}, '.format(Dt[i]))
        for j in range(pollen):
            f.write('{:5f}, '.format(polysX[i,j]))
        for j in range(pollen):
            f.write('{:5f}, '.format(polysY[i,j]))
        for j in range(pollen):
            f.write('{:5f}, '.format(polysZ[i,j]))
        for j in range(pollen):
            if (j == pollen - 1):
                f.write('{:5f}'.format(polysW[i,j]))
            else:
                f.write('{:5f}, '.format(polysW[i,j]))
        f.write('\n')

    f.close()

def ppFromfile(filename):
    """
    Read the polynomial coefficients from a file 
    The file will have a row for each piece and the row has the
    following format:

    Time_interval x_poly_coeff y_poly_coeff z_poly_coeff w_p ly_coeff

    """
    csvfile = open(filename, "r")

    reader = csv.reader(csvfile, delimiter=',') 
    
    data = [];
    for row in reader:
        data.append(row)

    # There is an extra initial row.
    nPieces = len(data) - 1

    # The first element is the time
    pollen = int(len(data[1]) - 1)//4

    print("Loading polynomial from file...")
    print("Num Pieces: " + str(nPieces))
    print("Num Coeff: " + str(pollen))

    polysX = np.zeros((nPieces, pollen), dtype=float)
    polysY = np.zeros((nPieces, pollen), dtype=float)
    polysZ = np.zeros((nPieces, pollen), dtype=float)
    polysW = np.zeros((nPieces, pollen), dtype=float)

    Dt = np.zeros((nPieces), dtype=float)
    for i in range(nPieces):
        Dt[i] = data[i + 1][0]
        for j in range(pollen):
            polysX[i,j] = data[i + 1][j + 1]
            polysY[i,j] = data[i + 1][j + 1 + 8] 
            polysZ[i,j] = data[i + 1][j + 1 + 16]
            polysW[i,j] = data[i + 1][j + 1 + 24]

    print("Dt:")
    print(Dt)
    print("Imported polyX:")
    print(polysX)
    print("Imported polyY:")
    print(polysY)
    print("Imported polyZ:")
    print(polysZ)
    print("Imported polyW:")
    print(polysW)
    csvfile.close()

    return (Dt, polysX, polysY, polysZ, polysW) 

######################################################################
## Helper functions for polynomial generation


## =================================================
## =================================================
def t_vec(t, deg):
    """
    Generate the time polynomial vector [1 t t^2 t^3 ... t^deg]
    """
    v = np.ones((deg + 1), dtype=float)

    for i in range(1, deg+1):
        v[i] = v[i-1] * t

    return v;


## =================================================
## =================================================
def derivmat(deg):
    """
    Compute the matrix, which maps time polynomial vector in their
    derivative.
    """
    msize = deg + 1
    M = np.zeros((msize, msize), dtype=float)

    for i in range(deg):
        M[i + 1][i] = i + 1

    return M


## =================================================
## =================================================
def polyder(np_arr_t, n_der):
    """
    Compute the n_der derivative of a given time vector vector
    """
    # Compute the length of the array
    v_len = np.size(np_arr_t)

    if (n_der > v_len - 1):
        print("Derivative order higher than the degree of the polynomial!")
        return np.zeros((v_len), dtype = float)

    v = np_arr_t

    # Derivative matrix (vlen = deg + 1)
    M = derivmat(v_len - 1)

    for i in range(1,n_der + 1):
        v = np.matmul(M, v)

    return v


## =================================================
## =================================================
def constrMat(tBase, indConstrDer):
    """
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

    """
    if (type(indConstrDer) == list):
        nconstr = len(indConstrDer)
    else:
        if (type(indConstrDer) == np.ndarray):
            nconstr = indConstrDer.size
        else:
            print("Wrong type for the index variable")
            return


    ncoeff = tBase.size

    A = np.zeros((nconstr, ncoeff), dtype = float)

    for i in range(nconstr):
        A[i, :] = polyder(tBase, indConstrDer[i]).transpose()

    return A


## =================================================
## =================================================
def numOfEquations(W):
    """
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
    """

    nWr = W.shape[0]
    nWc = W.shape[1]

    # First and End condition (fixed points)
    nCr = 2 * nWr

    # Junction points of the trajecotry
    # For each fixed point condition we will get 2 conditions, whilst, for
    # the smooth constraints we will obtain only 1 condition.

    # Select the waypoints, which are shared between polynomial
    Wshared = W[:, 1: (nWc - 1)]
    fWshared = Wshared.flatten()

    # Count how many fixed point condition and smooth condition I have
    nshared_fix = np.count_nonzero(~np.isnan(fWshared))
    nshared_smooth = np.count_nonzero(np.isnan(fWshared))
    nCr += 2 * nshared_fix + nshared_smooth

    return nCr


## =================================================
## =================================================
def selIndex(i, numCoeff):
    """
    Generate the indexes to select a subvector made of pieces of
    length numCoeff
    """
    return range(numCoeff * i, numCoeff * (i + 1))
