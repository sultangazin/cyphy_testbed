import numpy as np
import math
import time
import rospy


############################   Helper functions

def quat2yaw(q):
    yaw = math.atan2(2.0 * (q[0] * q[3] + q[1] * q[2]),
            1.0 - 2.0 * (q[2]**2 + q[3]**2))
    return yaw

def quat2Z(q):
    Z = np.array([2.0 * (q[0]*q[2] + q[1]*q[3]),
         2.0*(q[2]*q[3] - q[0]*q[1]),
         1.0 - 2.0 * (q[1] * q[1] + q[2] * q[2])]) 
    return Z

def Mq(p):
    M = np.array([
        [p[0], -p[1], -p[2], -p[3]], 
        [p[1], p[0], -p[3], p[2]],
        [p[2], p[3], p[0], -p[1]],
        [p[3], -p[2], p[1], p[0]]
        ])
    return M

def quatMult(p, q):
    M = np.array([
        [p[0], -p[1], -p[2], -p[3]], 
        [p[1], p[0], -p[3], p[2]],
        [p[2], p[3], p[0], -p[1]],
        [p[3], -p[2], p[1], p[0]]
        ])
    res = np.matmul(M, q)


# Create the knots vector
def updateKnots(t_impact, dt, Trec):
    t1 = t_impact - dt
    t2 = t_impact + Trec
        
    knots = np.array([0,
                      t1,
                      t_impact,
                      t2]
                    )
    return knots


def AddConstraint(A, constr):
    N = constr.size
    col_constr = constr.reshape(N,1)
    if (A.size == 0):
        A = col_constr
    else:
        A = np.hstack((A, col_constr))
    return A

# Integration Step 
def integrationStep(X, u, dt, direction):
    # x(k+1) = A(dt) x(k)  + B(dt) u(k)
    A = np.eye((6), dtype=float)
    A[0][3] = dt
    A[1][4] = dt
    A[2][5] = dt

    B1 = np.eye((3), dtype=float) * (dt**2)/2.0
    B2 = np.eye((3), dtype=float) * dt
    B = np.vstack((B1, B2))
    
    if (direction == -1):
        # x(k) = A_(dt) x(k+1) + B_(dt) u(k)
        A = np.linalg.inv(A)
        B = -1.0 * np.matmul(A, B)
  
    X = np.matmul(A, X) + np.matmul(B, u)
    return X


# Back integration
def Integration(p, v, a, Tf, dt, direction):  
    t = 0.0  
    N = p.size
    out_state = np.zeros((N, 1), dtype=float)
    
    # State vector composed by position and velocity
    if (N > 1):
        p = np.resize(p, (N,1))
        v = np.resize(v, (N,1))
        a = np.resize(a, (N,1))
    
    X = np.vstack((p, v))

    while (t < Tf):
        X = integrationStep(X, a, dt, direction)
        t = t + dt

    out_state = np.copy(X)

    return out_state


# Generate the Terminal Flight
def computeTerminalTrjStart(tg, tg_q, v_norm, a_norm, DT):

    # Compute the normal of the target surface
    tg_Zi = np.array([2.0*tg_q[0]*tg_q[2] + 2.0*tg_q[1]*tg_q[3],
         2.0*(tg_q[2]*tg_q[3] - tg_q[0]*tg_q[1]),
         tg_q[0]*tg_q[0] -tg_q[1]*tg_q[1] -tg_q[2]*tg_q[2] + tg_q[3]*tg_q[3]]) 

    # Compute the acceleration vector
    a_dem = a_norm * tg_Zi - np.array([0.0, 0.0, 9.81])
    # Compute the velocity vector on the target
    v_dem = -v_norm * tg_Zi

    # Compute the waypoints near the target
    # I am considering moving with a constant acceleration (negative), while going towards the target
    xv_pre = Integration(tg, v_dem, a_dem, DT, 0.001, -1) 
    p_pre = np.reshape(xv_pre[0:3], (3,))
    v_pre = np.reshape(xv_pre[3:6], (3,))
    
    return (p_pre, v_pre, a_dem)


# Generate the matrices for the interpolation problem
def genInterpolProblem(tg, vtg, atg, yaw, t_impact):

    X = np.array([[]])
    Y = np.array([[]])
    Z = np.array([[]])
    W = np.array([[]])
    
    xtrg = np.array([tg[0], vtg[0], atg[0], 0]) 
    ytrg = np.array([tg[1], vtg[1], atg[1], 0])
    ztrg = np.array([tg[2], vtg[2], atg[2], 0]) 
    
    
    X = AddConstraint(X, np.array([0,0,0,0]))
    Y = AddConstraint(Y, np.array([0,0,0,0]))
    Z = AddConstraint(Z, np.array([0,0,0,0]))
    W = AddConstraint(W, np.array([0,0,0,0]))

    knots = np.array([0.0])
    N = 0
    for i in range(N):
        xst = np.array([tg[0] * (i + 1)/(N + 1), np.nan, np.nan, np.nan]) 
        X = AddConstraint(X, xst)

        yst = np.array([tg[1] * (i + 1)/(N + 1), np.nan, np.nan, np.nan])
        Y = AddConstraint(Y, yst)

        zst = np.array([tg[2] * (i + 1)/(N + 1), np.nan, np.nan, np.nan]) 
        Z = AddConstraint(Z, zst)
    
        W = AddConstraint(W, np.array([0, np.nan, np.nan, 0]))

        knots = np.append(knots, t_impact * (i + 1)/(N + 1))

    X = AddConstraint(X, xtrg)
    Y = AddConstraint(Y, ytrg)
    Z = AddConstraint(Z, ztrg)
    W = AddConstraint(W, np.array([0, yaw, 0, 0]))
  
    knots = np.append(knots, t_impact)

    print("\n\n ===============  Generated Waypoints ============ ")
    print("Relative Target = \n", tg)
    print("Vel = \n", vtg)
    print("Acc = \n", atg)
    print("Knots = \n ", knots)
    print("\n")

    return (X, Y, Z, W, knots)


def genInterpolMatricesBezier(tg, tg_q, yaw, v_norm, a_norm):
    
    # Extract the coordinates of the target Z axis from the rotation matrix
    # extressed with the quaternion
    tg_Zi = quat2Z(tg_q)

    a_dem = a_norm * tg_Zi - np.array([0.0, 0.0, 9.81])
    v_dem = - v_norm * tg_Zi

    rospy.loginfo("Rel Target = " +  str(tg))
    rospy.loginfo("Vel = " + str(v_dem))
    rospy.loginfo("Acc = " +  str(a_dem))

    #   Relative waypoint data
    #   Start   PreTarget   Target      PostTarget  End
    X = np.array([
        [ 0,   tg[0]    ],
        [ 0,   v_dem[0] ],
        [ 0,   a_dem[0] ],
    ])

    Y = np.array([
        [ 0,   tg[1]    ],
        [ 0,   v_dem[1] ],
        [ 0,   a_dem[1] ],
    ])

    Z = np.array([
        [ 0,   tg[2]    ],
        [ 0,   v_dem[2] ],
        [ 0,   a_dem[2] ],
    ])

    W = np.array([
        [ 0,   yaw ],
        [ 0,   0.0  ],
        [ 0,   0.0  ],
    ])
    
    return (X, Y, Z, W)



def getInterpolMatrices(tg, tg_q, yaw, v_norm, a_norm, dt):
    
    # Extract the coordinates of the target Z axis from the rotation matrix
    # extressed with the quaternion
    tg_Zi = quat2Z(tg_q)

    a_dem = a_norm * tg_Zi - np.array([0.0, 0.0, 9.81])
    v_dem = - v_norm * tg_Zi

    # Compute the waypoints near the target
    p_end = tg + v_dem * dt
    
    rospy.loginfo("Rel Target = " +  str(tg))
    rospy.loginfo("Vel = " + str(v_dem))
    rospy.loginfo("Acc = " +  str(a_dem))
    rospy.loginfo("Rel Recoil point = " + str(p_end))

    #   Relative waypoint data
    #   Start   PreTarget   Target      PostTarget  End
    X = np.array([
        [ 0,   tg[0],      p_end[0]],
        [ 0,   v_dem[0],   0.0],
        [ 0,   a_dem[0],   0.0],
        [ 0,   np.nan,     0.0]
    ])

    Y = np.array([
        [ 0,   tg[1],      p_end[1]],
        [ 0,   v_dem[1],   0.0],
        [ 0,   a_dem[1],   0.0],
        [ 0,   np.nan,     0.0]
    ])

    Z = np.array([
        [ 0,   tg[2],      p_end[2]],
        [ 0,   v_dem[2],   0.0],
        [ 0,   a_dem[2],   0.0],
        [ 0,   np.nan,     0.0]
    ])

    W = np.array([
        [ 0,   yaw,       0.0],
        [ 0,   np.nan,    0.0],
        [ 0,   np.nan,    0.0],
        [ 0,   np.nan,    0.0]
    ])
    
    return (X, Y, Z, W)



