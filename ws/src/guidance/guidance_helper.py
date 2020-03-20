import numpy as np
import numpy.matlib
import math
import time
import rospy


################ HELPERS #################
def posFromPoseMsg(pose_msg):
    pos = np.array([pose_msg.pose.position.x, 
                pose_msg.pose.position.y, 
                pose_msg.pose.position.z])
    return pos

def quatFromPoseMsg(pose_msg):
    quat = np.array([pose_msg.pose.orientation.w,
        pose_msg.pose.orientation.x, 
        pose_msg.pose.orientation.y,
        pose_msg.pose.orientation.z])
    return quat 

def posFromOdomMsg(odom_msg):
    pos = np.array([odom_msg.p.x, 
                odom_msg.p.y, 
                odom_msg.p.z])
    return pos

def velFromOdomMsg(odom_msg):
    vel = np.array([odom_msg.v.x, 
                odom_msg.v.y, 
                odom_msg.v.z])
    return vel

def quatFromOdomMsg(odom_msg):
    quat = np.array([odom_msg.q.w,
        odom_msg.q.x, 
        odom_msg.q.y,
        odom_msg.q.z])
    return quat 


def quat2yaw(q):
    yaw = math.atan2(2.0 * (q[0] * q[3] + q[1] * q[2]),
            1.0 - 2.0 * (q[2]**2 + q[3]**2))
    return yaw

def quat2Z(q):
    Z = np.array([2.0 * (q[0]*q[2] + q[1]*q[3]),
         2.0*(q[2]*q[3] - q[0]*q[1]),
         1.0 - 2.0 * (q[1] * q[1] + q[2] * q[2])]) 
    return Z

# Convert from quaternion to euler angles
def ToEulerAngles(q):
    # roll (x-axis rotation)
    sinr_cosp = 2.0 * (q[0] * q[1] + q[2] * q[3]);
    cosr_cosp = 1.0 - 2.0 * (q[1] * q[1] + q[2] * q[2]);
    roll = math.atan2(sinr_cosp, cosr_cosp);

    # pitch (y-axis rotation)
    sinp = 2.0 * (q[0] * q[2] - q[3] * q[1]);

    pitch = math.asin(sinp);

    # yaw (z-axis rotation)
    siny_cosp = 2.0 * (q[0] * q[3] + q[1] * q[2]);
    cosy_cosp = 1.0 - 2.0 * (q[2] * q[2] + q[3] * q[3]);

    yaw = math.atan2(siny_cosp, cosy_cosp);

    return np.array([roll, pitch, yaw])


def Rx(a):
    # Roll Matrix
    R = np.eye(3)
    R[1][1] = math.cos(a)
    R[1][2] = -math.sin(a)
    R[2][1] = math.sin(a)
    R[2][2] = math.cos(a)
    return R
    
def Ry(a):
    # Pitch Matrix
    R = np.eye(3)
    R[0][0] = math.cos(a)
    R[0][2] = math.sin(a)
    R[2][0] = -math.sin(a)
    R[2][2] = math.cos(a)
    return R

def Rz(a):
    # Yaw Matrix
    R = np.eye(3)
    R[0][0] = math.cos(a)
    R[0][1] = -math.sin(a)
    R[1][0] = math.sin(a)
    R[1][1] = math.cos(a)
    return R

def quat2Rot(q):
    # Quaternion to Rotation matrix
    R = np.zeros((3,3))
    eul = ToEulerAngles(q)

    # Yaw * Pitch * Roll (Robotic convention)
    R = np.matmul(np.matmul(Rz(eul[2]), Ry(eul[1])), Rx(eul[0]))
    return R


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


def vex(v):
    """
    Compute the Vex matrix from a 
    vector
    """
    O = np.zeros((3,3), dtype = float)

    O[0, 1] = -v[2]
    O[0, 2] = v[1]
    O[1, 0] = v[2]
    O[1, 2] = -v[0]

    O[2, 0] = v[1]
    O[2, 1] = v[0]

    return O

def vee(M):
    """
    Compute the Vee vector from a 
    matrix 
    """
    v = np.zeros((3), dtype = float)

    v[2] = -M[0, 1] 
    v[1] = M[0, 2]
    v[0] = M[1, 2]

    return v

def Rtoq(R):
    q = np.zeros(4)
    q[0] = 1.0

    psi = np.arccos((np.trace(R) - 1.0) / 2.0)

    if (abs(psi) > 0.001):
        u = vee(R - R.transpose()) / (2.0 * math.sin(psi))

        q[0] = math.cos(psi / 2.0)
        q[1:4] = u * math.sin(psi / 2.0)
    
    return q

        
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

    u = np.reshape(u, (3, 1))
    B1 = np.eye((3), dtype=float) * (dt*dt)/2.0
    B2 = np.eye((3), dtype=float) * dt
    B = np.vstack((B1, B2)) 
   
    # print("A = \n", A)
    # print("B = \n", B)
    
    if (direction == -1):
        # x(k) = A_(dt) x(k+1) + B_(dt) u(k)
        A = np.linalg.inv(A)
        # print("Back A = \n", A)
        B = -1.0 * np.matmul(A, B)
        # print("Back B = \n", B)
  
    Xx = np.matmul(A, X)
    Xu = np.matmul(B, u)

    # print("Ax = \n", Xx)
    # print("Bu = \n", Xu)
    X =  Xx + Xu

    return X


def IntegrationAtt(q, w, Tf, dt, direction):
    t = 0.0  
    N = 4
    out_state = np.zeros((N, 1), dtype=float)
    
    X = np.vstack((q))

    while (t < Tf):
        X = integrationAttStep(X, w, dt, direction)
        t = t + dt

    out_state = np.copy(X)

    return out_state


# Integration Step 
def integrationAttStep(X, w, dt, direction):
    # x(k+1) = A(dt) x(k)  + B(dt) u(k)
    wq = np.concatenate(([0], -w))  
    
    qd = M(X) * wq * 0.5

    X = X + qd * dt * direction

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


def evalObstacleInt(p0, pf, po, mindist):
    """
    p0: Starting position
    pf: Final position
    po: Obstacle position
    """
    V = np.zeros(2)
    Ve = np.zeros(2)

    V = pf[0:2] - p0[0:2]
    n = V / np.linalg.norm(V)

    Ve = po[0:2] - p0[0:2]
    print(Ve)
    t = np.dot(Ve, n) * n 
    e = t - Ve

    curr_mindist = np.linalg.norm(e)
    print("Distance from path")
    print(curr_mindist)

    if (curr_mindist < mindist):
        return (True, e)
    else:
        return (False, e)

def genAvoidWaypoints(p0, pf, po, r): 
    o = np.array(po[0:2])
    p = np.array(p0[0:2])
    f = np.array(pf[0:2])

    wps = []

    W = np.zeros(2)
    b = 0.0
    # Compute the hyperplane
    # The p0 is quite [0, 0]
    if (np.linalg.norm(p) < 0.001 and np.linalg.norm(f) > 0.001):
        W = np.array([f[1], -f[0]])
        W = W / np.linalg.norm(W)
        b = 0.0;
    
    if (np.linalg.norm(p) > 0.001 and np.linalg.norm(f) < 0.001):
        W = np.array([p[1], -p[0]])
        W = W / np.linalg.norm(W)
        b = 0.0
    
    if (np.linalg.norm(p) > 0.001 and np.linalg.norm(f) > 0.001):
        X = np.concatenate(([p], [f]))
        W = -np.matmul(np.linalg.inv(X), np.ones(2))
        b = 1.0
 
    Vertex = np.matlib.repmat([o], 4, 1)
    temp = np.matlib.repmat(np.array([r, r]), 4, 1)
    temp_1 = np.concatenate((
        np.transpose(np.array([1, 1, -1, -1], ndmin = 2)),
        np.transpose(np.array([1, -1, 1, -1], ndmin = 2))
        ), axis=1)
    Vertex = Vertex +  temp * temp_1

    print("Obstacle vertices: ")
    print(Vertex)

    # Check which vertex should I pass by 
    if (np.dot(o, W) + b < 0):
        # Select vertex > 0
        for i in range(Vertex.shape[0]):
            if (np.dot(W, Vertex[i]) + b) > 0:
                print("Adding Vertex")
                print(Vertex[i])
                wps.append(Vertex[i])
    else:
        # Select vertex > 0
        for i in range(Vertex.shape[0]):
            if (np.dot(W, Vertex[i]) + b) < 0:
                print("Adding Vertex")
                print(Vertex[i])
                wps.append(Vertex[i])


    if (len(wps) == 0):
            return list(wps)

    wps_dist = map(np.linalg.norm, wps - p0[0:2])
    print("Waypoints Distances")
    print(wps_dist)

    wps_ = [] 
    
    index = np.argsort(wps_dist)

    for el in list(index):
        wps_.append(np.concatenate((wps[el], [pf[2]])))

    print("Ordered Waypoints")
    print(wps_)
    wps = np.copy(wps_) 
    return list(wps)


def computeTerminalNormalVelAcc(tg_q, v_norm, Tz_norm):
    """
    Given the orientation of a surface and the norm of
    the velocity and acceleration, returns the vectors
    of the velocity and accelation perpendicular to the
    surface.
    """
    # Compute the normal of the target surface
    tg_Zi = quat2Z(tg_q)

    n = np.matmul(vex(np.array([0, 0, 1.0])), tg_Zi)
    norm_n = np.linalg.norm(n)
    alpha = math.asin(norm_n)

    angle_lim = math.pi / 3.0
    if (abs(alpha) > angle_lim):
        print("Reducing Angle")
        new_angle = angle_lim * np.sign(alpha) 
        temp_quat = np.concatenate((
            [math.cos(new_angle / 2.0)], 
            n * math.sin(new_angle / 2.0)))

        tg_Zi = quat2Z(temp_quat)

    # Compute the acceleration vector
    a_dem = Tz_norm * tg_Zi - np.array([0.0, 0.0, 9.81])

    # Compute the velocity vector on the target
    v_dem = -v_norm * tg_Zi

    return (v_dem, a_dem)


def computeTerminalNormalVel(tg_q, v_norm):
    """
    Given the orientation of the target and the norm
    of the velocity at which we want to hit it, returns
    the velocity vector perpendicular to the surface of
    the target
    """
    # Compute the normal of the target surface
    tg_Zi = quat2Z(tg_q)

    # Compute the velocity vector on the target
    v_dem = -v_norm * tg_Zi
    a_dem = np.zeros(3, dtype=float)

    return (v_dem, a_dem)
    

# Generate the Terminal Flight
def computeTerminalTrj(tg, tg_v, tg_a, DT):

    # Compute the waypoints near the target
    # I am considering moving with a constant acceleration (negative), while going towards the target
    a_pre = tg_a

    if (DT < 0.001):
        p_pre = np.reshape(tg, (3,))
        v_pre = np.reshape(tg_v, (3,))
    else:
        xv_pre = Integration(tg, tg_v, tg_a, DT, 0.0001, -1) 
        p_pre = np.reshape(xv_pre[0:3], (3,))
        v_pre = np.reshape(xv_pre[3:6], (3,))
    
    print("Acc on TG = \n", tg_a)
    print("Vel on TG = \n", tg_v)
    print("Pos on TG = \n", tg)

    return (p_pre, v_pre, a_pre)


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



def genInterpolationMatrices(start_vel, tg_prel, tg_v, tg_a):
    """
    Generate interpolation matrices for a given starting speed
    and a generic end point. The position is considered to be
    relative to the vehicle.
    """
    X = np.array([
        [ 0.0,          tg_prel[0]],
        [ start_vel[0], tg_v[0]],
        [ 0.0,          tg_a[0]],
        ])

    Y = np.array([
        [ 0.0,          tg_prel[1]],
        [ start_vel[1], tg_v[1]],
        [ 0.0,          tg_a[1]],
        ])
    
    Z = np.array([
        [ 0.0,          tg_prel[2]],
        [ start_vel[2], tg_v[2]],
        [ 0.0,          tg_a[2]],
        ])

    W = np.array([
        [ 0.0,    0.0],
        [ 0.0,    0.0],
        [ 0.0,    0.0],
        ])

    return (X, Y, Z, W)



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
