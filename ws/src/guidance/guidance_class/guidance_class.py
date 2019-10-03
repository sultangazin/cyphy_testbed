# Guidance class
import os
import sys
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '../trjgen')))
import numpy as np

import rospy

from tf.transformations import euler_from_matrix

from testbed_msgs.msg import CustOdometryStamped
from testbed_msgs.msg import ControlSetpoint 
from geometry_msgs.msg import PoseStamped

from guidance.srv import GenImpTrajectoryAuto
from guidance.srv import GenTrackTrajectory
from guidance.srv import ExeMission

import trjgen.class_pwpoly as pw
import trjgen.class_trajectory as trj
import trjgen.trjgen_helpers as trjh
import trjgen.trjgen_core as tj

import trjgen.class_bz as bz
import trjgen.class_bztraj as bz_t

from guidance_helper import *
from guidance_helper_classes.mission_class import Mission, MissionType, TrajectoryType
from guidance_helper_classes.trajectorygenerator_class import TrajectoryGenerator


class ConstAttitudeTrj:
    def __init__(self, p0, v0, a, r, p, y):
        self.r = 0
        self.p = 0
        self.y = 0

        self.p_0 = p0
        self.v_0 = v0
        self.a_0 = a 

        self.dt = 0.001
        
        self.pos = np.zeros(3)
        self.vel = np.zeros(3)

        self.r = r
        self.p = p
        self.y = y

        #self.qf = ToQuaternion(r, p, y)

    def eval(self, t):
        x = Integration(self.p_0, self.v_0, self.a_0, t, self.dt, 1)
        
        self.pos = x[0:3]
        self.vel = x[3:6]
        
        X = np.zeros(3, dtype=float)
        Y = np.zeros(3, dtype=float)
        Z = np.zeros(3, dtype=float)

        X[0] = self.pos[0]
        Y[0] = self.pos[1]
        Z[0] = self.pos[2]

        X[1] = self.vel[0]
        Y[1] = self.vel[1]
        Z[1] = self.vel[2]

        X[2] = self.a_0[0]
        Y[2] = self.a_0[1]
        Z[2] = self.a_0[2]

        return (X, Y, Z, self.r, self.p, self.y)


class MissionQueue:
    def __init__(self):
        self.missList = []
        self.numItems = 0
        self.currItem = None

    def insertItem(self, MissionItem):
        print("Insert item")
        self.missList.append(MissionItem)
        self.numItems = len(self.missList)
        print("{} items in the list".format(self.numItems))

    def reset(self):
        print("Resetting missions queue")
        self.missList = []
        self.numItems = 0

    def update(self, MissionItem):
        print("Update MissionQueue with a new mission")
        self.reset()
        self.insertItem(MissionItem)
    
    def getNumItems(self):
        print("Number of items = {}".format(self.numItems))
        return self.numItems

    def getItemAtTime(self, t):
        if (len(self.missList) == 0):
            return None

        for i in range(self.numItems):
            if (self.missList[i].isStop):
                return self.missList[i] 

            if (t < self.missList[i].t_stop and t > self.missList[i].t_start):
                if (self.currItem != i):
                    self.currItem = i
                    print("Mission {} active".format(i))
                    print("t = {} in [{}, {}] \n".format(t, self.missList[i].t_start, self.missList[i].t_stop))

                return self.missList[i]

        return None


###### HELPERS ######
def gen_MissionAuto(ndeg, v0, x0, xf, vf, af, T): 
    # Generate the interpolation matrices to reach the pre-impact 
    # point with a given speed and acceleration.
    (Xwp, Ywp, Zwp, Wwp) = genInterpolationMatrices(v0, xf - x0, vf, 
            af)

    # Generation of the trajectory with Bezier curves
    x_lim = [3.5, 2.5, 9.0]
    y_lim = [3.5, 2.5, 9.0]
    z_lim = [2.4, 2.3, 5.0]

    x_cnstr = np.array([[-x_lim[0], x_lim[0]], [-x_lim[1], x_lim[1]], [-x_lim[2], x_lim[2]]])
    y_cnstr = np.array([[-y_lim[0], y_lim[0]], [-y_lim[1], y_lim[1]], [-y_lim[2], y_lim[2]]])
    z_cnstr = np.array([[-x0[2], z_lim[0]], [-z_lim[1], z_lim[1]], [-9.90, z_lim[2]]])

    # Generate the polynomial
    #
    print("Generating X")
    bz_x = bz.Bezier(waypoints=Xwp, constraints=x_cnstr, degree=ndeg, s=T, opt_der=3)
    print("\nGenerating Y")
    bz_y = bz.Bezier(waypoints=Ywp, constraints=y_cnstr, degree=ndeg, s=T, opt_der=3)
    print("\nGenerating Z")
    bz_z = bz.Bezier(waypoints=Zwp, constraints=z_cnstr, degree=ndeg, s=T, opt_der=3)
    print("\nGenerating W")
    bz_w = bz.Bezier(waypoints=Wwp, degree=7, s=T, opt_der=4)

    print(" ================================================== \n")
    print("Summary ")
    print("Pos pre = \n", xf)
    print("Vel pre = \n", vf)
    print("Acc pre = \n", af)

    print("Final Relative Position: [%.3f, %.3f, %.3f]"%(Xwp[0,1], Ywp[0,1], Zwp[0,1]))
    print("Final Velocity: [%.3f, %.3f, %.3f]"%(Xwp[1,1], Ywp[1,1], Zwp[1,1]))
    print("Final Acceleration: [%.3f, %.3f, %.3f]"%(Xwp[2,1], Ywp[2,1], Zwp[2,1]))

    print("Solution Position: [%.3f, %.3f, %.3f]"%(bz_x.eval(T), bz_y.eval(T), bz_z.eval(T)))
    print("Solution Velocity: [%.3f, %.3f, %.3f]"%
            (bz_x.eval(T, [1]), bz_y.eval(T, [1]), bz_z.eval(T, [1])))
    print("Solution Acceleration: [%.3f, %.3f, %.3f]"%(bz_x.eval(T, [2]), bz_y.eval(T, [2]), bz_z.eval(T, [2])))


    trj_obj = TrajectoryGenerator(bz_t.BezierCurve(bz_x, bz_y, bz_z, bz_w))
  
    # Compute the absolute times for this trajectory
    t_start = rospy.get_time()
    t_end = t_start + T

    mission = Mission()
    mission.update(
                p = x0,
                v = v0, 
                tg_p = xf, 
                tg_v = vf, 
                tg_a = af,
                trj_gen = trj_obj,
                start_time = t_start,
                stop_time = t_end, 
                ttype = TrajectoryType.FullTrj)

    return mission

def gen_MissionAtt(qf, x0, v0, xf, vf, af, t_s, DT):
    # Extract the Z axis from the target quaternion
    tg_Z = quat2Z(qf)
    #b_Z = quat2Z(qb)

    c_prod = np.matmul(vex(np.array([0,0,1])), tg_Z)
    c_prod_norm = np.linalg.norm(c_prod)

    theta = math.asin(c_prod_norm)
    n = c_prod / c_prod_norm 

    #R = quat2Rot(qb)
    #nb = np.matmul(np.transpose(R), n) 
    temp_q = np.concatenate(([math.cos(theta / 2.0)], 
        n * math.sin(theta / 2.0)))
   
    eul_angles = ToEulerAngles(temp_q) 

    endTrj = ConstAttitudeTrj(x0, v0, af, eul_angles[0], eul_angles[1], eul_angles[2])

    mission = Mission()
    t_start = t_s 
    t_end = t_start + DT

    mission.update(
                p = x0,
                v = v0, 
                tg_p = xf, 
                tg_v = vf,
                tg_a = af,
                trj_gen = endTrj,
                start_time = t_start,
                stop_time = t_end, 
                ttype = TrajectoryType.AttTrj)

    return mission






################# GUIDANCE CLASS #####################
class GuidanceClass:

    def __init__(self):

        self.initData()

        self.loadParameters()

        self.advertiseTopics()

        self.registerCallbacks()

        self.registerServices()


        rospy.loginfo("\n [%s] Guidance Initialized!"%rospy.get_name()) 


    def initData(self):
        self.current_odometry = CustOdometryStamped()
        self.current_target = PoseStamped()

        # Mission object to store information about
        # the current status of the plan.
        self.current_mission = Mission()

        self.mission_queue = MissionQueue()
        
        self.StopUpdating = True 

        self.Active = False

        self.Stopped =  True


    def loadParameters(self):
        self.vehicle_mass = rospy.get_param('~vehicle_mass', 0.032)

        self.target_frame = rospy.get_param('~target_frame', 'cf1')
     
        # Load the name of the Output Topics 
        self.ctrlsetpoint_topic_ = rospy.get_param('topics/out_ctrl_setpoint', "setpoint")

        # Load the name of the Input Topics
        self.dr_odom_topic_ = rospy.get_param('topics/in_vehicle_odom_topic', 'external_codom')    
        target_name = rospy.get_param('topics/in_tg_pose_topic', "target")
        self.tg_pose_topic_ = '/' + target_name + '/external_pose'

    def registerCallbacks(self):
        # Subscribe to vehicle state update
        rospy.Subscriber(self.dr_odom_topic_, CustOdometryStamped, self.odom_callback)
        rospy.Subscriber(self.tg_pose_topic_, PoseStamped, self.tg_callback)


    def registerServices(self):
        # Services 
        self.service_track = rospy.Service('gen_TrackTrajectory', 
            GenTrackTrajectory, self.handle_genTrackTrj)

        self.service_imp_auto = rospy.Service('gen_ImpTrajectoryAuto', 
            GenImpTrajectoryAuto, self.handle_genImpTrjAuto)

        self.service_vel_auto = rospy.Service('gen_TrajectoryAuto',
                GenImpTrajectoryAuto, self.handle_genTrjAuto)

        self.service_exe_mission = rospy.Service('exe_Mission',
                ExeMission, self.handle_exeMission)


    def advertiseTopics(self):
        # Setpoint Publisher
        self.ctrl_setpoint_pub = rospy.Publisher(self.ctrlsetpoint_topic_, 
            ControlSetpoint, queue_size=10)
 

    ###### CALLBACKS
    # On new vehicle information
    def odom_callback(self, odometry_msg):

        # Update the pose/twist information of the controlled vehicle
        self.current_odometry = odometry_msg
       
        # Evaluate where I am in the trajectory, using time information.
        t = rospy.get_time()
       
        current = self.mission_queue.getItemAtTime(t)
 
        
        # In case there are not other mission element to process...
        if current == None:
            # If is not the time to stop, ask for the last point of the 
            # last mission element.
            if not self.Stopped:
                output_msg = ControlSetpoint()
                output_msg.header.stamp = rospy.Time.now()

                recovery_time = 0.3
                (keep_pos, v, _) = self.current_mission.getEnd() 
                # Send the message once and then set the flag to stop
                # updating.
                if self.Active:
                    output_msg.p.x = keep_pos[0] + v[0] * recovery_time 
                    output_msg.p.y = keep_pos[1] + v[1] * recovery_time 
                    output_msg.p.z = keep_pos[2] + v[2] * recovery_time 
                    
                    output_msg.v.x = 0.0 
                    output_msg.v.y = 0.0
                    output_msg.v.z = 0.0
                    output_msg.a.x = 0.0
                    output_msg.a.y = 0.0
                    output_msg.a.z = 0.0
                    output_msg.setpoint_type  = "FullTrj"

                    print("End of Trajectory")
                    print("Stopping at \n", keep_pos)
                    self.ctrl_setpoint_pub.publish(output_msg)
                    self.Active = False
            # If a stop has been requested, send a stop 
            # command to the controller.
            else: 
                output_msg = ControlSetpoint()
                output_msg.header.stamp = rospy.Time.now()
                
                output_msg.setpoint_type = "StopCmd"
                self.ctrl_setpoint_pub.publish(output_msg)
                self.Active = False

        else:
            output_msg = ControlSetpoint()
            output_msg.header.stamp = rospy.Time.now()

            self.current_mission = current

            # If the current mission element is an empty mission,
            # reset the mission queue and set the flag to indicate
            # that a stop has been requested.
            if (self.current_mission.isStop == True):
                self.mission_queue.reset()
                self.Stopped = True
                return

            self.Stopped = False

            trj_type = self.current_mission.getTrjType()

            # Evaluate the current setpoint
            if trj_type != TrajectoryType.AttTrj:
                (X, Y, Z, W, R, Omega) = self.current_mission.getRef(t) 
                output_msg.setpoint_type = "FullTrj"
            else:
                (X, Y, Z, roll, pitch, yaw) = self.current_mission.getRef(t)
                output_msg.setpoint_type = "AttTrj" 

             
            # Fill  the trajectory object
            output_msg.p.x = X[0]
            output_msg.p.y = Y[0]
            output_msg.p.z = Z[0]

            output_msg.v.x = X[1] 
            output_msg.v.y = Y[1]
            output_msg.v.z = Z[1]
            output_msg.a.x = X[2]
            output_msg.a.y = Y[2]
            output_msg.a.z = Z[2]

            if (trj_type == TrajectoryType.FullTrj):
                # Conver the Rotation matrix to euler angles
                (roll, pitch, yaw) = euler_from_matrix(R)
            
                output_msg.rpy.x = roll
                output_msg.rpy.y = pitch
                output_msg.rpy.z = yaw
                output_msg.brates.x = Omega[0]
                output_msg.brates.y = Omega[1]
                output_msg.brates.z = Omega[2]

            if (trj_type == TrajectoryType.AttTrj):
                output_msg.rpy.x = roll
                output_msg.rpy.y = pitch
                output_msg.rpy.z = yaw            

            # Pubblish the evaluated trajectory
            if self.Active:
                self.ctrl_setpoint_pub.publish(output_msg)

        return


    # On new target information
    def tg_callback(self, pose_msg):
        self.current_target = pose_msg
        return


    ## ===================================================================
    #####                     Guidance Methods
    ## ===================================================================
    def gen_goTo(self, p, t2go = 0.0):
        """
        Generate a tracking trajectory to reach an absolute waypoint 
        with a given velocity and acceleration.
        """
        start_pos = posFromOdomMsg(self.current_odometry)
        start_vel = velFromOdomMsg(self.current_odometry)

        tg_p = p 
        tg_prel = tg_p - start_pos
        tg_v = np.zeros((3))
        tg_a = np.zeros((3))
                 
        goto_vel = 0.5 

        rospy.loginfo("Target Relative = [%.3f, %.3f, %.3f]" % (tg_prel[0], tg_prel[1], tg_prel[2]))

        (X, Y, Z, W) = genInterpolationMatrices(start_vel, tg_prel, tg_v, tg_a)
        
        if (t2go is not None and t2go != 0.0):
            t_end = t2go
        else:
            t_end = np.linalg.norm(tg_prel) / goto_vel 

        # Times (Absolute and intervals)
        knots = np.array([0, t_end]) # One second each piece

        # Polynomial characteristic:  order
        ndeg = 7

        ppx = pw.PwPoly(X, knots, ndeg)
        ppy = pw.PwPoly(Y, knots, ndeg)
        ppz = pw.PwPoly(Z, knots, ndeg)
        ppw = pw.PwPoly(W, knots, ndeg) 
        traj_obj = trj.Trajectory(ppx, ppy, ppz, ppw)

        print("Final Relative Position: [%.3f, %.3f, %.3f]"%(X[0,1], Y[0,1], Z[0,1]))
        print("Solution: [%.3f, %.3f, %.3f]"%(ppx.eval(t_end, 0), ppy.eval(t_end, 0), ppz.eval(t_end, 0)))

        # Update the mission object
        t_start = rospy.get_time()
        miss = Mission()
        miss.update(
                p = start_pos,
                v = start_vel, 
                tg_p = tg_p, 
                tg_v = tg_v, 
                tg_a = tg_a,
                trj_gen = traj_obj,
                start_time = t_start,
                stop_time = t_end + t_start,
                ttype = TrajectoryType.FullTrj
                )
        self.mission_queue.update(miss)

        return True

    def gen_goToBZ(self, p, t2go = 0.0):
        """
        Generate a tracking trajectory to reach an absolute waypoint 
        with a given velocity and acceleration.
        """
        start_pos = posFromOdomMsg(self.current_odometry)
        start_vel = velFromOdomMsg(self.current_odometry)

        tg_p = p 
        tg_prel = tg_p - start_pos
        tg_v = np.zeros((3))
        tg_a = np.zeros((3))
                 

        if (t2go == 0.0):
            goto_vel = 0.5 
            T = np.linarg.norm(tg_prel) / goto_vel
        else:
            T = t2go

        ndeg = 12
        mission_element = gen_MissionAuto(ndeg, start_vel, 
                start_pos, 
                tg_p,
                tg_v, 
                np.zeros(3, dtype=float), 
                T)

        self.mission_queue.update(mission_element)

        return True


    def gen_land(self, p = None):
        """
        Generate a tracking trajectory to reach an absolute waypoint 
        with a given velocity and acceleration.
        """
        start_pos = posFromOdomMsg(self.current_odometry)
        start_vel = velFromOdomMsg(self.current_odometry)

        vland = 0.3

        if (p is not None):
            tg_prel = np.array([p[0] - start_pos[0], p[1] - start_pos[1], -start_pos[2]])
            tg_p = np.copy(p)
            tg_p[2] = 0.0
        else:
            tg_prel = np.array([0.0, 0.0, -start_pos[2]])
            tg_p = np.copy(start_pos)
            tg_p[2] = 0.0

        t_end = (start_pos[2] / vland)
        tg_v = np.zeros((3))
        tg_a = np.zeros((3))
                 
        print("Landing in [{}, {}]\n".format(tg_prel[0], tg_prel[1]))

        (X, Y, Z, W) = genInterpolationMatrices(
                start_vel, 
                tg_prel, 
                tg_v, 
                tg_a)
        
        # Times (Absolute and intervals)
        knots = np.array([0, t_end]) # One second each piece

        # Polynomial characteristic:  order
        ndeg = 7

        ppx = pw.PwPoly(X, knots, ndeg)
        ppy = pw.PwPoly(Y, knots, ndeg)
        ppz = pw.PwPoly(Z, knots, ndeg)
        ppw = pw.PwPoly(W, knots, ndeg) 
        traj_obj = trj.Trajectory(ppx, ppy, ppz, ppw)


        # Update the mission object
        t_start = rospy.get_time()
        miss_element = Mission()
        miss_element.update(
                p = start_pos,
                v = start_vel, 
                tg_p = tg_p, 
                tg_v = tg_v, 
                tg_a = tg_a,
                trj_gen = traj_obj,
                start_time = t_start,
                stop_time = t_end + t_start,
                ttype = TrajectoryType.LandTrj
                )
        self.mission_queue.update(miss_element)
        self.mission_queue.insertItem(Mission())

        return True 


    def gen_takeoff(self, h, t2go=None):
        """
        Generate a tracking trajectory to reach an absolute waypoint 
        with a given velocity and acceleration.
        """
        start_pos = posFromOdomMsg(self.current_odometry)

        vtakeoff = 0.3
        t_end = h / vtakeoff

        if (t2go is not None and t2go != 0.0):
            t_end = t2go
            vtakeoff = h / t2go

        tg_p = start_pos + np.array([0, 0, h])
        tg_prel = np.array([0.0, 0.0, h])
        tg_v = np.zeros((3))
        tg_a = np.zeros((3))
                 
        (X, Y, Z, W) = genInterpolationMatrices(
                np.zeros(3), 
                tg_prel, 
                tg_v, 
                tg_a)
        
        # Times (Absolute and intervals)
        knots = np.array([0, t_end]) # One second each piece

        # Polynomial characteristic:  order
        ndeg = 7

        ppx = pw.PwPoly(X, knots, ndeg)
        ppy = pw.PwPoly(Y, knots, ndeg)
        ppz = pw.PwPoly(Z, knots, ndeg)
        ppw = pw.PwPoly(W, knots, ndeg) 
        traj_obj = trj.Trajectory(ppx, ppy, ppz, ppw)


        # Update the mission object
        t_start = rospy.get_time()
        miss = Mission()
        miss.update(
                p = start_pos,
                v = np.zeros(3), 
                tg_p = tg_p, 
                tg_v = tg_v, 
                tg_a = tg_a,
                trj_gen = traj_obj,
                start_time = t_start,
                stop_time = t_end + t_start,
                ttype = TrajectoryType.Takeoff
                )
        self.mission_queue.update(miss)

        return True


    ## =================================================================
    ###### SERVICES
    ## =================================================================
    # Service handler: Generation of generic trajectory piece
    def handle_genTrackTrj(self, req):
        """
        Generate a tracking trajectory to reach an absolute waypoint 
        with a given velocity and acceleration.
        """
        start_pos = posFromOdomMsg(self.current_odometry)
        start_vel = velFromOdomMsg(self.current_odometry)
        
        t_end = req.tg_time
        
        tg_v = req.target_v
        tg_a = req.target_a

        # A little logic considering what could have been requested
        if (req.ref == "Absolute"):
            tg_p = req.target_p
            tg_prel = tg_p - start_pos
        elif (req.ref == "Relative"):
            tg_prel = req.target_p
            # Detect if you are requesting a landing
            if ((start_pos[2] + tg_prel[2]) <= 0):
                # I have to redefine because I cannot mutate tuples
                tg_prel = [tg_prel[0], tg_prel[1], -start_pos[2]]
                # Automatically calculate the time to land, if not specified
                if (t_end == 0.0):
                    t_end = (start_pos[2]/0.3)
                tg_v = np.zeros((3))
                tg_a = np.zeros((3))
        else:
            rospy.loginfo("Error passing reference to guidance")
                 
        rospy.loginfo("Target Relative = [%.3f, %.3f, %.3f]" % (tg_prel[0], tg_prel[1], tg_prel[2]))

        (X, Y, Z, W) = genInterpolationMatrices(start_vel, tg_prel, tg_v, tg_a)
        
        # Times (Absolute and intervals)
        knots = np.array([0, t_end]) # One second each piece

        # Polynomial characteristic:  order
        ndeg = 7

        ppx = pw.PwPoly(X, knots, ndeg)
        ppy = pw.PwPoly(Y, knots, ndeg)
        ppz = pw.PwPoly(Z, knots, ndeg)
        ppw = pw.PwPoly(W, knots, ndeg) 
        traj_obj = trj.Trajectory(ppx, ppy, ppz, ppw)

        print("Final Relative Position: [%.3f, %.3f, %.3f]"%(X[0,1], Y[0,1], Z[0,1]))
        print("Solution: [%.3f, %.3f, %.3f]"%(ppx.eval(t_end, 0), ppy.eval(t_end, 0), ppz.eval(t_end, 0)))

        # Update the mission object
        t_start = rospy.get_time()
        miss = Mission()
        miss.update(
                p = start_pos,
                v = start_vel, 
                tg_p = tg_prel, 
                tg_v = tg_v, 
                tg_a = tg_a,
                trj_gen = traj_obj,
                start_time = t_start,
                stop_time = t_end + t_start
                )
        self.mission_queue.update(miss)

        return True 
 

    def handle_genTrjAuto(self, req):
        """
        Generate a impact trajectory just specifying the modulus of the 
        speed and the time-to-go to reach the target.
        """
        ndeg = 17
        a_norm = req.a_norm
        v_norm = req.v_norm

        # Take the current pose of the vehicle
        start_pos = posFromOdomMsg(self.current_odometry)
        start_vel = velFromOdomMsg(self.current_odometry)
        start_orientation = quatFromOdomMsg(self.current_odometry)
        start_yaw = quat2yaw(start_orientation)

        # Take the current target pose
        tg_pos = posFromPoseMsg(self.current_target)
        tg_q = quatFromPoseMsg(self.current_target) 
        tg_yaw = quat2yaw(tg_q)

        rospy.loginfo("\n\n\nOn Target in " + str(req.t2go) + " sec!")
        rospy.loginfo("Target = [" + str(tg_pos[0]) + " " +
                str(tg_pos[1]) + " " + str(tg_pos[2]) + "]")
        rospy.loginfo("Vehicle = [" + str(start_pos[0]) + " " +
                str(start_pos[1]) + " " + str(start_pos[2]) + "]")

        T = req.t2go
        DT = req.tbal
        
        # Compute the velocity and acceleration on the target
        # (Want to get there with 0 acceleration)
        (tg_v, _) = computeTerminalNormalVel(tg_q, v_norm)

        tg_v[2] = -2.0
        
        # Generate the entry poin to the final part of the trajectory 
        # This part is balistic because we are supposed to control just
        # in attitude.
        (tg_pre, tg_vpre, tg_apre) = computeTerminalTrj(
                tg_pos, 
                tg_v, 
                np.array([0, 0, -9.81]), 
                DT) 
        
        # Generate the interpolation matrices to reach the pre-impact point
        # This service produce a trajectory to reach the point with a
        # constant speed.
        mission_element = gen_MissionAuto(ndeg, start_vel, 
                start_pos, 
                tg_pre,# + np.array([0,0,-0.10]), 
                tg_vpre, 
                np.zeros(3, dtype=float), 
                T)

        t_end = mission_element.t_stop 
        # Reset
        self.mission_queue.update(mission_element)
        
        mission_element = gen_MissionAtt(tg_q, tg_pre, tg_vpre, tg_pos, tg_v, np.array([0, 0, -9.81]), t_end, DT * 5)   
        self.mission_queue.insertItem(mission_element)
        self.mission_queue.insertItem(Mission()) # Insert empty mission element to stop

        self.Active = True
        return True


    def handle_genImpTrjAuto(self, req):
        """
        Generate a impact trajectory just specifying the modulus of the 
        acceleration, speed and the time to go.
        """
        ndeg = 8
        Tz_norm = req.a_norm
        v_norm = req.v_norm

        # Take the current pose of the vehicle
        start_pos = posFromOdomMsg(self.current_odometry)
        start_vel = velFromOdomMsg(self.current_odometry)
        start_orientation = quatFromOdomMsg(self.current_odometry)
        start_yaw = quat2yaw(start_orientation)

        start_vel = np.array([0,0,0])
        # Take the current target pose
        tg_pos = posFromPoseMsg(self.current_target)
        tg_q = quatFromPoseMsg(self.current_target) 
        tg_yaw = quat2yaw(tg_q)

        rospy.loginfo("\n\n\nOn Target in " + str(req.t2go) + " sec!")
        rospy.loginfo("Target = [" + str(tg_pos[0]) + " " +
                str(tg_pos[1]) + " " + str(tg_pos[2]) + "]")
        rospy.loginfo("Vehicle = [" + str(start_pos[0]) + " " +
                str(start_pos[1]) + " " + str(start_pos[2]) + "]")

        T = req.t2go
        DT = req.tbal

        # Compute the velocity and acceleration on the target considering
        # a Tz_norm acceleration along the Zb
        (tg_v, tg_a) = computeTerminalNormalVelAcc(tg_q, v_norm, Tz_norm)

        tg_v[2] = -2.0 
 
        (tg_pre, tg_vpre, tg_apre) = computeTerminalTrj(
                tg_pos,
                tg_v,
                tg_a,
                DT) 
        
        mission_element = gen_MissionAuto(ndeg, start_vel,
                start_pos,
                tg_pre + np.array([0,0,-0.10]),
                tg_vpre,
                tg_apre,
                T)
        
        t_stop = mission_element.t_stop 

        # Reset the current mission queue
        self.mission_queue.update(mission_element)

        mission_element = gen_MissionAtt(tg_q, tg_pre, tg_vpre, tg_pos, tg_v, tg_a, t_stop, DT * 5)   
        self.mission_queue.insertItem(mission_element)
        self.mission_queue.insertItem(Mission())

        self.Active = True
        return True

    def gen_flip(self, duration):
        """
        Generate a tracking trajectory to reach an absolute waypoint 
        with a given velocity and acceleration.
        """
        start_pos = posFromOdomMsg(self.current_odometry)
        start_vel = velFromOdomMsg(self.current_odometry)

        # Generate a trajectory to start the flip with a given 
        # upwards velocity.
        tg_p = start_pos + np.array([0, 0, 1.0])
        tg_prel = tg_p - start_pos
        tg_v = np.array([0, 0, 2.0])
        tg_a = np.zeros((3))
                 
        goto_vel = 1.0 
        (X, Y, Z, W) = genInterpolationMatrices(start_vel, tg_prel, tg_v, tg_a)
        
        DT = np.linalg.norm(tg_prel) / goto_vel 

        # Times (Absolute and intervals)
        knots = np.array([0, DT]) # One second each piece
        # Polynomial characteristic:  order
        ndeg = 7
        ppx = pw.PwPoly(X, knots, ndeg)
        ppy = pw.PwPoly(Y, knots, ndeg)
        ppz = pw.PwPoly(Z, knots, ndeg)
        ppw = pw.PwPoly(W, knots, ndeg) 
        traj_obj = trj.Trajectory(ppx, ppy, ppz, ppw)

        # Update the mission object
        t_start = rospy.get_time()
        t_end = t_start + DT
        miss_element = Mission()
        miss_element.update(
                p = start_pos,
                v = start_vel, 
                tg_p = tg_p, 
                tg_v = tg_v, 
                tg_a = tg_a,
                trj_gen = traj_obj,
                start_time = t_start,
                stop_time = t_end, 
                ttype = TrajectoryType.FullTrj
                )
        self.mission_queue.update(miss_element)
        
        # Tilt the drone: the motion is free falling (balistic)
        tg_a = np.array([0, 0, -9.81])
        endTrj = ConstAttitudeTrj(tg_p, tg_v, tg_a, (45.0 / 180.0) * math.pi, 0.0, 0.0) 
        mission = Mission()
        t_start = t_end 
        t_end = t_start + duration

        (X, Y, Z, self.r, self.p, self.y) = endTrj.eval(duration)
        p_end = np.array([X[0], Y[0], Z[0]])
        v_end = np.array([X[1], Y[1], Z[1]])
        a_end = np.array([X[2], Y[2], Z[2]])
        mission.update(
                    p = tg_p,
                    v = tg_v, 
                    tg_p = p_end,
                    tg_v = v_end,
                    tg_a = a_end,
                    trj_gen = endTrj,
                    start_time = t_start,
                    stop_time = t_end, 
                    ttype = TrajectoryType.AttTrj)

        self.mission_queue.insertItem(mission)

        # Generate the recovery trajectory. 
        RecTime = 0.5
        (X, Y, Z, W) = genInterpolationMatrices(v_end, p_end + v_end * RecTime, np.zeros(3), np.zeros(3))
        
        # Times (Absolute and intervals)
        knots = np.array([0, RecTime]) # One second each piece

        # Polynomial characteristic:  order
        ndeg = 7

        ppx = pw.PwPoly(X, knots, ndeg)
        ppy = pw.PwPoly(Y, knots, ndeg)
        ppz = pw.PwPoly(Z, knots, ndeg)
        ppw = pw.PwPoly(W, knots, ndeg) 
        traj_obj = trj.Trajectory(ppx, ppy, ppz, ppw)

        # Update the mission object
        t_start = t_end 
        t_end = t_start + RecTime 
        miss = Mission()
        miss.update(
                p = p_end,
                v = v_end, 
                tg_p = p_end + v_end * RecTime, 
                tg_v = np.zeros(3), 
                tg_a = np.zeros(3),
                trj_gen = traj_obj,
                start_time = t_start,
                stop_time = t_end, 
                ttype = TrajectoryType.FullTrj
                )
        self.mission_queue.insertItem(miss)


        return True 


    def handle_exeMission(self, req):
        # Dispach the mission requests

        t2go = req.tg_time   
        tg_p = req.target_p
        tg_v = req.target_v
        tg_a = req.target_a


        if (req.mission_type == "goTo"):
            #self.gen_goTo(tg_p, t2go)        
            self.gen_goToBZ(tg_p, t2go)
 
        if (req.mission_type == "land"): 
            self.gen_land()

        if (req.mission_type == "takeoff"):
            h = tg_p[2]
            self.gen_takeoff(h, t2go)

        if (req.mission_type == "impact"):
            pass

        if (req.mission_type == "impact_acc"):
            self.gen_impact()

        if (req.mission_type == "flip"):
            self.gen_flip(t2go)

        self.Active = True
        return True


################# END OF GUIDANCE CLASS #################
