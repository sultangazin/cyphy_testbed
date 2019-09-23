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

import trjgen.class_pwpoly as pw
import trjgen.class_trajectory as trj
import trjgen.trjgen_helpers as trjh
import trjgen.trjgen_core as tj

import trjgen.class_bz as bz
import trjgen.class_bztraj as bz_t

from guidance_helper import *
from guidance_helper_classes.mission_class import Mission, MissionType, TrajectoryType
from guidance_helper_classes.trajectorygenerator_class import TrajectoryGenerator



class MissionQueue:
    def __init__(self):
        self.missionList = []
        self.numItems = 0

    def insertItem(self, MissionItem):
        self.missionList.append(MissionItem)
        self.numItems = self.numItems + 1

    def reset(self):
        self.missionList = []
        self.numItems = 0

    def update(self, MissionItem):
        self.reset()
        self.insertItem(MissionItem)

    def getItemAtTime(self, t):
        if (self.numItems == 0):
            print("Mission queue is empty")
            return None

        for i in range(self.numItems):
            if self.missionList[i].t_stop < t:
                return self.missionList[i]

        print("All the mission item are obsolete")
        return None




################# GUIDANCE CLASS #####################
class GuidanceClass:

    def __init__(self):

        self.initData()

        self.loadParameters()

        self.registerCallbacks()

        self.registerServices()

        self.advertiseTopics()

        rospy.loginfo("\n [%s] Guidance Initialized!"%rospy.get_name()) 


    def initData(self):
        self.current_odometry = CustOdometryStamped()
        self.current_target = PoseStamped()

        # Mission object to store information about
        # the current status of the plan.
        self.current_mission = Mission()

        self.MissionList = []
        
        self.StopUpdating = False 


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

        current_mission = self.MissionQueue.getItemAtTime(t)


        output_msg = ControlSetpoint()
        output_msg.header.stamp = t

        # Fill the output message for the controller differently depending on 
        # the type of mission that is going to be requested:
        if current_mission == None:
            self.StopUpdating = True
            (keep_pos, _, _) = self.current_mission.getEnd() 
            output_msg.p.x = keep_pos[0]
            output_msg.p.y = keep_pos[1]
            output_msg.p.z = keep_pos[2]
            
            output_msg.v.x = 0.0 
            output_msg.v.y = 0.0
            output_msg.v.z = 0.0
            output_msg.a.x = 0.0
            output_msg.a.y = 0.0
            output_msg.a.z = 0.0
        else:
            self.StopUpdating = False   
            miss_type = current_mission.getType()

            if miss_type != TrajectoryType.AttTrj:
                (X, Y, Z, W, R, Omega) = current_mission.getRef(t)
            else:
                (roll, pitch, yaw) = current_mission.getRef(t)

            output_msg.setpoint_type = miss_type 
             
            # Fill  the trajectory object
            if (miss_type != TrajectoryType.AttTrj):
                output_msg.p.x = X[0]
                output_msg.p.y = Y[0]
                output_msg.p.z = Z[0]

            if (miss_type == TrajectoryType.FullTrj):
                output_msg.v.x = X[1] 
                output_msg.v.y = Y[1]
                output_msg.v.z = Z[1]
                output_msg.a.x = X[2]
                output_msg.a.y = Y[2]
                output_msg.a.z = Z[2]

            if (miss_type == TrajectoryType.FullTrj):
                # Conver the Rotation matrix to euler angles
                (roll, pitch, yaw) = euler_from_matrix(R)
            
                output_msg.rpy.x = roll
                output_msg.rpy.y = pitch
                output_msg.rpy.z = yaw
                output_msg.brates.x = Omega[0]
                output_msg.brates.y = Omega[1]
                output_msg.brates.z = Omega[2]

            if (miss_type == TrajectoryType.AttTrj):
                output_msg.rpy.x = roll
                output_msg.rpy.y = pitch
                output_msg.rpy.z = yaw            

        # Pubblish the evaluated trajectory
        if (not self.StopUpdating):
            self.ctrl_setpoint_pub.publish(output_msg)
        return


    # On new target information
    def tg_callback(self, pose_msg):
        self.current_target = pose_msg
        return


    ###### SERVICES
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
        current_mission = Mission()

        current_mission.update(
                p = start_pos,
                v = start_vel, 
                tg_p = tg_prel, 
                tg_v = tg_v, 
                tg_a = tg_a,
                trj_gen = traj_obj,
                start_time = t_start,
                stop_time = np.array([t_end + t_start])
                )

        self.MissionList.update(current_mission)

        return True 

    def handle_genImpTrjAuto(self, req):
        """
        Generate a impact trajectory just specifying the modulus of the 
        acceleration, speed and the time to go.
        """
        ndeg = 13
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

        tg_z = quat2Z(tg_q)
        rospy.loginfo("\nOn Target in " + str(req.t2go) + " sec!")
        rospy.loginfo("Target = [" + str(tg_pos[0]) + " " +
                str(tg_pos[1]) + " " + str(tg_pos[2]) + "]")
        rospy.loginfo("Target Z = [" + str(tg_z[0]) + " " +
                str(tg_z[1]) + " " + str(tg_z[2]) + "]")
        rospy.loginfo("Vehicle = [" + str(start_pos[0]) + " " +
                str(start_pos[1]) + " " + str(start_pos[2]) + "]")

        T = req.t2go
        DT = 0.5 
        (tg_pre, tg_v, tg_a) = computeTerminalTrjStart(tg_pos, tg_q, v_norm, a_norm, DT) 
     
        rospy.loginfo("Target Vel= [" + str(tg_v[0]) + " " +
                str(tg_v[1]) + " " + str(tg_v[2]) + "]")

        rospy.loginfo("Target Acc= [" + str(tg_a[0]) + " " +
                str(tg_a[1]) + " " + str(tg_a[2]) + "]")

        # Generate the interpolation matrices to reach the pre-impact point
        (Xwp, Ywp, Zwp, Wwp) = genInterpolationMatrices(start_vel, tg_pos - start_pos, tg_v, tg_a)

        # Generation of the trajectory with Bezier curves
        x_lim = [3.0, 5.5, 11.0]

        y_lim = [3.0, 5.5, 11.0]
        z_lim = [2.4, 2.5, 14.0]

        x_cnstr = np.array([[-x_lim[0], x_lim[0]], [-x_lim[1], x_lim[1]], [-x_lim[2], x_lim[2]]])
        y_cnstr = np.array([[-y_lim[0], y_lim[0]], [-y_lim[1], y_lim[1]], [-y_lim[2], y_lim[2]]])
        z_cnstr = np.array([[-start_pos[2], z_lim[0]], [-z_lim[1], z_lim[1]], [-9.81, z_lim[2]]])

        # Generate the polynomial
        #
        print("Generating X")
        bz_x = bz.Bezier(waypoints=Xwp, constraints=x_cnstr, degree=ndeg, s=T)
        print("\nGenerating Y")
        bz_y = bz.Bezier(waypoints=Ywp, constraints=y_cnstr, degree=ndeg, s=T)
        print("\nGenerating Z")
        bz_z = bz.Bezier(waypoints=Zwp, constraints=z_cnstr, degree=ndeg, s=T)
        print("\nGenerating W")
        bz_w = bz.Bezier(waypoints=Wwp, degree=7, s=T)

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
                    p = start_pos,
                    v = start_vel, 
                    tg_p = tg_pos, 
                    tg_v = tg_v, 
                    tg_a = tg_a,
                    trj_gen = trj_obj,
                    start_time = t_start,
                    stop_time = t_end, 
                    mtype = MissionType.Composite)

        self.MissionList.update(mission)

        endTrj = ConstAttitudeTrj()
        endTrj.set()

        mission.update(
                    p = start_pos,
                    v = start_vel, 
                    tg_p = tg_pos, 
                    tg_v = tg_v, 
                    tg_a = tg_a,
                    trj_gen = trj_obj,
                    start_time = t_start,
                    stop_time = t_end, 
                    mtype = MissionType.Composite)

        return True

################# END OF GUIDANCE CLASS #################
