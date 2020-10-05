# Guidance class
import os
import sys
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '../trjgen')))
import numpy as np
from threading import Thread

import rospy

from tf.transformations import euler_from_matrix

from testbed_msgs.msg import CustOdometryStamped
from testbed_msgs.msg import ControlSetpoint 
from testbed_msgs.msg import BZCurve
from geometry_msgs.msg import PoseStamped

from guidance.srv import GenImpTrajectoryAuto
from guidance.srv import GenTrackTrajectory

import trjgen.class_pwpoly as pw
import trjgen.class_trajectory as trj

import trjgen.class_bz as bz
import trjgen.class_bztraj as bz_t

from guidance_helper import *

import actionlib
from guidance.msg import GuidanceTargetAction, GuidanceTargetFeedback, GuidanceTargetResult



################# GUIDANCE CLASS #####################
class GuidanceClass:

    def __init__(self):

        self.initData()

        self.loadParameters()

        self.advertiseTopics()

        self.registerCallbacks()

        self.registerServices()

        self.action_server.start()

        rospy.loginfo("\n [%s] Guidance Initialized!"%rospy.get_name()) 


    def initData(self):
        self.current_odometry = CustOdometryStamped()

        
    def loadParameters(self):
        # Load the name of the Output Topics 
        self.ctrlsetpoint_topic_ = rospy.get_param('topics/out_ctrl_setpoint', "setpoint")

        # Load the name of the Input Topics
        self.dr_odom_topic_ = rospy.get_param('topics/in_vehicle_odom_topic', 'external_codom')    

    def registerCallbacks(self):
        # Subscribe to vehicle state update
        rospy.Subscriber(self.dr_odom_topic_, CustOdometryStamped, self.odom_callback)


    def registerServices(self):
        # Services 
        self.action_server = actionlib.SimpleActionServer('AAAA',
                GuidanceTargetAction, self.handle_guidance_action, False)


    def advertiseTopics(self):
        # Setpoint Publisher
        self.ctrl_setpoint_pub = rospy.Publisher(
                self.ctrlsetpoint_topic_, ControlSetpoint, queue_size=10)
 

    ###### CALLBACKS
    # On new vehicle information
    def odom_callback(self, odometry_msg):
        # Update the pose/twist information of the controlled vehicle
        self.current_odometry = odometry_msg
        return



    ## ===================================================================
    #####                     Guidance Methods
    ## ===================================================================
    def gen_goToBZ(self, p, t2go = 0.0):
        """
        Generate a tracking trajectory to reach an absolute waypoint 
        with a given velocity and acceleration.
        """
        start_pos = posFromOdomMsg(self.current_odometry)
        start_vel = velFromOdomMsg(self.current_odometry)

        # Go to the point and stop there
        tg_p = p 
        tg_prel = tg_p - start_pos
        tg_v = np.zeros((3))
        tg_a = np.zeros((3))
                 
        ndeg = 5
        rospy.loginfo("Start Point = [%.3f, %.3f, %.3f]" % (start_pos[0], start_pos[1], start_pos[2]))
        rospy.loginfo("End Point = [%.3f, %.3f, %.3f]" % (tg_p[0], tg_p[1], tg_p[2]))

        # Check whether there is an intersection with the obstacle.
        if (t2go == 0.0):
            goto_vel = 0.5
            T = np.linalg.norm(tg_prel) / goto_vel
        else:
            T = t2go

        (trj_obj, Tend) = self.gen_MissionAuto(
                ndeg, start_vel, 
                start_pos, 
                tg_p,
                tg_v, 
                np.zeros(3, dtype=float), 
                T)

        return (trj_obj, Tend) 


    def gen_land(self, p = None):
        """
        Generate a landing trajectory
        """
        start_pos = posFromOdomMsg(self.current_odometry)
        start_vel = velFromOdomMsg(self.current_odometry)

        vland = 0.3

        if (p is not None):
            tg_prel = np.array([p[0] - start_pos[0], p[1] - start_pos[1], -start_pos[2]])
            tg_p = np.copy(p)
            tg_p[2] = 0.0
            print("Landing in [{}, {}]\n".format(p[0], p[1]))
        else:
            tg_prel = np.array([0.0, 0.0, -start_pos[2]])
            tg_p = np.copy(start_pos)
            tg_p[2] = 0.0
            print("Landing in [{}, {}]\n".format(start_pos[0], start_pos[1]))

        t_end = (start_pos[2] / vland)
        tg_v = np.zeros((3))
        tg_a = np.zeros((3))

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

        return (traj_obj, t_end)
 


    def gen_takeoff(self, h, t2go=None):
        """
        Generate a tracking trajectory to reach an absolute waypoint 
        with a given velocity and acceleration.
        """
        start_pos = posFromOdomMsg(self.current_odometry)

        if (start_pos[2] > 0.4):
            rospy.loginfo("Already Flying!")
            return (None, 0) 
                
        vtakeoff = 0.3
        t_end = h / vtakeoff

        if (t2go is not None and t2go != 0.0):
            t_end = t2go
            vtakeoff = h / t2go

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

        return (traj_obj, t_end)


    ## =================================================================
    ###### SERVICES
    ## =================================================================
    def gen_MissionAuto(self, ndeg, v0, x0, xf, vf, af, T): 
        # Wrapper for the trajectory generation machinery

        (Xwp, Ywp, Zwp, Wwp) = genInterpolationMatrices(v0, xf - x0, vf, af)

        delta = (xf - x0) * 1.1
        # Generation of the trajectory with Bezier curves
        x_lim = [5.5, 15.5, 9.0]
        y_lim = [5.5, 15.5, 9.0]
        z_lim = [2.4, 2.3, 5.0]

        x_cnstr = np.array([
            #[min(delta[0], -0.2), max(delta[0], 0)],
            [-x_lim[0], x_lim[0]],
            [-x_lim[1], x_lim[1]],
            [-x_lim[2], x_lim[2]]])
        y_cnstr = np.array([
            #[min(delta[1], -0,2), max(delta[1], 0)],
            [-y_lim[0], y_lim[0]], 
            [-y_lim[1], y_lim[1]], 
            [-y_lim[2], y_lim[2]]])
        z_cnstr = np.array([
            [-z_lim[1], z_lim[1]],
            [-9.90, z_lim[2]]])
        

        # Generate the polynomial
        #print("\nGenerating X")
        bz_x = bz.Bezier(waypoints=Xwp, constraints=x_cnstr, degree=ndeg, s=T, opt_der=3)
        #print("\nGenerating Y")
        bz_y = bz.Bezier(waypoints=Ywp, constraints=y_cnstr, degree=ndeg, s=T, opt_der=3)
        #print("\nGenerating Z")
        bz_z = bz.Bezier(waypoints=Zwp, constraints=z_cnstr, degree=ndeg, s=T, opt_der=3)
        #print("\nGenerating W")
        bz_w = bz.Bezier(waypoints=Wwp, degree=ndeg, s=T, opt_der=0)

        
        print("Final Relative Position: [%.3f, %.3f, %.3f]"%(Xwp[0,1], Ywp[0,1], Zwp[0,1]))
        print("Final Velocity: [%.3f, %.3f, %.3f]"%(Xwp[1,1], Ywp[1,1], Zwp[1,1]))
        print("Final Acceleration: [%.3f, %.3f, %.3f]"%(Xwp[2,1], Ywp[2,1], Zwp[2,1]))

        print("Solution Position: [%.3f, %.3f, %.3f]"%(bz_x.eval(T), bz_y.eval(T), bz_z.eval(T)))
        print("Solution Velocity: [%.3f, %.3f, %.3f]"%
                (bz_x.eval(T, [1]), bz_y.eval(T, [1]), bz_z.eval(T, [1])))
        print("Solution Acceleration: [%.3f, %.3f, %.3f]\n"%(bz_x.eval(T, [2]), bz_y.eval(T, [2]), bz_z.eval(T, [2])))

        trj_obj = bz_t.BezierCurve(bz_x, bz_y, bz_z, bz_w)
      
        # Compute the absolute times for this trajectory
        t_start = rospy.get_time()
        t_end = t_start + T

        return (trj_obj, t_end)


    def handle_guidance_action(self, goal):

        start_point = posFromOdomMsg(self.current_odometry)
        frequency = 200;

        trajectory = None

        t2go = goal.tg_time
        tg_p = goal.target_p
        tg_v = goal.target_v
        tg_a = goal.target_a


        # If the goal is relative compute the absolute before requesting 
        # the trajectory: the functions think in absolute position
        if (goal.relative):
            tg_p = tg_p + start_point

        # Call the specific trajectory generation
        if (goal.mission_type == "goTo"):
            (trajectory, Tend) = self.gen_goToBZ(tg_p, t2go)

        if (goal.mission_type == "land"): 
            (trajectory, Tend) = self.gen_land(tg_p)

        if (goal.mission_type == "takeoff"):
            h = tg_p[2]
            (trajectory, Tend) = self.gen_takeoff(h, t2go)


        if (trajectory is not None):
            self.action_thread(goal.mission_type, trajectory, start_point, frequency)
        else:
            print("Trajectory not generated!")
       # 
       # if (req.mission_type == "impact"):
       #     (trjectory, Tend) = self.handle_genImpTrjDet
       #     t = Thread(target = action_thread, args = (trajectory, start_point, frequency)).start()

        return True
    
    def action_thread(self, trj_type, trajectory, start_position, freq):
        timeSpan = trajectory.get_duration(); 

        r = rospy.Rate(freq)

        start_time = rospy.get_time() 
        end_time = start_time + trajectory.get_duration()

        curr_time = start_time
        #print("Start time: ", start_time)
        #print("Expected end time: ", end_time)

        result = GuidanceTargetResult()

        X = np.zeros(4)
        Y = np.zeros(4)
        Z = np.zeros(4)
        W = np.zeros(4)
        R = np.eye(3)
        Omega = np.zeros((3,3))

        # Publishing Loop
        while (curr_time < end_time):
            # Check for preemptions
            if (self.action_server.is_preempt_requested()):
                result = GuidanceTargetResult()
                result.ret_status = 1
                result.mission_type = trj_type
                print("Action Server preempted!")
                self.action_server.set_preempted(result, "Action Preempted")
                return 


            if (self.action_server.is_new_goal_available()):
                result = GuidanceTargetResult()
                print("Action Server preempted!")
                result.mission_type = trj_type
                result.ret_status = 1
                print("Action Server serving new goal!")
                self.action_server.accept_new_goal()
                return

            output_msg = ControlSetpoint()
            output_msg.setpoint_type = "active"
            output_msg.header.stamp = rospy.Time.now()

            # Evaluate the trajectory
            trj_time = curr_time - start_time
            (X, Y, Z, W, R, Omega) = trajectory.eval(trj_time, [0,1,2,3]) 
             
            # Fill  the trajectory object
            output_msg.p.x = start_position[0] + X[0]
            output_msg.p.y = start_position[1] + Y[0]
            output_msg.p.z = start_position[2] + Z[0]

            output_msg.v.x = X[1] 
            output_msg.v.y = Y[1]
            output_msg.v.z = Z[1]

            output_msg.a.x = X[2]
            output_msg.a.y = Y[2]
            output_msg.a.z = Z[2]
            
            # Action Feedback
            fb_data = GuidanceTargetFeedback()
            fb_data.curr_pos = [output_msg.p.x, output_msg.p.y, output_msg.p.z]
            fb_data.curr_status = 1;
            self.action_server.publish_feedback(fb_data)

            # Pubblish the evaluated trajectory
            self.ctrl_setpoint_pub.publish(output_msg)

            # Wait the next loop
            r.sleep()
            # Take the time
            curr_time = rospy.get_time()

        # End Setpoint
        output_msg = ControlSetpoint()
        output_msg.header.stamp = rospy.Time.now()

        if (trj_type != "land"):
            output_msg.setpoint_type = "active"
            output_msg.p.x = start_position[0] + X[0]
            output_msg.p.y = start_position[1] + Y[0]
            output_msg.p.z = start_position[2] + Z[0]
        else:
            output_msg.setpoint_type = "stop"

        self.ctrl_setpoint_pub.publish(output_msg)

        result = GuidanceTargetResult()
        result.ret_status = 0;
        result.mission_type = trj_type
        self.action_server.set_succeeded(result, "Mission completed succesfully")


################# END OF GUIDANCE CLASS #################
