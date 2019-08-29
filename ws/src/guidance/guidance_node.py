#!/usr/bin/env python

# This is the guidance node 
# It genereates the trajectory to reach a target point using 
# polynomials.
import numpy as np
import math

import rospy
import time
from threading import Thread
from tf.transformations import euler_from_matrix
from nav_msgs.msg import Odometry
from testbed_msgs.msg import ControlSetpoint 
from geometry_msgs.msg import PoseStamped

from guidance.srv import GenGoToTrajectory
from guidance.srv import GenImpTrajectoryAuto
from guidance.srv import GenTrackTrajectory

import trjgen.class_pwpoly as pw
import trjgen.class_trajectory as trj
import trjgen.trjgen_helpers as trjh
import trjgen.trjgen_core as tj

from guidance_helper import *

current_odometry = Odometry()
current_target = PoseStamped()


def odom_callback(odometry_msg):
    global current_odometry 
    current_odometry = odometry_msg
    return

def tg_callback(pose_msg):
    global current_target
    current_target = pose_msg
    return


def handle_genImpTrjAuto(req):
    """
    Generate a impact trajectory just specifying the modulus of the 
    acceleration, speed and the time to go.
    
    It uses a 7 degree polynomial, such that the trajectory is also
    uploadable on the vehicle for onboard tracking.
    """
    ndeg = 12
    a_norm = req.a_norm
    v_norm = req.v_norm

    # Take the current pose of the vehicle
    start_pos = np.array([current_odometry.pose.pose.position.x, 
            current_odometry.pose.pose.position.y, 
            current_odometry.pose.pose.position.z])
    start_orientation = np.array([current_odometry.pose.pose.orientation.w,
        current_odometry.pose.pose.orientation.x, 
        current_odometry.pose.pose.orientation.y,
        current_odometry.pose.pose.orientation.z]
    )
    start_yaw = quat2yaw(start_orientation)

    # Take the current target pose
    tg_pos = np.array([current_target.pose.position.x,
            current_target.pose.position.y,
            current_target.pose.position.z])

    tg_q = np.array([current_target.pose.orientation.w,
        current_target.pose.orientation.x,
        current_target.pose.orientation.y,
        current_target.pose.orientation.z]
        ) 
    tg_yaw = quat2yaw(tg_q)

    rospy.loginfo("\nOn Target in " + str(req.t2go) + " sec!")
    rospy.loginfo("Target = [" + str(tg_pos[0]) + " " +
            str(tg_pos[1]) + " " + str(tg_pos[2]) + "]")
    rospy.loginfo("Vehicle = [" + str(start_pos[0]) + " " +
            str(start_pos[1]) + " " + str(start_pos[2]) + "]")

    # =================================================================
    # Time to go
    t_impact = req.t2go
    t_impact = np.linalg.norm(tg_pos)/0.8

    DT = 0.1 
    (p_pre, v_pre, a_dem) = computeTerminalTrjStart(tg_pos, tg_q, v_norm, a_norm, DT) 
 
    # Generate the interpolation matrices to reach the pre-impact point
    (X, Y, Z, W, knots) = genInterpolProblem(p_pre - start_pos, v_pre, a_dem, tg_yaw - start_yaw, t_impact)
    
    # Generate the polynomials
    ppx = pw.PwPoly(X, knots, ndeg)
    ppy = pw.PwPoly(Y, knots, ndeg)
    ppz = pw.PwPoly(Z, knots, ndeg)
    ppw = pw.PwPoly(W, knots, ndeg)

    frequency = rospy.get_param('~freq', 30.0);

    my_traj = trj.Trajectory(ppx, ppy, ppz, ppw)

    Dt = knots[1:len(knots)] - knots[0:len(knots)-1]

    my_traj.writeTofile(Dt, '/tmp/toTarget.csv')

    t = Thread(target=rep_ImpactTrajectory, args=(my_traj, start_pos, max(knots), DT, frequency)).start()

    return True



# Generate a tracking trajectory to reach an absolute waypoint 
# with a given velocity and acceleration.
def handle_genTrackTrj(req):
 
    start_pos = np.array([current_odometry.pose.pose.position.x, 
            current_odometry.pose.pose.position.y, 
            current_odometry.pose.pose.position.z])

    start_vel = np.array([current_odometry.twist.twist.linear.x, 
            current_odometry.twist.twist.linear.y, 
            current_odometry.twist.twist.linear.z])

    t_impact = req.tg_time
    
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
            if (t_impact == 0.0):
                t_impact = (start_pos[2]/0.3)
            tg_v = np.zeros((3))
            tg_a = np.zeros((3))
    else:
        rospy.loginfo("Error passing reference to guidance")
            
    # Times (Absolute and intervals)
    knots = np.array([0, t_impact]) # One second each piece
    Dt = knots[1:len(knots)] - knots[0:len(knots) - 1]

    # Polynomial characteristic:  order
    ndeg = 7
    nconstr = 4

    X = np.array([
        [ 0.0,          tg_prel[0]],
        [ start_vel[0], tg_v[0]],
        [ 0.0,          tg_a[0]],
        [ 0.0,          0.0],
        ])

    Y = np.array([
        [ 0.0,          tg_prel[1]],
        [ start_vel[1], tg_v[1]],
        [ 0.0,          tg_a[1]],
        [ 0.0,          0.0],
        ])
    
    Z = np.array([
        [ 0.0,          tg_prel[2]],
        [ start_vel[2], tg_v[2]],
        [ 0.0,          tg_a[2]],
        [ 0.0,          0.0],
        ])

    W = np.array([
        [ 0.0,    0.0],
        [ 0.0,    0.0],
        [ 0.0,    0.0],
        [ 0.0,    0.0],
        ])

    ppx = pw.PwPoly(X, knots, ndeg)
    ppy = pw.PwPoly(Y, knots, ndeg)
    ppz = pw.PwPoly(Z, knots, ndeg)
    ppw = pw.PwPoly(W, knots, ndeg)

    frequency = rospy.get_param('~freq', 30.0);

    my_traj = trj.Trajectory(ppx, ppy, ppz, ppw)

    my_traj.writeTofile(Dt, '/tmp/toTarget.csv')

    t = Thread(target=rep_trajectory, 
            args=(my_traj, start_pos, t_impact, frequency)).start()

    return True 


def rep_trajectory(traj, start_position, timeSpan, freq):
    global ctrl_setpoint_pub

    mass = 0.032;
    thr_lim = 9.81 * mass * 1.5

    r = rospy.Rate(freq)

    start_time = rospy.get_time() 
    curr_time = start_time
    end_time = start_time + timeSpan

    msg = ControlSetpoint()
    rtime = rospy.get_rostime()
    msg.header.stamp = rtime

    # Publishing Loop
    while (curr_time < end_time):
        # Evaluate the trajectory
        (X, Y, Z, W, R, Omega) = traj.eval((curr_time - start_time), [0, 1, 2, 3])

        msg.p.x = X[0] + start_position[0]
        msg.p.y = Y[0] + start_position[1]
        msg.p.z = Z[0] + start_position[2]

        msg.v.x = X[1] 
        msg.v.y = Y[1]
        msg.v.z = Z[1]

        msg.a.x = X[2]
        msg.a.y = Y[2]
        msg.a.z = Z[2]

        # Evaluate the thrust margin of the trjectory at the current time
        (ffthrust, available_thrust) = trjh.getlimits(X, Y, Z, mass, thr_lim)

        if (available_thrust < 0):
            rospy.loginfo("Exceding thrust limits!!")
            rospy.loginfo("\t" + str(available_thrust))

        # Conver the Rotation matrix to euler angles
        (roll, pitch, yaw) = euler_from_matrix(R)

        msg.rpy.x = roll
        msg.rpy.y = pitch
        msg.rpy.z = yaw

        msg.brates.x = Omega[0]
        msg.brates.y = Omega[1]
        msg.brates.z = Omega[2]
 
        # Pubblish the evaluated trajectory
        ctrl_setpoint_pub.publish(msg)

        # Wait the next loop
        r.sleep()
        # Take the time
        curr_time = rospy.get_time()


# Generate the terminal trajectory
def rep_ImpactTrajectory(traj, start_position, timeSpan, timeTerminal, freq):
    global ctrl_setpoint_pub

    mass = 0.032;
    thr_lim = 9.81 * mass * 1.5

    r = rospy.Rate(freq)

    start_time = rospy.get_time() 
    curr_time = start_time
    end_time = start_time + timeSpan

    msg = ControlSetpoint()
    rtime = rospy.get_rostime()
    msg.header.stamp = rtime

    # Publishing Loop
    while (curr_time < end_time):
        # Evaluate the trajectory
        (X, Y, Z, W, R, Omega) = traj.eval((curr_time - start_time), [0, 1, 2, 3])

        msg.p.x = X[0] + start_position[0]
        msg.p.y = Y[0] + start_position[1]
        msg.p.z = Z[0] + start_position[2]

        msg.v.x = X[1] 
        msg.v.y = Y[1]
        msg.v.z = Z[1]

        msg.a.x = X[2]
        msg.a.y = Y[2]
        msg.a.z = Z[2]

        # Evaluate the thrust margin of the trjectory at the current time
        (ffthrust, available_thrust) = trjh.getlimits(X, Y, Z, mass, thr_lim)

        if (available_thrust < 0):
            rospy.loginfo("Exceding thrust limits!!")
            rospy.loginfo("\t" + str(available_thrust))

        # Conver the Rotation matrix to euler angles
        (roll, pitch, yaw) = euler_from_matrix(R)

        msg.rpy.x = roll
        msg.rpy.y = pitch
        msg.rpy.z = yaw

        msg.brates.x = Omega[0]
        msg.brates.y = Omega[1]
        msg.brates.z = Omega[2]
 
        # Pubblish the evaluated trajectory
        ctrl_setpoint_pub.publish(msg)

        # Wait the next loop
        r.sleep()
        # Take the time
        curr_time = rospy.get_time()

    
    # Generate the terminal part of the trajectory
    p = np.array([X[0] + start_position[0], 
            Y[0] + start_position[1], 
            Z[0] + start_position[2]])
    p = np.reshape(p, (3,1))
    v = np.array([X[1], Y[1], Z[1]])
    v = np.reshape(v, (3,1))
    a = np.array([X[2], Y[2], Z[2]])
    a = np.reshape(a, (3,1))

    rospy.loginfo("Reached the pre-point: \n");
    rospy.loginfo(p)
    rospy.loginfo(v)
    rospy.loginfo(a)


    X = np.vstack((p,v))
    while (curr_time < end_time + timeTerminal):
        dt = 1/freq
        X = integrationStep(X, a, dt, 1)
        p = X[0:3]
        v = X[3:6]

        msg.p.x = p[0]
        msg.p.y = p[1]
        msg.p.z = p[2]

        msg.v.x = v[0] 
        msg.v.y = v[1]
        msg.v.z = v[2]

        msg.a.x = a[0]
        msg.a.y = a[1]
        msg.a.z = a[2]

        # Pubblish the evaluated trajectory
        ctrl_setpoint_pub.publish(msg)

        # Wait the next loop
        r.sleep()
        # Take the time
        curr_time = rospy.get_time()

    rospy.loginfo("Terminated")

         
if __name__ == '__main__':
    rospy.init_node('Guidance_Node')

    vehicle_mass = rospy.get_param('~vehicle_mass', 0.032)
    n_points = rospy.get_param('traject_points', 100) 

    commander_id = rospy.get_param('~command_id', 'cm1')
    target_frame = rospy.get_param('~target_frame', 'cf1')
    
    dr_odom_topic_ = rospy.get_param('topics/in_vehicle_odom_topic', 'external_odom')
    tg_pose_topic_ = rospy.get_param('topics/in_tg_pose_topic', "/vrpn_client_node/target/pose")

    ctrlsetpoint_topic_ = rospy.get_param('topics/out_ctrl_setpoint', "setpoint")

    service_track = rospy.Service('gen_TrackTrajectory', 
            GenTrackTrajectory, handle_genTrackTrj)

    service_imp_auto = rospy.Service('gen_ImpTrajectoryAuto', 
            GenImpTrajectoryAuto, handle_genImpTrjAuto)

    # Subscribe to vehicle state update
    rospy.Subscriber(dr_odom_topic_, Odometry, odom_callback)
    rospy.Subscriber(tg_pose_topic_, PoseStamped, tg_callback)

    # Setpoint Publisher
    ctrl_setpoint_pub = rospy.Publisher(ctrlsetpoint_topic_, 
            ControlSetpoint, queue_size=10)

    rospy.spin()

