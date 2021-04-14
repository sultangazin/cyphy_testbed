#!/usr/bin/env python

# Python script to start a crazyflie mission.
# 
# The script expects the name of the file as a parameter. It is also possible
# to specify the frequency of the thread publishing the position of the 
# 'ghost', that is the simulation of the expected trajectory.
# 
# Precisely the script will:
#     Load a trajectory from file
#     Upload the trajectory on the crazyflie
#     Ask the crazyflie to takeoff
#     Send the command to start the mission(trajectory)
#     Start a thread that simulate the trjectory execution
#     Ask the crazyflie to land after the end of the mission
# 
# 

import numpy as np
import rospy
import crazyflie
import time
import uav_trajectory
from threading import Thread
from crazyflie_demo.msg import Trajectory 
from tf.transformations import euler_from_matrix

if __name__ == '__main__':
    rospy.init_node('Node_upload_and_exe_trj')

    cf = crazyflie.Crazyflie("cf1", "/tf")

    rospy.loginfo("Uploading Trajectory...")
    traj = uav_trajectory.Trajectory()
    traj.loadcsv('/tmp/toTarget.csv') 
    cf.uploadTrajectory(0, 0, traj)
    rospy.loginfo("Trajectory duration: " + str(traj.duration))
    time.sleep(3)

    rospy.loginfo("Starting Trajectory")
    cf.startTrajectory(0, timescale=1.0)
    cf.startTrajectory(0, timescale=1.0)

    time.sleep(traj.duration * 1.5)
 
    cf.stop()

