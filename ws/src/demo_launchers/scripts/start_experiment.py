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

from crazyflie_driver.srv import UpdateParams

from threading import Thread
from geometry_msgs.msg import Vector3 
from tf.transformations import euler_from_matrix

from crazyflie_demo.msg import ExperimentLog 

# Trajectory Publisher


# Global Variables
dist_enable = False
onboard_distortion = True 

enabled_distortion = 0
malicious_anchor = 4
enabled_module = 0
exp_msg = ExperimentLog()

exp_msg.enabled_distortion = enabled_distortion
exp_msg.malicious_anchor = malicious_anchor
exp_msg.enabled_module = enabled_module


# This was publishing Trajectory messages.
def rep_trajectory(trajectory, start_position, freq):
        timeSpan = trajectory.duration; 

        r = rospy.Rate(freq)

        print("Running at freq. = ", r)
        start_time = rospy.get_time() 
        curr_time = start_time
        print("Start Position: ", start_position)
        print("Current time: ", curr_time)
        print("Start time: ", start_time)
        print("Expected end time: ", start_time + timeSpan)
        end_time = start_time + timeSpan

        msg = Vector3()

        # Publishing Loop
        while (curr_time < end_time):
            # Evaluate the trajectory
            rep_trj = trajectory.eval(curr_time - start_time)

            msg.x = rep_trj.pos[0]
            msg.y = rep_trj.pos[1]
            msg.z = rep_trj.pos[2]

            # Pubblish the evaluated trajectory
            ghost_pub.publish(msg)

            # Wait the next loop
            # Take the time
            curr_time = curr_time + 0.1



def req_takeoff(cf, h):
    time_ = 1.5
    cf.takeoff(targetHeight = h, duration = time_)
    cf.takeoff(targetHeight = h, duration = time_)
    time.sleep(time_)

def req_start_trj(cf):
    rospy.loginfo("Starting Trajectory")
    cf.startTrajectory(0, timescale=1.0, relative=False)
    cf.startTrajectory(0, timescale=1.0, relative=False)
    time.sleep(0.5)
    

def switch_mnd_module(cf, flag):
    global exp_msg

    if (flag):
        print("Activating the Malicious Node Detector")
        print(rospy.get_rostime())
        cf.setParam("mnd_param/activate", 1)
        exp_msg.enabled_module = 1;
    else:
        print("Deactivating the Malicious Node Detector")
        print(rospy.get_rostime())
        cf.setParam("mnd_param/activate", 0)
        exp_msg.enabled_module = 0;

    update_params(["mnd_param/activate"])
    update_params(["mnd_param/activate"])
   
    now_time = rospy.get_rostime()
    exp_msg.header.seq = exp_msg.header.seq + 1
    exp_msg.header.stamp.secs = now_time.secs
    exp_msg.header.stamp.nsecs = now_time.nsecs
    experiment_pub.publish(exp_msg)

    rospy.sleep(0.2) 


def set_threshold(cf, value):
    cf.setParam("mnd_param/abs_threshold", value)

    update_params(["mnd_param/abs_threshold"])
    update_params(["mnd_param/abs_threshold"])


def switch_distortion(flag, dist_amount=0.3):
    global exp_msg
    global dist_enable
    global malicious_anchor

    if (flag):
        dist_enable = True
        if (onboard_distortion):
            print("Enabling Distortion Onboard:")
            print(rospy.get_rostime())
            cf.setParam("twr/enable_distortion", 1)
            cf.setParam("twr/malicious_id", malicious_anchor)
            cf.setParam("twr/dist_amount", dist_amount)
            # For recording purposes I do the same for the simulated
            rospy.set_param("/Dummy_Anchors/enable_distortion", True)
        else:
            print("Enabling Distortion Offboard...")
            print(rospy.get_rostime())
            rospy.set_param("/Dummy_Anchors/distortion_value", dist_amount)
            rospy.set_param("/Dummy_Anchors/enable_distortion", True)
        exp_msg.enabled_distortion = 1
    else:
        dist_enable = False
        print("Disabling Distortion...")
        print(rospy.get_rostime())
        if (onboard_distortion):
            cf.setParam("twr/enable_distortion", 0)
            # For recording purposes I do the same for the simulated
            rospy.set_param("/Dummy_Anchors/enable_distortion", False)
        else:
            rospy.set_param("/Dummy_Anchors/enable_distortion", False)

        exp_msg.enabled_distortion = 0

    update_params(["twr/enable_distortion"])
    update_params(["twr/enable_distortion"])

    update_params(["twr/malicious_id"])
    update_params(["twr/malicious_id"])
    
    update_params(["twr/dist_amount"])
    update_params(["twr/dist_amount"])

    exp_msg.header.seq = exp_msg.header.seq + 1
    now_time = rospy.get_rostime()
    exp_msg.header.stamp.secs = now_time.secs
    exp_msg.header.stamp.nsecs = now_time.nsecs
    experiment_pub.publish(exp_msg)


def req_landing(cf):
    rospy.loginfo("Landing")
    cf.land(targetHeight = 0.0, duration = 2.0)
    time.sleep(0.1)
    cf.land(targetHeight = 0.0, duration = 2.0)
    time.sleep(2)

def uploadTrajToDrone(cf, traj):
    rospy.loginfo("Uploading Trajectory...")
     
    cf.uploadTrajectory(0, 0, traj)
    rospy.loginfo("Trajectory duration: " + str(traj.duration))
    time.sleep(1)

    return traj


def pieceWiseTrj(cf, distortion=False):
    _T = 5.0
     
    cf.goTo([0.79, -0.7, 0.6], 0.0, _T, relative=False)
    cf.goTo([0.79, -0.7, 0.6], 0.0, _T, relative=False)
    cf.goTo([0.79, -0.7, 0.6], 0.0, _T, relative=False)
    start_time = rospy.get_time()
    stop_time = start_time +  _T
    time.sleep(stop_time - rospy.get_time())

    
    cf.goTo([0.79, 0.3, 0.6], 0.0, _T, relative=False)
    cf.goTo([0.79, 0.3, 0.6], 0.0, _T, relative=False)
    cf.goTo([0.79, 0.3, 0.6], 0.0, _T, relative=False) 
    start_time = rospy.get_time()
    stop_time = start_time +  _T
    if (distortion and distortion != dist_enable):
        time.sleep(1.0)
        if (onboard_distortion):
            switch_distortion(True, 15.00)
        else:
            switch_distortion(True, 0.45)
        time.sleep(0.6)
        switch_mnd_module(cf, True)
        time.sleep(stop_time - rospy.get_time())
    else:
        time.sleep(stop_time - rospy.get_time())

    
    cf.goTo([0.79, 0.3, 0.3], 0.0, _T, relative=False)
    cf.goTo([0.79, 0.3, 0.3], 0.0, _T, relative=False)
    cf.goTo([0.79, 0.3, 0.3], 0.0, _T, relative=False)
    start_time = rospy.get_time()
    stop_time = start_time +  _T 
    time.sleep(stop_time - rospy.get_time())

    
    cf.goTo([0.79, -0.7, 0.3], 0.0, _T, relative=False)
    cf.goTo([0.79, -0.7, 0.3], 0.0, _T, relative=False)
    cf.goTo([0.79, -0.7, 0.3], 0.0, _T, relative=False)
    start_time = rospy.get_time()
    stop_time = start_time +  _T 
    time.sleep(stop_time - rospy.get_time())
    time.sleep(0.5)

if __name__ == '__main__':
    rospy.init_node('Captain')

    rospy.loginfo("Starting Node Captain")

    cf = crazyflie.Crazyflie("cf2", "/tf")

    rospy.wait_for_service('/cf2/update_params')
    rospy.loginfo("Commander found update_params service")
    update_params = rospy.ServiceProxy('/cf2/update_params', UpdateParams)

    frequency = rospy.get_param('freq_ghost', 2.0);

    ## Load Trajectory File
    file_name = rospy.search_param('trajectory_file')
    if (file_name):
        trj_file = rospy.get_param(file_name) 
        print("Trajectory file parameter found! ", trj_file)
    else:
        rospy.signal_shutdown("Trjectory file not found!")
    traj = uav_trajectory.Trajectory()
    traj.loadcsv(trj_file)

    ## Upload the trajectory file to drone
    uploadTrajToDrone(cf, traj)

    ####### START MISSION
    now_time = rospy.get_rostime()
    exp_msg.header.stamp.secs = now_time.secs
    exp_msg.header.stamp.nsecs = now_time.nsecs
    experiment_pub.publish(exp_msg)


    # Disable distortion and the onboard filter
    switch_distortion(False)
    switch_mnd_module(cf, False)

    if (onboard_distortion):
        set_threshold(cf, 800.0)
    else:
        set_threshold(cf, 0.25)
    time.sleep(0.5)

    req_takeoff(cf, 0.5) 

    time.sleep(1.0)

    cf.goTo([0.79, -0.70, 0.3], 0.0, 5.0, relative=False)
    cf.goTo([0.79, -0.70, 0.3], 0.0, 5.0, relative=False)
    time.sleep(5.5)

    ## Follow Trajectory 1
    pieceWiseTrj(cf) 
    #req_start_trj(cf) 
    #t = Thread(target=rep_trajectory,
    #        args=(traj,[0.0, 0.0, 0.0], frequency)).start() 
    #time.sleep(traj.duration)


    # Follow Trajetory 2 and enable distortion

    pieceWiseTrj(cf, True) 
    #req_start_trj(cf)
    ## Enable Distortion
    #_dist_delay = 1.0
    #time.sleep(_dist_delay) # After 3 seconds: activate distortion and the module
    print("===============================")
    #time.sleep(0.5)

#    if (onboard_distortion):
#        switch_distortion(True, 28.00)
#    else:
#        switch_distortion(True, 0.45)
#    time.sleep(0.5)
#
#    switch_mnd_module(cf, True)

    #time.sleep(traj.duration - _dist_delay) 

    pieceWiseTrj(cf)
    #req_start_trj(cf)
    #time.sleep(traj.duration + 1.0)
    
    ####### END MISSION
    req_landing(cf) 
    switch_distortion(False)
    switch_mnd_module(cf, False)
    cf.stop()

