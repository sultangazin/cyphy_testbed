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

import uav_trajectory
from threading import Thread
from crazyflie_demo.msg import Trajectory 
from geometry_msgs.msg import Vector3 
from tf.transformations import euler_from_matrix

from crazyflie_demo.msg import ExperimentLog 



def req_takeoff(cf):
    cf.takeoff(targetHeight = 0.7, duration = 2.0)
    cf.takeoff(targetHeight = 0.7, duration = 2.0)
    time.sleep(2.5)

def switch_ctrl_module(cf, ctrl_id):
    print('Selecting Controller ' + str(ctrl_id))
    cf.setParam("stabilizer/controller", ctrl_id)
    update_params(["stabilizer/controller"])

def switch_est_module(cf, est_id):
    print('Selecting Estimator ' + str(est_id))
    cf.setParam("stabilizer/estimator", est_id)
    update_params(["stabilizer/estimator"])

def set_ctrl_gains(cf, gains):
    print('Setting Control Gains...')
    for (name, k) in gains.items():
        print(name + " --> " +  str(k))
        while (cf.getParam("ctrlDD_par/" + name) != k):
            cf.setParam("ctrlDD_par/" + name, k)
            update_params(["ctrlDD_par/" + name])
            time.sleep(0.1)
    print("DONE!\n")

def set_est_gains(cf, gains):
    print('Setting Estimator Gains...')
    for (name, k) in gains.items():
        print(name + " --> " +  str(k))
        while (cf.getParam("estimatorDD_par/" + name) != k):
            cf.setParam("estimatorDD_par/" + name, k)
            update_params(["estimatorDD_par/" + name])
            time.sleep(0.1)
    print("DONE!\n")




def req_landing(cf):
    rospy.loginfo("Landing")
    cf.land(targetHeight = 0.05, duration = 2.0)
    time.sleep(0.1)
    cf.land(targetHeight = 0.0, duration = 2.0)
    time.sleep(2)


if __name__ == '__main__':
    rospy.init_node('Captain')

    rospy.loginfo("Starting Node Captain")
    vehicle_name = "cf3"
    
    cf = crazyflie.Crazyflie(vehicle_name, "/tf")

    rospy.wait_for_service('/' + vehicle_name + '/update_params')
    rospy.loginfo("Commander found update_params service")
    update_params = rospy.ServiceProxy('/' + vehicle_name + '/update_params', UpdateParams)

    ControlGains = {
            'Kxy': -0.25,
            'Kxy_d': -0.1,
            'Kz': -20,
            'Kz_d': -10,
            'Katt': -25.0,
            'Katt_d': -5.0,
            'Kyaw': 60.0,
            'Kyaw_d': 0.0
    }

    EstimationGains = {
            'K_x': 0.00001, 
            'K_x_d': 0.1,  
            'K_y': 0.00001,
            'K_y_d': 0.1,  
            'K_a2d0': 0.02,
            'K_a2d1': 0.02,
            'K_a2d2': 0.02,
            'K_a2d3': 0.02,
            'K_b2d0': 0.0005,
            'K_b2d1': 0.0005,
            'K_b2d2': 0.0005,
            'K_b2d3': 0.0005,
            'K_b2d6': 0.0005,
            'K_b2d7': 0.0005,
            'K_b2d8': 0.0005,
            'K_b2d9': 0.0005,
            'K_b2d10': 0.0005,
            'K_b2d11': 0.0005,
            'K_b2d12': 0.0005,
            'K_b2d13': 0.0005,
            'K_b2d14': 0.0005,
            'K_b2d15': 0.0005,
            
    }

    set_ctrl_gains(cf, ControlGains)
    time.sleep(1.0)
    set_est_gains(cf, EstimationGains)
    time.sleep(1.0)

    switch_ctrl_module(cf, 4)
    switch_est_module(cf, 3)

    time.sleep(2.0)

    #req_takeoff(cf) 
    cf.reboot()

    #tg_pos = [0, 0.2, 0.7]
    #cf.goTo(goal = tg_pos, yaw=0.0, duration = 3.0, relative = False)
    
    ####### END MISSION
    #req_landing(cf)
 
    #cf.stop()

