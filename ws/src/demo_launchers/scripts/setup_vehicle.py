#!/usr/bin/env python

# Python script to setup the crazyflie algorithms
# 
# - Set the commander level
# - Set the estimation algorithm
# - Set the control algorithm
# 

import rospy
from crazyflie_py import crazyflie
import time

from crazyflie_driver.srv import UpdateParams
from geometry_msgs.msg import PointStamped

# Callback to reset the initial status of the Kalman filter
# in case of external position

ekf_initialized = False;
init_ekf = False;

def set_params(cf, param_namespace, gains):
    """ 
    Helper function to set parameters on the drone
    Args:
        param_namespace: name of the parameter set (defined on the crazyflie firmware)
        gains: dictionary with the parameters
    """
    print('Setting {} Parameters...'.format(param_namespace))
    for (name, k) in gains.items():
        while (cf.getParam(param_namespace + '/' + name) != k):
            cf.setParam(param_namespace + '/' + name, k)
            update_params([param_namespace + '/' + name])
        print("Param " + name + " --> " +  str(k))
    print("DONE!\n")


def ext_pos_callback(ext_point_stmp):
    global ekf_initialized
    global init_ekf
    
    if (not ekf_initialized) and init_ekf:
        # initialize kalman filter
        if (rospy.get_param('stabilizer/estimator') == 2):
            x = ext_point_stmp.point.x
            y = ext_point_stmp.point.y
            z = ext_point_stmp.point.z
            
            rospy.loginfo("Initializing the KF: [" + str(x) + " " + 
                    str(y) + " " + str(z) + "]")
            
            par = {'initialX': x, 'initialY': y, 'initialZ': z}
            set_params(cf, 'kalman', par) 
           
            ekf_initialized = True
            rospy.loginfo("Setup of Vehicle " + cf_id + " Complete!\n")


if __name__ == '__main__':
    rospy.init_node('Setup_node')
    rospy.loginfo(" ====== START: Vehicle Custom Setup ===== ")

    est = None 
    ctr = None 
    cmode = None 

    # ============================= 
    # Read the parameters
    cf_id = rospy.get_param('~cf', 'cf1')
    comm_lev = rospy.get_param('~comm_lev', 1);
    estimator = rospy.get_param('~Estimator', 'EKF')
    controller = rospy.get_param('~Controller', 'Mellinger')
    req_reset = rospy.get_param('~ResEstimator', True)
    ctrlMode = rospy.get_param('~ctrlMode', 'Angles')

    rospy.loginfo("Selecting CF: " + str(cf_id))
    rospy.loginfo('Selecting Commander Level: ' + str(comm_lev))
    rospy.loginfo("Selecting Estimator: " + str(estimator))
    rospy.loginfo("Selecting Controller: " + str(controller))
    rospy.loginfo("Selecting Control Mode: " + str(ctrlMode))

    # ============================= 
    # Subscribe to the external position topic, in case it is necessary.
    ext_pos_topic = "/" + cf_id + "/external_position"
    rospy.loginfo("Subscribing to the external position topic: " + str(ext_pos_topic))
    rospy.Subscriber(ext_pos_topic, PointStamped, ext_pos_callback)

    # ============================= 
    # Create CF object
    cf = crazyflie.Crazyflie("/" + cf_id, "/tf")

    rospy.wait_for_service('update_params')
    rospy.loginfo("Found update_params service!")
    update_params = rospy.ServiceProxy('update_params', UpdateParams)

    # ============================= 
    # Configuring the vehicle


    # ESTIMATOR SETUP
    # Map the estimator name to index
    if (estimator == 'CMP'):
        est = 1
    elif (estimator == 'EKF'):
        est = 2
    elif (estimator == 'DD'):
        est = 3
    else:
        rospy.loginfo('Something wrong here!')
        est = 1
    # Set the estimator on the crazyflie
    par = {'estimator': est}
    set_params(cf, 'stabilizer', par)

    # If Kalman reset the estimator
    if (estimator == 'EKF' and req_reset):   
        rospy.set_param("kalman/resetEstimation", 1)
        update_params(["kalman/resetEstimation"])
        rospy.sleep(0.5)
        update_params(["kalman/resetEstimation"])
        rospy.sleep(0.5)
        rospy.set_param("kalman/resetEstimation", 0)
        update_params(["kalman/resetEstimation"])
        rospy.sleep(0.5)
        update_params(["kalman/resetEstimation"])
        rospy.sleep(0.5)
        init_ekf = True


    # CONTROL SETUP
    # Select the commander level
    par = {'enHighLevel': comm_lev}
    set_params(cf, 'commander', par)

    # Map the Control Mode to index
    if (ctrlMode == 'Angles'):
        rospy.loginfo("Set Angles Control Mode")
        cmode = 1
    elif (ctrlMode == 'Rates'):
        rospy.loginfo("Set Rates Control Mode")
        cmode = 0
    else:
        rospy.loginfo("Set Angles Control Mode")
        rospy.loginfo('Something wrong here!')
        cmode = 1

    # Map the controller name to index
    if (controller == 'PID'):
        ctr = 1
    elif (controller == 'Mellinger'):
        ctr = 2
        # If it is necessary to change the parameters...
        #gains__ = {
        #        'kR_xy': 7000,
        #        'kw_xy': 20000
        #        }
        #set_params(cf, "ctrlMel", gains__)
    elif (controller == 'DD'):
        ctr = 4
        ControlGains = {
                'Kxy': -4.0,
                'Kxy_d': -4.0,
                'Kz': -25.0,
                'Kz_d': -10.0,
                'Katt': -16.0,
                'Katt_d': -8.0,
                'Kyaw': 34.0,
                'Kyaw_d': 12.0
        }
        set_params(cf, 'ctrlDD_par', ControlGains)
    elif (controller == 'EXT' or controller == "Ext"):
        ctr = 5
    else:
        rospy.loginfo("Something is wrong with the controller selection...")
        ctr = 1

    # Set the controller on the crazyflie
    par = {'controller': ctr}
    set_params(cf, 'stabilizer', par)
    par = {'stabModeRoll': cmode, 'stabModePitch':cmode}
    set_params(cf, 'flightmode', par) 

    
    #  1) Start wait for the system to fetch the initial position of the drone;
    #  2) Initialize the state estimate on the drone
    #  3) Die :-)
    rate = rospy.Rate(1)
    while (init_ekf and not ekf_initialized and not rospy.is_shutdown()):
        rospy.loginfo("Waiting for filter initialization...")
        rate.sleep()
        rospy.spin()

    rospy.loginfo(" ====== END: Vehicle Custom Setup ===== ")
