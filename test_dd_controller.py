#!/usr/bin/env python

import rospy
import crazyflie
import time
import uav_trajectory

if __name__ == '__main__':
    rospy.init_node('test_dd_controller')

    cf = crazyflie.Crazyflie("cf1", "/tf")
    
    cf.setParam("stabilizer/dd_ctrl_active", 0)
    cf.setParam("controller_dd/ctrl_dd_L", 1)
    cf.takeoff(targetHeight = 1.2, duration = 3.0)
    time.sleep(3.0)

    # Hovering time
    time.sleep(10.0)

    # Starting the DD Controller
    while (cf.getParam("stabilizer/dd_ctrl_active") != 1):
        cf.setParam("stabilizer/dd_ctrl_active", 1)
        time.sleep(1.0)
    print("Activate Data Driven Controller")


    # Hovering with the DD active
    time.sleep(15.0)

    while (cf.getParam("stabilizer/dd_ctrl_active") != 0):
        cf.setParam("stabilizer/dd_ctrl_active", 0)
        time.sleep(0.5)
        


    # Landing
    cf.land(targetHeight = 0.0, duration = 2.0)
    cf.land(targetHeight = 0.0, duration = 2.0)
    time.sleep(2.0)
    cf.stop()
