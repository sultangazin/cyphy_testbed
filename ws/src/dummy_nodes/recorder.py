#!/usr/bin/env python
#
# Dummy node publishing dummy estimation data
#
#

import rospy
import numpy as np

from testbed_msgs.msg import AnchorMeas 
from crazyflie_driver.msg import GenericLogData 

anchor_meas = np.zeros((9,1));

def anchor_callback(msg):
    anchor_meas = msg.meas

def ctrl_callback(msg):
    pass

if __name__ == "__main__":
    rospy.init_node("RecordingNode")
    
    rospy.Subscriber("/cf2/simulated_anchors", 
            AnchorMeas, anchor_callback)

    rospy.Subscriber("/cf2/logCtrl", GenericLogData,
            ctrl_callback)

    record_publisher = rospy.Publisher("/MNDetection/record", 
            MNDectetionRecord, queue_size=10)

    while (not rospy.is_shutdown()):
        rospy.spin()
