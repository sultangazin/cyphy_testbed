#!/usr/bin/env python

# Node simulating radio anchors 
#
#

import rospy
import math
import numpy as np

from anchors import Anchors 

if __name__ == '__main__':
    rospy.init_node('Anchors_Node')

    # Instatiate the guidance class
    anchor = Anchors()

    rospy.spin()
