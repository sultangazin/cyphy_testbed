#!/usr/bin/env python

# Manager Node
import rospy
import os
import sys
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), './')))

from manager_class.manager_class import ManagerClass

if __name__ == '__main__':
    rospy.init_node('Manager_node')
    manager = ManagerClass()

    rospy.spin()
