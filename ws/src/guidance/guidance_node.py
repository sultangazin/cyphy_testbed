#!/usr/bin/env python3

# Guidance Node
import rospy
import os
import sys
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), './')))

from guidance_class.guidance_class import GuidanceClass 

if __name__ == '__main__':
    rospy.init_node('Guidance_node')

    # Instatiate the guidance class
    guidance = GuidanceClass()

    rospy.spin()
