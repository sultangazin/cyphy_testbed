#!/usr/bin/env python

# Guidance Node
import rospy

from rarena import RArenaClass


if __name__ == '__main__':
    rospy.init_node('RArena_node')

    rarena = RArenaClass("/topic/luigi/", 'target', 'target')

    rospy.spin()
