#!/usr/bin/env python

# This script loads a trajectory file and simulates the execution publishing a 
# custom trajectory message that is converted in odometry by another node

import numpy as np
import rospy
import crazyflie
import time
import uav_trajectory
from threading import Thread
from crazyflie_demo.msg import Trajectory 
from geometry_msgs.msg import Vector3 
from tf.transformations import euler_from_matrix


# Trajectory Publisher
ghost_pub = rospy.Publisher('cf2/ghost_trajectory', Vector3, queue_size=10)

# This was publishing Trajectory messages.
def rep_trajectory(trajectory, start_position, freq):
        timeSpan = trajectory.duration; 

        r = rospy.Rate(freq)

        print("Running at freq. = ", r)
        start_time = rospy.get_time() 
        curr_time = start_time
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
            r.sleep()
            # Take the time
            curr_time = rospy.get_time()


  

if __name__ == '__main__':
    rospy.init_node('Ghost_Node')

    start_point = rospy.get_param('start_p', [0.0, 0.0 ,0.0])
    
    file_name = rospy.search_param('trajectory_file')
    if (file_name):
        trj_file = rospy.get_param(file_name) 
        print("Trajectory file parameter found! ", trj_file)
    else:
        rospy.signal_shutdown("Trjectory file not found!")

    
    frequency = rospy.get_param('freq', 2.0);

    
    print("Loading trajectory from: ", trj_file)
    traj = uav_trajectory.Trajectory()
    traj.loadcsv(trj_file)

    t = Thread(target=rep_trajectory, args=(traj,start_point, frequency)).start()
 
