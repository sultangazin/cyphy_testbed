cmake_minimum_required(VERSION 2.8.6)
project(dummy_nodes)

find_package(catkin REQUIRED
    COMPONENTS
    rospy
    testbed_msgs
    )

catkin_package(
    CATKIN_DEPENDS
    rospy
    testbed_msgs
    message_runtime
    )
