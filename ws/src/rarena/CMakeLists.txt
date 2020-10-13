cmake_minimum_required(VERSION 2.8.6)
project(rarena)

find_package(catkin REQUIRED 
	COMPONENTS 
	rospy
	testbed_msgs
    commander_interface
	control_router
	)


catkin_package(
	CATKIN_DEPENDS 
    rospy
    testbed_msgs
	message_runtime
	control_router
	)
