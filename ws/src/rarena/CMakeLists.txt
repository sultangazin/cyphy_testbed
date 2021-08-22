cmake_minimum_required(VERSION 3.5)
project(rarena)

find_package(catkin REQUIRED 
	COMPONENTS 
	rospy
	testbed_msgs
	std_srvs
	commander_interface
	control_router
	)


catkin_package(
	CATKIN_DEPENDS 
	rospy
	testbed_msgs
	std_srvs
	message_runtime
	control_router
	)
