cmake_minimum_required(VERSION 2.8.6)
project(manager)

find_package(catkin REQUIRED 
	COMPONENTS 
	rospy
	testbed_msgs
	message_generation 
    commander_interface
	)

file(GLOB srv_files RELATIVE 
	${PROJECT_SOURCE_DIR}/srv ${PROJECT_SOURCE_DIR}/srv/*.srv)

add_service_files(
	DIRECTORY srv 
	FILES ${srv_files}
	)

generate_messages()

catkin_package(
	CATKIN_DEPENDS 
    rospy
    testbed_msgs
    commander_interface
	message_runtime)
