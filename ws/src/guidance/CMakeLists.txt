cmake_minimum_required(VERSION 2.8.6)
project(guidance)

find_package(catkin REQUIRED 
	COMPONENTS 
    roscpp
	rospy
	testbed_msgs
	message_generation 
	)

file(GLOB srv_files RELATIVE 
	${PROJECT_SOURCE_DIR}/srv ${PROJECT_SOURCE_DIR}/srv/*.srv)

add_service_files(
	DIRECTORY srv 
	FILES ${srv_files}
	)

generate_messages(DEPENDENCIES
    testbed_msgs)

catkin_package(
	CATKIN_DEPENDS 
    rospy
    testbed_msgs
	message_runtime)
