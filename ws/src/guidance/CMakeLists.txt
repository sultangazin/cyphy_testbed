cmake_minimum_required(VERSION 3.5)
project(guidance)

find_package(catkin REQUIRED 
	COMPONENTS 
    roscpp
	rospy
    actionlib_msgs
    actionlib
    genmsg
	testbed_msgs
	message_generation 
	)

file(GLOB srv_files RELATIVE 
	${PROJECT_SOURCE_DIR}/srv ${PROJECT_SOURCE_DIR}/srv/*.srv)

add_service_files(
	DIRECTORY srv 
	FILES ${srv_files}
	)

add_action_files(
    DIRECTORY action 
    FILES GuidanceTarget.action)

generate_messages(
    DEPENDENCIES 
    actionlib_msgs)

catkin_package(
	CATKIN_DEPENDS 
    rospy
    testbed_msgs
    actionlib_msgs
    actionlib
	message_runtime)
