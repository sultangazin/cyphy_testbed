cmake_minimum_required(VERSION 3.5)
project("testbed_msgs")

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(catkin REQUIRED COMPONENTS
	roscpp
	std_msgs
    geometry_msgs
	message_generation
	)


file(GLOB msg_files RELATIVE 
	${PROJECT_SOURCE_DIR}/msg ${PROJECT_SOURCE_DIR}/msg/*.msg)

add_message_files(
	DIRECTORY msg 
	FILES ${msg_files}
	)

generate_messages(
	DEPENDENCIES
	std_msgs
    geometry_msgs
	)

catkin_package(
	CATKIN_DEPENDS
	roscpp
	std_msgs
    geometry_msgs
	message_runtime
	)

