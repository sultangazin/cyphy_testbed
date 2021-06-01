cmake_minimum_required(VERSION 2.8.6)
project(control_router)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

# Find required packages
find_package(
	catkin REQUIRED 
	COMPONENTS
	message_generation
	roscpp
	testbed_msgs
	std_msgs
	crazyflie_driver
	utilities
	)

add_service_files(
	DIRECTORY srv
	FILES 
	EnableNWController.srv
	SelectController.srv
	)

add_message_files(
	DIRECTORY msg
	FILES
	NetworkStatusMsg.msg
	)

generate_messages(
	DEPENDENCIES
	std_msgs)

catkin_package(
	CATKIN_DEPENDS roscpp
	message_runtime
	std_msgs
	testbed_msgs
	crazyflie_driver
	DEPENDS
	)

include_directories(
	include/
	${catkin_INCLUDE_DIRS}
	)

add_executable(control_router_node
	exec/control_router_node.cpp
	src/control_router.cpp
	)

add_dependencies(control_router_node
	${catkin_EXPORTED_TARGETS}
	)

target_link_libraries(control_router_node
	${catkin_LIBRARIES}
	utilities
	)
