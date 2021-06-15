cmake_minimum_required(VERSION 2.8.3)
project(cmd_vel_converter)
add_compile_options(-std=c++17)

find_package(catkin REQUIRED 
	COMPONENTS
	message_generation
	roscpp
	tf
	std_msgs
	testbed_msgs
    	crazyflie_driver
)

find_package (Eigen3 REQUIRED NO_MODULE)

catkin_package(
	INCLUDE_DIRS 
	include
	CATKIN_DEPENDS 
	roscpp
	message_runtime
	tf
	std_msgs
	testbed_msgs
    	crazyflie_driver
)


include_directories(
	include/
	${catkin_INCLUDE_DIRS}
)

add_executable(cmd_vel_converter_node 
	exec/cmd_vel_converter_node.cpp 
	src/cmd_vel_converter.cpp
)

add_dependencies(cmd_vel_converter_node 
	${catkin_EXPORTED_TARGETS}
	)

target_link_libraries(
	cmd_vel_converter_node
	Eigen3::Eigen 
	${catkin_LIBRARIES}
)
