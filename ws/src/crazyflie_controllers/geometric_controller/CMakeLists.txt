cmake_minimum_required(VERSION 2.8.3)
project(geometric_controller)
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
	CATKIN_DEPENDS 
	roscpp
	message_runtime
	tf
	std_msgs
	testbed_msgs
    	crazyflie_driver
)


include_directories(
	include
	${catkin_INCLUDE_DIRS}
)


# Geometric Controller
add_executable(geometric_controller_node 
	exec/geometric_controller_node.cpp 
	src/geometric_controller.cpp
)

add_dependencies(geometric_controller_node
	${catkin_EXPORTED_TARGETS}
	)

target_link_libraries(
	geometric_controller_node
	Eigen3::Eigen 
	${catkin_LIBRARIES}
)
