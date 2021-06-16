cmake_minimum_required(VERSION 2.8.3)
project(fblin_controller)
add_compile_options(-std=c++17)

find_package(catkin REQUIRED 
	COMPONENTS
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
	tf
	std_msgs
	testbed_msgs
    	crazyflie_driver
)

include_directories(
	include
	${catkin_INCLUDE_DIRS}
)


# Flat output Controller
add_executable(fblin_controller 
	exec/fblin_controller_node.cpp 
	src/fblin_controller.cpp
)

add_dependencies(fblin_controller
	${catkin_EXPORTED_TARGETS}
	)

target_link_libraries(
	fblin_controller
	Eigen3::Eigen 
	${catkin_LIBRARIES}
)
