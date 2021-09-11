cmake_minimum_required(VERSION 2.8.3)
project(lfd_controller)
add_compile_options(-std=c++17)

find_package(catkin REQUIRED 
	COMPONENTS
	roscpp
	roslib
	std_msgs
	testbed_msgs
    	crazyflie_driver
)

find_package (Eigen3 REQUIRED NO_MODULE)

catkin_package(
		CATKIN_DEPENDS 
	#	roscpp
	#	std_msgs
		testbed_msgs
	#    	crazyflie_driver
)

include_directories(
	include
	${catkin_INCLUDE_DIRS}
)


# Flat output Controller
add_executable(lfd_controller_node
	exec/lfd_controller_node.cpp 
	src/lfd_controller.cpp
)

add_dependencies(lfd_controller_node
	${catkin_EXPORTED_TARGETS}
	)

target_link_libraries(lfd_controller_node
	Eigen3::Eigen 
	${catkin_LIBRARIES}
)
