cmake_minimum_required(VERSION 2.8.3)
project(state_aggregator)
add_compile_options(-std=c++11)

# Find required packages
find_package(
	catkin REQUIRED 
	COMPONENTS 
	roscpp 
	tf 
	std_msgs
	testbed_msgs
	)
find_package (Eigen3 REQUIRED NO_MODULE)

catkin_package(
	INCLUDE_DIRS include/ # The package contains header files
	CATKIN_DEPENDS roscpp tf std_msgs
	)

# Since we use eigen
include_directories(
	include/
	${catkin_INCLUDE_DIRS} # Found in "find_package"
	)

#include_directories(${/usr/local/include/eigen3/Eigen})

# Declare the executable to be build and the associated source
add_executable(state_aggregator 
	exec/state_aggregator_node.cpp 
	src/state_aggregator.cpp
	)

add_dependencies(state_aggregator 
	${catkin_EXPORTED_TARGETS}
	)

target_link_libraries(state_aggregator
	${catkin_LIBRARIES}
	Eigen3::Eigen 
	)
