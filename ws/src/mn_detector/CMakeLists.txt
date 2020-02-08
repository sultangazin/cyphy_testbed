cmake_minimum_required(VERSION 2.8.3)
project(mn_detector)

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

# Declare the executable to be build and the associated source
add_executable(mn_detector 
	exec/mn_detector_node.cpp 
	src/mn_detector.cpp
	)

add_dependencies(mn_detector 
	${catkin_EXPORTED_TARGETS}
	)

target_link_libraries(mn_detector
	${catkin_LIBRARIES}
	Eigen3::Eigen 
	)
