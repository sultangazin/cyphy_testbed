cmake_minimum_required(VERSION 2.8.3)
project(utilities)

# Find required packages
find_package(
	catkin REQUIRED 
	COMPONENTS 
	roscpp 
	std_msgs
    geometry_msgs
	)
find_package (Eigen3 REQUIRED NO_MODULE)

catkin_package(
	INCLUDE_DIRS include/ # The package contains header files
	CATKIN_DEPENDS roscpp tf std_msgs geometry_msgs
	)

# Since we use eigen
include_directories(
	include/
	${catkin_INCLUDE_DIRS} # Found in "find_package"
	)

# Declare the executable to be build and the associated source
add_library(utilities 
	src/network_parser/network_parser.cpp 
    src/timeutils/timeutils.cpp
    src/custom_conversion/custom_conversion.cpp
	)

add_dependencies(utilities 
	${catkin_EXPORTED_TARGETS}
	)

target_link_libraries(utilities
	${catkin_LIBRARIES}
	Eigen3::Eigen 
	)
