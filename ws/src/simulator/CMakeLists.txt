cmake_minimum_required(VERSION 2.8.3)
project(simulator)

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

# Add library for the simulation
add_library(simdyn
    include/simclass/dyn_class.cpp
    )
target_link_libraries(simdyn
	Eigen3::Eigen
	)

add_library(timespec_lib
    include/timespec_lib/timespec_lib.cpp
    )

# Declare the executable to be build and the associated source
add_executable(simulator 
	exec/simulator_node.cpp 
	src/simulator.cpp
	)

add_dependencies(simulator 
	${catkin_EXPORTED_TARGETS}
	)

target_link_libraries(simulator
	${catkin_LIBRARIES}
	Eigen3::Eigen 
    simdyn
    timespec_lib
	)
