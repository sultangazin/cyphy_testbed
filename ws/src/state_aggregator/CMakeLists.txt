cmake_minimum_required(VERSION 2.8.3)
project(state_aggregator)

# Find required packages
find_package(
	catkin REQUIRED 
	COMPONENTS 
	roscpp 
	tf 
	std_msgs
    message_generation
	testbed_msgs
    utilities
	)
find_package (Eigen3 REQUIRED NO_MODULE)


add_service_files(
	FILES
    ControlSensor.srv
	)

generate_messages()


catkin_package(
	INCLUDE_DIRS include/ # The package contains header files
	CATKIN_DEPENDS roscpp tf std_msgs
    message_runtime
	)

# Since we use eigen
include_directories(
	include/
	${catkin_INCLUDE_DIRS} # Found in "find_package"
	)

# Declare the executable to be build and the associated source
add_executable(state_aggregator 
	exec/state_aggregator_node.cpp 
	src/state_aggregator.cpp
    src/filter/ddfilter.cpp
    src/filter/polyfilter.cpp
	)

add_dependencies(state_aggregator 
	${catkin_EXPORTED_TARGETS}
	)

target_link_libraries(state_aggregator
	${catkin_LIBRARIES}
    utilities
	Eigen3::Eigen 
	)
