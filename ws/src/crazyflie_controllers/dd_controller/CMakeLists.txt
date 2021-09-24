cmake_minimum_required(VERSION 2.8.3)
project(dd_controller)
add_compile_options(-std=c++17)

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
	DIRECTORY srv
	FILES
	DDControllerTune.srv
	)

add_message_files(
	DIRECTORY msg 
	FILES
	MotorsCtrlStamped.msg
	ParamEstimateStamped.msg
	StateEstimateStamped.msg
	PerformanceMsg.msg
	)

generate_messages(
	DEPENDENCIES
	std_msgs
	geometry_msgs
	)


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
add_executable(dd_controller 
	exec/dd_controller_node.cpp 
	src/dd_controller.cpp
	src/dd_estimator.cpp
	src/dd_estimator_param.cpp
	src/dd_controller_ros.cpp
	)

add_dependencies(dd_controller 
	${catkin_EXPORTED_TARGETS}
	)

target_link_libraries(dd_controller
	${catkin_LIBRARIES}
	utilities
	Eigen3::Eigen 
	)
