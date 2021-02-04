cmake_minimum_required(VERSION 2.8.3)
project(cis_supervisor)
add_compile_options(-std=c++17)

#set(GUROBI_CXX_LIBRARY "$ENV{GUROBI_HOME}/lib/libgurobi_c++.a")
set(GUROBI_CXX_LIBRARY "$ENV{GUROBI_HOME}/src/build/libgurobi_c++.a")

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
	roslib
	)

find_package (Eigen3 REQUIRED NO_MODULE)
find_package(GUROBI REQUIRED)

add_service_files(
	DIRECTORY srv
	FILES
	CISSupervisorTune.srv
	)

add_message_files(
	DIRECTORY msg 
	FILES
	MotorsCtrlStamped.msg
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
	${GUROBI_INCLUDE_DIRS}
	)


add_library(gurobi_routines
	src/gurobiRoutines.cpp
	)

target_include_directories(gurobi_routines
	PUBLIC
	${GUROBI_INCLUDE_DIRS}
	)

message("XXXXXXXXXXXXXXXXXXXX")
message(${GUROBI_CXX_LIBRARY})
message(${GUROBI_LIBRARY})
message("Include: " ${GUROBI_INCLUDE_DIRS})

target_link_libraries(gurobi_routines
	PUBLIC
	${GUROBI_LIBRARY}
	${GUROBI_CXX_LIBRARY}
	Eigen3::Eigen
	)



# Declare the executable to be build and the associated source
add_executable(cis_supervisor 
	# src/gurobiRoutines.cpp
	exec/cis_supervisor_node.cpp 
	src/cis_supervisor.cpp
	src/cis_supervisor_ros.cpp)

add_dependencies(cis_supervisor 
	${catkin_EXPORTED_TARGETS}
	)

target_link_libraries(cis_supervisor
	${GUROBI_LIBRARY}
	${GUROBI_CXX_LIBRARY}
	${catkin_LIBRARIES}
	gurobi_routines
	utilities
	Eigen3::Eigen 
	)
