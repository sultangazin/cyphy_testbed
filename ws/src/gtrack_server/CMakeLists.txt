cmake_minimum_required(VERSION 2.8.3)
project(gtrack_server)

# Find required packages
find_package(
	catkin REQUIRED 
	COMPONENTS 
    tf
	roscpp 
	std_msgs
	testbed_msgs
	)
find_package (Eigen3 REQUIRED NO_MODULE)

catkin_package(
	INCLUDE_DIRS include/ # The package contains header files
   	CATKIN_DEPENDS roscpp tf std_msgs
	)

link_directories(/home/rt-2pm2/Extensions/rpclib/build)

# Since we use eigen
include_directories(
	include/
    ~/Extensions/rpclib/include
	${catkin_INCLUDE_DIRS} # Found in "find_package"
	)

# Declare the executable to be build and the associated source
add_executable(gtrack_server 
	exec/gtrack_server_node.cpp 
	src/gtrack_server.cpp
    src/gatlas.cpp
	)

add_dependencies(gtrack_server 
	${catkin_EXPORTED_TARGETS}
	)

target_link_libraries(gtrack_server
    librpc.a
	${catkin_LIBRARIES}
	Eigen3::Eigen 
	)
