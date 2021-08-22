cmake_minimum_required(VERSION 3.5)
project(cis_supervisor)
add_compile_options(-std=c++17)

find_package(catkin REQUIRED 
	COMPONENTS
	roscpp
	std_msgs
	testbed_msgs
	geometry_msgs
	message_generation
)

find_package (Eigen3 REQUIRED NO_MODULE)
find_package(CGAL)
find_package(cis2m REQUIRED NO_MODULE)
find_package(drake CONFIG REQUIRED PATHS /opt/drake)

add_message_files(
	DIRECTORY msg
	FILES ObstacleMsg.msg
	)

generate_messages(DEPENDENCIES std_msgs geometry_msgs)

include_directories(
	include
	${catkin_INCLUDE_DIRS}
	${DRAKE_INCLUDE_DIRS}
)

# Flat output Controller
add_executable(cis_supervisor_node
	exec/cis_supervisor_node.cpp 
	src/cis_supervisor.cpp
	src/cis_supervisor_ros.cpp
)

add_dependencies(cis_supervisor_node
	${catkin_EXPORTED_TARGETS}
	)

target_link_libraries(cis_supervisor_node
	Eigen3::Eigen 
	CGAL::CGAL
	cis2m::cis2m
	drake::drake
	${catkin_LIBRARIES}
)

catkin_package(
		CATKIN_DEPENDS 
		roscpp
		std_msgs
		geometry_msgs
		message_runtime
)
