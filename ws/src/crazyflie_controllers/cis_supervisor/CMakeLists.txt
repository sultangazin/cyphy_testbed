cmake_minimum_required(VERSION 2.8.3)
project(cis_supervisor)
add_compile_options(-std=c++17)

find_package(catkin REQUIRED 
	COMPONENTS
	roscpp
	std_msgs
	testbed_msgs
)

find_package (Eigen3 REQUIRED NO_MODULE)
find_package(CGAL)

catkin_package(
		CATKIN_DEPENDS 
	#	roscpp
	#	std_msgs
		testbed_msgs
)

include_directories(
	include
	${catkin_INCLUDE_DIRS}
)

# Flat output Controller
add_executable(cis_contoller_node
	exec/cis_supervisor_node.cpp 
	src/cis_supervisor.cpp
	src/cis_supervisor_ros.cpp
)

add_dependencies(cis_contoller_node
	${catkin_EXPORTED_TARGETS}
	)

target_link_libraries(cis_contoller_node
	Eigen3::Eigen 
	CGAL::CGAL
	${catkin_LIBRARIES}
)
