cmake_minimum_required(VERSION 2.8.3)
project(crazyflie_controllers)
add_compile_options(-std=c++17)

find_package(catkin REQUIRED 
	COMPONENTS
	message_generation
	roscpp
	tf
	std_msgs
	testbed_msgs
    	crazyflie_driver
)

find_package (Eigen3 REQUIRED NO_MODULE)

catkin_package(
	INCLUDE_DIRS 
	common/include 
	geometric_controller/include
	fblin_controller/include
	CATKIN_DEPENDS 
	roscpp
	message_runtime
	tf
	std_msgs
	testbed_msgs
    	crazyflie_driver
)

include_directories(
	common/include
	geometric_controller/include
	fblin_controller/include
	${catkin_INCLUDE_DIRS}
)



add_executable(cmd_vel_converter_node 
	common/exec/cmd_vel_converter_node.cpp 
	common/src/cmd_vel_converter.cpp
)

add_dependencies(cmd_vel_converter_node 
	${catkin_EXPORTED_TARGETS}
	)

target_link_libraries(
	cmd_vel_converter_node
	Eigen3::Eigen 
	${catkin_LIBRARIES}
)


# Geometric Controller
add_executable(geometric_controller_node 
	geometric_controller/exec/geometric_controller_node.cpp 
	geometric_controller/src/geometric_controller.cpp
)

add_dependencies(geometric_controller_node
	${catkin_EXPORTED_TARGETS}
	)

target_link_libraries(
	geometric_controller_node
	Eigen3::Eigen 
	${catkin_LIBRARIES}
)


# Flat output Controller
add_executable(fblin_controller_node 
	fblin_controller/exec/fblin_controller_node.cpp 
	fblin_controller/src/fblin_controller.cpp
)

add_dependencies(fblin_controller_node
	${catkin_EXPORTED_TARGETS}
	)

target_link_libraries(
	fblin_controller_node
	Eigen3::Eigen 
	${catkin_LIBRARIES}
)







