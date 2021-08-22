cmake_minimum_required(VERSION 3.5)
project(commander_interface)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

# Find required packages
find_package(
	catkin REQUIRED 
	COMPONENTS
	roscpp
	message_generation
	guidance
    control_router
    actionlib
)

file(GLOB srv_files RELATIVE
	${PROJECT_SOURCE_DIR}/srv ${PROJECT_SOURCE_DIR}/srv/*.srv)

add_service_files(
	DIRECTORY srv
	FILES ${srv_files}
	)

generate_messages()

catkin_package(
	CATKIN_DEPENDS roscpp
	message_runtime
	DEPENDS
	guidance
    actionlib
    control_router
	)

include_directories(
	include/
	${catkin_INCLUDE_DIRS}
	)

add_executable(commander
	exec/commander_node.cpp
	src/commander.cpp
	)

add_dependencies(commander 
	${catkin_EXPORTED_TARGETS}
	${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(commander
	${catkin_LIBRARIES}
	)
