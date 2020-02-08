cmake_minimum_required(VERSION 2.8.6)
project(dummy_nodes)

find_package(catkin REQUIRED
    COMPONENTS
    rospy
    roscpp
    roslib
    testbed_msgs
    )

find_package(Eigen3 REQUIRED NO_MODULE)
find_package(yaml-cpp REQUIRED )

catkin_package(
    INCLUDE_DIRS include/
    CATKIN_DEPENDS
    rospy
    roscpp
    roslib
    testbed_msgs
    message_runtime
    )

include_directories(
    include/
    ${catkin_INCLUDE_DIRS}
    ${YAML_CPP_INCLUDE_DIR}
    )

add_executable(dummy_anchors
    exec/dummy_anchors_node.cpp
    src/dummy_anchors.cpp
    )

add_dependencies(dummy_anchors
    ${catkin_EXPORTED_TARGETS}
    )

target_link_libraries(dummy_anchors
    ${catkin_LIBRARIES}
    ${YAML_CPP_LIBRARIES}
    Eigen3::Eigen
    yaml-cpp
    )


