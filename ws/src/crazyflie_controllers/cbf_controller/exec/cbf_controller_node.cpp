#include <ros/ros.h>
#include "cbf_controller/cbf_controller_ros.hpp"

int main(int argc, char** argv) {
  ros::init(argc, argv, "cbf_controller");
  ros::NodeHandle n("~");

  CBFControllerROS controller;

  if (!controller.Initialize(n)) {
    ROS_ERROR("%s: Failed to initialize CBF controller.",
              ros::this_node::getName().c_str());
    return EXIT_FAILURE;
  }

  ros::spin();

  return EXIT_SUCCESS;
}
