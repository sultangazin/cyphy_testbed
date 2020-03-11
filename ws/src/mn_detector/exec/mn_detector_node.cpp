#include <ros/ros.h>
#include "mn_detector/mn_detector.hpp"

int main(int argc, char** argv) {
    ros::init(argc, argv, "mn_detector");
    ros::NodeHandle nl;

    ROS_INFO("Starting Malicious Node detector Node");

    MNDetector da;

    if (!da.Initialize(nl)) {
        ROS_ERROR("%s: Failed to initialize the node!\n", 
                ros::this_node::getName().c_str());
        return EXIT_FAILURE;
    }

    ros::spin();

    return EXIT_SUCCESS;
}
