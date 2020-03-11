#include <ros/ros.h>
#include "dummy_anchors/dummy_anchors.hpp"

int main(int argc, char** argv) {
    ros::init(argc, argv, "dummy_anchors_node");
    ros::NodeHandle nl;

    ROS_INFO("Starting Dummy Anchor Node");

    DummyAnchors da;

    if (!da.Initialize(nl)) {
        ROS_ERROR("%s: Failed to initialize the node!\n", 
                ros::this_node::getName().c_str());
        return EXIT_FAILURE;
    }

    ros::spin();

    return EXIT_SUCCESS;
}
