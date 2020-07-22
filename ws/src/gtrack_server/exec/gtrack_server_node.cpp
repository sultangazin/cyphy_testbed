#include <ros/ros.h>
#include "gtrack_server/gtrack_server.hpp"

int main(int argc, char** argv) {
    ros::init(argc, argv, "gtrack_server_node");
    ros::NodeHandle nl;

    ROS_INFO("RPC Server Node");

    GTrackServer rpc_server;

    if (!rpc_server.Initialize(nl)) {
        ROS_ERROR("%s: Failed to initialize the node!\n", 
                ros::this_node::getName().c_str());
        return EXIT_FAILURE;
    }
    ROS_INFO("Started");

    ros::spin();

    return EXIT_SUCCESS;
}
