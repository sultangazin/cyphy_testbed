#include <ros/ros.h>
#include "simulator/simulator.hpp"

int main(int argc, char** argv) {
    ros::init(argc, argv, "simulator_node");
    ros::NodeHandle nl;

    ROS_INFO("Simulator Node");

    XSimulator sim_n;

    if (!sim_n.Initialize(nl)) {
        ROS_ERROR("%s: Failed to initialize the node!\n", 
                ros::this_node::getName().c_str());
        return EXIT_FAILURE;
    }
    ROS_INFO("Started");
    
    ros::spin();

    return EXIT_SUCCESS;
}
